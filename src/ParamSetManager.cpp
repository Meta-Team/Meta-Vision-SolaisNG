//
// Created by liuzikai on 3/6/21.
//

#include "ParamSetManager.h"
#include "Utilities.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <google/protobuf/util/json_util.h>
#include <boost/asio/ip/host_name.hpp>
#include <fstream>

namespace meta {

// PARAM_SET_ROOT defined in CMakeLists.txt
ParamSetManager::ParamSetManager()
        : paramSetRoot(fs::path(PARAM_SET_ROOT) / "params"), defaultParamSetName(boost::asio::ip::host_name()) {
    std::cout << "Root"<<paramSetRoot<<std::endl;
    std::cout << "ParamSetManager: using default ParamSet " << defaultParamSetName << ".json" << std::endl;

    if (!fs::exists(paramSetRoot / ".." / "params_backup")) {
        fs::create_directories(paramSetRoot / ".." / "params_backup");
    }
}

void ParamSetManager::reloadParamSetList() {
    paramsSetNames.clear();

    if (!fs::is_directory(paramSetRoot) || !fs::exists(paramSetRoot / (defaultParamSetName + ".json"))) {
        fs::create_directories(paramSetRoot);

        // Create default parameter file
        ParamSet params;

        params.set_image_width(1280);
        params.set_image_height(720);
        params.set_roi_width(1280);
        params.set_roi_height(720);
        params.set_enemy_color(ParamSet::BLUE);
        params.set_video_speed(1);
        params.set_video_playback_speed(1);

        params.set_camera_backend(ParamSet::OPENCV);
        params.set_camera_id(0);
        params.set_fps(120);
        params.set_allocated_gamma(allocToggledFloat(false));
        params.set_allocated_manual_exposure(allocToggledInt(false));

        params.set_brightness_threshold(155);

        params.set_color_threshold_mode(ParamSet::RB_CHANNELS);
        params.set_allocated_hsv_red_hue(allocFloatRange(150, 30));  // across the 0 (180) point
        params.set_allocated_hsv_blue_hue(allocFloatRange(90, 150));
        params.set_rb_channel_threshold(55);
        params.set_allocated_contour_erode(allocToggledInt(false, 3));
        params.set_allocated_contour_dilate(allocToggledInt(false, 3));

        params.set_allocated_contour_open(allocToggledInt(true, 3));
        params.set_allocated_contour_close(allocToggledInt(true, 3));
        params.set_contour_fit_function(ParamSet::ELLIPSE);
        params.set_allocated_contour_pixel_count(allocToggledFloat(true, 15));
        params.set_allocated_contour_min_area(allocToggledFloat(false, 3));
        params.set_allocated_long_edge_min_length(allocToggledInt(true, 30));
        params.set_allocated_light_aspect_ratio(allocToggledFloatRange(true, 2, 30));
        params.set_allocated_light_max_rotation(allocToggledFloat(true, 15));

        params.set_allocated_light_length_max_ratio(allocToggledFloat(true, 1.5));
        params.set_allocated_light_x_dist_over_l(allocToggledFloatRange(false, 1, 3));
        params.set_allocated_light_y_dist_over_l(allocToggledFloatRange(false, 0, 1));
        params.set_allocated_light_angle_max_diff(allocToggledFloat(true, 10));
        params.set_allocated_small_armor_aspect_ratio(allocFloatRange(1.25, 2));
        params.set_allocated_large_armor_aspect_ratio(allocFloatRange(2, 5));
        params.set_allocated_manual_pnp_rect_max_height(allocToggledInt(false, 50));
        params.set_dist_manual_offset(45);

        params.set_allocated_small_armor_size(allocIntPair(120, 60));
        params.set_allocated_large_armor_size(allocIntPair(240, 60));

        params.set_pulse_min_x_offset(500);
        params.set_pulse_max_y_offset(300);
        params.set_pulse_min_interval(43);
        params.set_allocated_tk_threshold(allocIntPair(3, 1500));
        params.set_tk_compute_period_using_pulses(2);
        params.set_tk_target_dist_offset(-50);
        params.set_tracking_life_time(40);
        params.set_allocated_manual_delta_offset(allocFloatPair(0, 0));

        std::cout << "ParamSetManager: create default ParamSet " << defaultParamSetName << ".json" << std::endl;
        saveParamSetToJson(params, paramSetRoot / (defaultParamSetName + ".json"));
    }

    for (const auto &entry : fs::directory_iterator(paramSetRoot)) {
#ifdef BOOST_OS_WINDOWS
        if (wcscmp(entry.path().extension().c_str(), L".json") == 0) {
#else
        if (strcasecmp(entry.path().extension().c_str(), ".json") == 0) {
#endif
            paramsSetNames.emplace_back(entry.path().stem().string());
        }
    }

    curParamSetName = defaultParamSetName;  // switch to default
}

ParamSet ParamSetManager::loadCurrentParamSet() const {
    ParamSet p;
    auto filename = paramSetRoot / (curParamSetName + ".json");

    std::ifstream file(filename.string());
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto status = JsonStringToMessage(content, &p, google::protobuf::util::JsonParseOptions());
    if (!status.ok()) {
        std::cerr << "Failed to load " << filename << ": " << status.message() << std::endl;
        return ParamSet();
    } else {
        return p;
    }
}

void ParamSetManager::saveCurrentParamSet(const ParamSet &p) {
    saveParamSetToJson(p, paramSetRoot / (curParamSetName + ".json"));

    // Backup
    fs::path filename = paramSetRoot / ".." / "params_backup" / fs::path(curParamSetName + "_" + currentTimeString() + ".json");
    saveParamSetToJson(p, filename);  // simply overwrite if exists
}

void ParamSetManager::saveParamSetToJson(const ParamSet &p, const fs::path &filename) {
    std::string content;

    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;
    options.preserve_proto_field_names = true;
    MessageToJsonString(p, &content, options);

    std::ofstream(filename.string()) << content;
}

}