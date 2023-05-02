//
// Created by liuzikai on 3/6/21.
//

#ifndef META_VISION_SOLAIS_PARAMSETMANAGER_H
#define META_VISION_SOLAIS_PARAMSETMANAGER_H

#include "Parameters.h"
#include <filesystem>

namespace meta {

namespace fs = std::filesystem;

class ParamSetManager {
public:

    ParamSetManager();

    void reloadParamSetList();  // switch to default

    const std::vector<std::string> &getParamSetList() const { return paramsSetNames; }

    void switchToParamSet(const std::string &paramSetName) { curParamSetName = paramSetName; }

    const std::string &currentParamSetName() const { return curParamSetName; }

    ParamSet loadCurrentParamSet() const;

    void saveCurrentParamSet(const ParamSet &p);

private:

    const fs::path paramSetRoot;

    const std::string defaultParamSetName;      // use host name as default (without the extension)

    std::vector<std::string> paramsSetNames;    // json filenames without the extension
    std::string curParamSetName;                // json filenames without the extension

    void saveParamSetToJson(const ParamSet &p, const fs::path &filename);
};

}

#endif //META_VISION_SOLAIS_PARAMSETMANAGER_H
