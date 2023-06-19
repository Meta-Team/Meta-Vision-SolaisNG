if(Qt5_FOUND
   AND Boost_FOUND
   AND TARGET libParameters
   AND TARGET libTerminalSocket
)
    # message("=> Target SolaisTerminal is available to build")

    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)

    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Parameters.ui.h
        COMMAND
            python3 ${CMAKE_CURRENT_SOURCE_DIR}/GeneratePhaseUI.py
            ${PROJECT_SOURCE_DIR}/src/Parameters.proto > ${CMAKE_CURRENT_BINARY_DIR}/Parameters.ui.h
        MAIN_DEPENDENCY ${PROJECT_SOURCE_DIR}/src/Parameters.proto
        DEPENDS GeneratePhaseUI.py
    )

    add_executable(
        SolaisTerminal
        main.cpp
        MainWindow.ui
        MainWindow.cpp
        CollapsibleGroupBox.hpp
        # AnnotatedMat.cpp
        # AnnotatedMatViewer.ui
        # AnnotatedMatViewer.cpp
        # QAsioEventDispatcher.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/Parameters.ui.h
    )
    target_link_libraries(
        SolaisTerminal
        # ${OpenCV_LIBRARIES}
        ${QT_LIBS_QUALIFIED}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        libParameters
        libTerminalSocket
    )
    target_include_directories(
        SolaisTerminal PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}
                               ${CMAKE_CURRENT_SOURCE_DIR}
    )
else()
    message(
        "=> Target SolaisTerminal is not available to build. Depends: Qt, Boost, libParameters, libTerminalSocket"
    )
endif()
