cmake_minimum_required(VERSION 3.5)
project(AKFSFSimulation)

set(CMAKE_CXX_STANDARD 11)

include_directories("/usr/include/eigen3"
        "${CMAKE_SOURCE_DIR}/include" )
find_package(Eigen3 REQUIRED NO_MODULE)

set(SRC_CPP
        main.cpp
        Logger.cpp
        MARGData.cpp
        utils.cpp
        kalmanfilter.cpp
        DataLoader.cpp
)

add_executable(${PROJECT_NAME} ${SRC_CPP})
target_link_libraries(${PROJECT_NAME}
        SDL2
        SDL2_ttf
)

if (MINGW)
    set_target_properties(${PROJECT_NAME} PROPERTIES
            LINK_FLAGS "-Wl,--subsystem,console"
    )
endif()
