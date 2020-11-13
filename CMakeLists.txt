cmake_minimum_required(VERSION 3.12)
project(yarp-ros2
        LANGUAGES CXX
        VERSION 3.4.100)

include(FeatureSummary)

find_package(YCM 0.12 REQUIRED)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "yarp-ros2 is always built with dynamic plugins")
set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "Build libraries as shared as opposed to static")

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

include(AddUninstallTarget)

find_package(YARP 3.4.1 COMPONENTS os sig dev REQUIRED)
set(CMAKE_C_FLAGS "${YARP_C_FLAGS} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${YARP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

yarp_configure_plugins_installation(yarp-ros2)

add_subdirectory(src)

#find_package(ament_cmake REQUIRED)

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  ## the following line skips the linter which checks for copyrights
  ## uncomment the line when a copyright and license is not present in all source files
  ##set(ament_cmake_copyright_FOUND TRUE)
  ## the following line skips cpplint (only works in a git repo)
  ## uncomment the line when this package is not in a git repo
  ##set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

#ament_package()

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
