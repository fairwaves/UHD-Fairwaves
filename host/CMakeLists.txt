#
# Copyright 2012-2014 Fairwaves
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
## Build the UmTRX UHD module
########################################################################
cmake_minimum_required(VERSION 2.8)
project(UmTRX-UHD)

########################################################################
# extract version info from git
########################################################################
include(${PROJECT_SOURCE_DIR}/cmake/GetGitRevisionDescription.cmake)
git_describe(UMTRX_VERSION --dirty)
string(REPLACE "g" "" UMTRX_VERSION ${UMTRX_VERSION}) #remove hash prefix g
message(STATUS "UMTRX_VERSION: ${UMTRX_VERSION}")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/umtrx_version.in.hpp"
    "${CMAKE_CURRENT_BINARY_DIR}/umtrx_version.hpp"
    IMMEDIATE @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

########################################################################
## Create a list of module sources
########################################################################
list(APPEND UMTRX_SOURCES
    umtrx_impl.cpp
    umtrx_monitor.cpp
    umtrx_io_impl.cpp
    umtrx_find.cpp
    umtrx_iface.cpp
    umtrx_eeprom.cpp
    lms6002d.cpp
    lms6002d_ctrl.cpp
    tmp102_ctrl.cpp
    ads1015_ctrl.cpp
    power_amp.cpp
    umtrx_fifo_ctrl.cpp
    missing/platform.cpp #not properly exported from uhd, so we had to copy it
    cores/rx_frontend_core_200.cpp
    cores/tx_frontend_core_200.cpp
    cores/rx_dsp_core_200.cpp
    cores/tx_dsp_core_200.cpp
    cores/time64_core_200.cpp
    cores/validate_subdev_spec.cpp
    cores/apply_corrections.cpp
    umsel2_ctrl.cpp
)

########################################################################
## Find UHD dev environment
########################################################################
#expand pkg-config search path to include CMAKE_INSTALL_PREFIX
set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/pkgconfig/:$ENV{PKG_CONFIG_PATH}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(UHD REQUIRED)

INCLUDE_DIRECTORIES(${UHD_INCLUDE_DIRS})
list(APPEND UMTRX_LIBRARIES ${UHD_LIBRARIES})

MESSAGE(STATUS "UHD include directories: ${UHD_INCLUDE_DIRS}")
MESSAGE(STATUS "UHD libraries: ${UHD_LIBRARIES}")

if (EXISTS "${UHD_INCLUDE_DIRS}/uhd/utils/msg.hpp")
    add_definitions(-DUHD_HAS_MSG_HPP)
    message(STATUS "  use msg.hpp for logging")
else()
    message(STATUS "  use log.hpp for logging")
endif()

if (EXISTS "${UHD_INCLUDE_DIRS}/uhd/utils/thread.hpp")
    add_definitions(-DTHREAD_PRIORITY_HPP_DEPRECATED)
    message(STATUS "  use thread.hpp (new interface)")
else()
    message(STATUS "  use thread_priority.hpp (old interface)")
endif()

########################################################################
# Setup Boost
########################################################################
MESSAGE(STATUS "")
MESSAGE(STATUS "Configuring Boost C++ Libraries...")
SET(BOOST_REQUIRED_COMPONENTS
    date_time
    filesystem
    system
    thread
    program_options
)

IF(UNIX AND EXISTS "/usr/lib64")
    LIST(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
ENDIF(UNIX AND EXISTS "/usr/lib64")

IF(MSVC)
    SET(BOOST_ALL_DYN_LINK "${BOOST_ALL_DYN_LINK}" CACHE BOOL "boost enable dynamic linking")
    IF(BOOST_ALL_DYN_LINK)
        ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK) #setup boost auto-linking in msvc
    ELSE(BOOST_ALL_DYN_LINK)
        UNSET(BOOST_REQUIRED_COMPONENTS) #empty components list for static link
    ENDIF(BOOST_ALL_DYN_LINK)
ENDIF(MSVC)

SET(Boost_ADDITIONAL_VERSIONS
    "1.35.0" "1.35" "1.36.0" "1.36" "1.37.0" "1.37" "1.38.0" "1.38" "1.39.0" "1.39"
    "1.40.0" "1.40" "1.41.0" "1.41" "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
)
FIND_PACKAGE(Boost 1.36 REQUIRED COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
list(APPEND UMTRX_LIBRARIES ${Boost_LIBRARIES})

MESSAGE(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
MESSAGE(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
MESSAGE(STATUS "Boost libraries: ${Boost_LIBRARIES}")

#additional pthread requirement that may not be automatically included by boost
if (UNIX)
    set(CMAKE_THREAD_PREFER_PTHREAD ON)
    find_package(Threads)
    list(APPEND UMTRX_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
endif()

#make boost property tree thread safe
#http://stackoverflow.com/questions/8156948/is-boostproperty-treeptree-thread-safe
add_definitions(-DBOOST_SPIRIT_THREADSAFE)

########################################################################
# Helpful compiler flags
########################################################################
if(CMAKE_COMPILER_IS_GNUCXX)

    #force a compile-time error when symbols are missing
    set(CMAKE_MODULE_LINKER_FLAGS "-Wl,--no-undefined")

    #common warnings to help encourage good coding practices
    add_definitions(-Wall)
    add_definitions(-Wextra)

    #symbols are only exported from libraries/modules explicitly
    add_definitions(-fvisibility=hidden)

endif()

########################################################################
# UHD compatibility checks
########################################################################
message(STATUS "Checking uhd::device::register_device() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/device.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/device.hpp device_hpp)
string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
if ("${has_device_filter}" STREQUAL "-1")
    message(STATUS "  has original API")
else()
    add_definitions(-DUHD_HAS_DEVICE_FILTER)
    message(STATUS "  has filter API")
endif()

message(STATUS "Checking uhd::property::set_publisher() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp property_tree_hpp)
string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
if ("${has_set_publisher}" STREQUAL "-1")
    message(STATUS "  missing set_publisher() API")
else()
    add_definitions(-Dpublish=set_publisher)
    add_definitions(-Dsubscribe=add_desired_subscriber)
    add_definitions(-Dcoerce=add_coerced_subscriber)
    message(STATUS "  has set_publisher() API")
endif()

########################################################################
# Build the UmTRX module
########################################################################
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
add_library(umtrx MODULE ${UMTRX_SOURCES})
target_link_libraries(umtrx ${UMTRX_LIBRARIES})

########################################################################
# Install the UmTRX module
########################################################################
install(
    TARGETS umtrx
    DESTINATION lib${LIB_SUFFIX}/uhd/modules
)

add_subdirectory(utils)

########################################################################
# uninstall target
########################################################################
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
    set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "UmTRX UHD support module")
set(CPACK_PACKAGE_VENDOR              "Fairwaves")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION ${UMTRX_VERSION})
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-all-dev, uhd")
SET(CPACK_DEBIAN_PACKAGE_RECOMMENDS "python")
SET(CPACK_PACKAGE_FILE_NAME "umtrx_${UMTRX_VERSION}_${CMAKE_SYSTEM_PROCESSOR}")
SET(CPACK_SET_DESTDIR "ON")
include(CPack) #include last
