include(FindPackageHandleStandardArgs)

# Try getting ROS_DISTRO safely
if(NOT DEFINED ROS_DISTRO)
    find_program(ROSVERSION rosversion)
    if(ROSVERSION)
        execute_process(COMMAND ${ROSVERSION} -d OUTPUT_VARIABLE ROS_DISTRO OUTPUT_STRIP_TRAILING_WHITESPACE)
    elseif(DEFINED ENV{ROS_DISTRO})
        set(ROS_DISTRO $ENV{ROS_DISTRO})
    else()
        set(ROS_DISTRO "")
    endif()
endif()

set(_MAVLINK_EXTRA_SEARCH_HINTS
    ${CMAKE_SOURCE_DIR}/mavlink/
    ../../mavlink/
    ../mavlink/
    ${CMAKE_CURRENT_BINARY_DIR}/../mavlink/
    )

set(_MAVLINK_EXTRA_SEARCH_PATHS
    /usr/
    /usr/local/
    )

# look in hints first
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink_types.h
    PATH_SUFFIXES include
    HINTS ${_MAVLINK_EXTRA_SEARCH_HINTS}
    NO_DEFAULT_PATH
)

# then system-wide
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink_types.h
    PATH_SUFFIXES include/mavlink
    PATHS ${_MAVLINK_EXTRA_SEARCH_PATHS}
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

# then ROS 2 install space
if(ROS_DISTRO)
    find_path(_MAVLINK_INCLUDE_DIR
        NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
        PATH_SUFFIXES include
        PATHS /opt/ros/${ROS_DISTRO}/
    )
endif()

# detect version
if(EXISTS ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h)
    file(READ ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h MAVLINK_CONFIG_FILE)
    string(REGEX MATCH "#define MAVLINK_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\"" _MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
    set(MAVLINK_VERSION "${CMAKE_MATCH_1}")
else()
    set(MAVLINK_VERSION "2.0")
endif()

set(MAVLINK_INCLUDE_DIRS ${_MAVLINK_INCLUDE_DIR})

find_package_handle_standard_args(MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
    VERSION_VAR MAVLINK_VERSION
)
