# - Try to find gtsam lib
#
# Once done this will define
#
#  GTSAM_FOUND - system has gtsam lib with correct version
#  GTSAM_INCLUDE_DIRS - the gtsam include directory
#  GTSAM_LIBRARIES - the gtsam library
#  GTSAM_UNSTABLE_LIBRARIES - the gtsam unstable library

find_path(GTSAM_INCLUDE_DIR "gtsam/inference/FactorGraph.h")

unset(GTSAM_LIBRARIES)
unset(GTSAM_INCLUDE_DIRS)

if (NOT GTSAM_FIND_COMPONENTS)
    set(GTSAM_FIND_COMPONENTS gtsam)
endif()

if (GTSAM_FIND_COMPONENTS)
    foreach (comp ${GTSAM_FIND_COMPONENTS})
        find_library(GTSAM_${comp}_LIBRARY ${comp})
        if (GTSAM_${comp}_LIBRARY)
            set(GTSAM_${comp}_FOUND 1)
            mark_as_advanced(GTSAM_${comp}_LIBRARY)
            set(GTSAM_${comp}_LIBRARIES ${GTSAM_${comp}_LIBRARY})
            list(APPEND GTSAM_LIBRARIES ${GTSAM_${comp}_LIBRARY})
        endif ()
    endforeach ()
endif ()

find_package(Boost COMPONENTS serialization system filesystem thread date_time regex timer chrono QUIET)

include(FindPackageHandleStandardArgs)

if(GTSAM_INCLUDE_DIR)
    file(READ ${GTSAM_INCLUDE_DIR}/gtsam/config.h GTSAM_VERSION_STRING)
    string(REGEX MATCH "#define GTSAM_VERSION_STRING *\".*([0-9]+).([0-9]+).([0-9]+)\"" _ ${GTSAM_VERSION_STRING})
    set(GTSAM_VERSION_MAJOR ${CMAKE_MATCH_1})
    set(GTSAM_VERSION_MINOR ${CMAKE_MATCH_2})
    set(GTSAM_VERSION_PATCH ${CMAKE_MATCH_3})
    set(GTSAM_VERSION ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
endif(GTSAM_INCLUDE_DIR)

find_package_handle_standard_args(GTSAM REQUIRED_VARS GTSAM_INCLUDE_DIR Boost_LIBRARIES HANDLE_COMPONENTS)

mark_as_advanced(GTSAM_LIBRARY GTSAM_UNSTABLE_LIBRARY GTSAM_INCLUDE_DIR)

set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})

if (GTSAM_FOUND)
    foreach (comp ${GTSAM_FIND_COMPONENTS})
        if(GTSAM_${comp}_FOUND)
            if(NOT TARGET GTSAM::${comp})
                add_library(GTSAM::${comp} UNKNOWN IMPORTED)
                set_target_properties(GTSAM::${comp} PROPERTIES
                    IMPORTED_LOCATION "${GTSAM_${comp}_LIBRARY}"
                    INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIR}"
                    INTERFACE_LINK_LIBRARIES "Boost::serialization;Boost::system;Boost::filesystem;Boost::thread;Boost::date_time;Boost::regex;Boost::timer;Boost::chrono;"
                )
            endif()
        endif()
    endforeach()
endif ()

