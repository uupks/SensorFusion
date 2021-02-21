list(APPEND CMAKE_MODULE_PATH /usr/local/share/cmake/GeographicLib)
find_package (GeographicLib REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES})