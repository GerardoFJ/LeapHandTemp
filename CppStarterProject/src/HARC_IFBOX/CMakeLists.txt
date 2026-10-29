cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME HARC_IFBOX)
set(PROJECT_DESCRIPTION "Acquisition library for HARC IFBOX sensors")
set(PROJECT_URL https://github.com/isri-aist/HARC_IFBOX)
set(PROJECT_VERSION 1.0.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Disable some of jrl-cmakemodules features
set(INSTALL_DOCUMENTATION
    OFF
    CACHE BOOL "")
set(INSTALL_GENERATED_HEADERS
    OFF
    CACHE BOOL "")
set(INSTALL_PKG_CONFIG_FILE
    OFF
    CACHE BOOL "")

# Disable -Werror for now
set(CXX_DISABLE_WERROR ON)

include(cmake/base.cmake)
project(${PROJECT_NAME} LANGUAGES CXX)

find_package(serial REQUIRED)

add_project_dependency(yaml-cpp 0.5.0 REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_CPP_TARGET yaml-cpp)
endif()

set(lib_INCLUDE include/${PROJECT_NAME})
set(lib_HEADERS
    ${lib_INCLUDE}/HARC_IFBOX.h ${lib_INCLUDE}/Capture.h
    ${lib_INCLUDE}/DataMatrix.h ${lib_INCLUDE}/Patch.h
    ${lib_INCLUDE}/Configuration.h)
set(lib_SRC src/HARC_IFBOX.cpp src/Capture.cpp src/Configuration.cpp)

add_library(${PROJECT_NAME} SHARED ${lib_SRC} ${lib_HEADERS})

target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>                   
  $<INSTALL_INTERFACE:include>     
  ${serial_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${serial_LIBRARIES}
  ${YAML_CPP_TARGET}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(FILES ${lib_HEADERS} DESTINATION ${lib_INCLUDE})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

set(CURSES_NEED_NCURSES ON)
find_package(Curses REQUIRED)
add_executable(viewer_test tests/viewer_test.cpp)
target_link_libraries(viewer_test PUBLIC ${PROJECT_NAME} ${CURSES_LIBRARIES})
