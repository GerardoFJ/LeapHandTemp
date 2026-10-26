cmake_minimum_required(VERSION 3.16)
project(myproj CXX)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11 GIT_TAG v2.4.2)
FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG yaml-cpp-0.9.0)
FetchContent_MakeAvailable(cli11 yaml-cpp)

add_library(options INTERFACE)
target_include_directories(options INTERFACE libs)
target_link_libraries(options INTERFACE CLI11::CLI11 yaml-cpp::yaml-cpp)

foreach(src test)
  add_executable(${src} src/${src}.cpp)
  target_link_libraries(${src} PRIVATE options)
endforeach()
