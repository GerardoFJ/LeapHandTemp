cmake_minimum_required(VERSION 3.16)
project(myproj CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11 GIT_TAG v2.4.2)
FetchContent_Declare(yaml-cpp GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git GIT_TAG yaml-cpp-0.9.0)
FetchContent_MakeAvailable(cli11 yaml-cpp)

add_library(options INTERFACE)
target_include_directories(options INTERFACE libs)
target_link_libraries(options INTERFACE CLI11::CLI11 yaml-cpp::yaml-cpp)

# ---- HaptX SDK (only when building for Windows with MSVC/Wine) ----
if(WIN32)
  set(HAPTX_SDK "/root/.wine/drive_c/Program Files/HaptX/SDK" CACHE PATH "HaptX SDK root")

  add_library(haptx INTERFACE)
  target_include_directories(haptx INTERFACE "${HAPTX_SDK}/Include")
  target_compile_definitions(haptx INTERFACE "HAPTXAPI_DLLEXPORT=__declspec(dllimport)")
  target_compile_options(haptx INTERFACE /wd4251)
  target_link_libraries(haptx INTERFACE "${HAPTX_SDK}/Dll/Release/HaptxApi.lib" user32 gdi32)
endif()

# ---- Executables ----
foreach(src ik_stream ik_stream_feedback)
  add_executable(${src} src/${src}.cpp)
  target_link_libraries(${src} PRIVATE options)

  if(WIN32)
    target_link_libraries(${src} PRIVATE haptx)
    add_custom_command(TARGET ${src} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${HAPTX_SDK}/Dll/Release/HaptxApi.dll"
        "${HAPTX_SDK}/ThirdParty/OpenVR/openvr_api.dll"
        $<TARGET_FILE_DIR:${src}>)
  endif()
endforeach()
