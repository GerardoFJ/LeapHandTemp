# This is a sample CMakeLists.txt that builds HaptX's CppStarterProject.
# Modify this file to suit your own needs, or use it as a jumping off point for
# another build system.

cmake_minimum_required(VERSION 3.5)
project(CppStarterProject)

option(USE_SHARED_LIB "Use HaptxApi DLL instead of static library" OFF)

# The HaptX SDK requires C++17.
set(CMAKE_CXX_STANDARD 17)

# Supported build types are "Debug" and "Release".
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(CppStarterProject main.cpp)

# The HaptX SDK Installer should set the HAPTX_SDK_HOME environmental
# variable for you.  If it has been unset for some reason, just substitute
# the path where you installed the SDK.
target_include_directories(CppStarterProject PRIVATE $ENV{HAPTX_SDK_HOME}/include)

if (MSVC AND USE_SHARED_LIB)
  target_link_libraries(CppStarterProject PRIVATE $ENV{HAPTX_SDK_HOME}/Dll/${CMAKE_BUILD_TYPE}/${CMAKE_IMPORT_LIBRARY_PREFIX}HaptxApi${CMAKE_IMPORT_LIBRARY_SUFFIX})
  target_compile_definitions(CppStarterProject PUBLIC HAPTXAPI_DLLEXPORT=__declspec\(dllimport\))
  add_custom_command(TARGET CppStarterProject POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different "$ENV{HAPTX_SDK_HOME}/Dll/${CMAKE_BUILD_TYPE}/HaptxApi.dll" "$<TARGET_FILE_DIR:CppStarterProject>"
  )
else()
  target_link_libraries(CppStarterProject PRIVATE
    $ENV{HAPTX_SDK_HOME}/Lib/${CMAKE_BUILD_TYPE}/${CMAKE_STATIC_LIBRARY_PREFIX}HaptxApi${CMAKE_STATIC_LIBRARY_SUFFIX}
    $ENV{HAPTX_SDK_HOME}/Lib/${CMAKE_BUILD_TYPE}/${CMAKE_STATIC_LIBRARY_PREFIX}openvr_api${CMAKE_STATIC_LIBRARY_SUFFIX}
  )

  if (MSVC)
    target_link_libraries(CppStarterProject PRIVATE wsock32.lib ws2_32.lib Iphlpapi.lib)
  else()
    target_link_libraries(CppStarterProject PRIVATE pthread dl)
  endif()
endif()

add_custom_command(TARGET CppStarterProject POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$ENV{HAPTX_SDK_HOME}/ThirdParty/OpenVR/openvr_api.dll" "$<TARGET_FILE_DIR:CppStarterProject>")

if (MSVC)
  target_compile_definitions(CppStarterProject PUBLIC _USE_MATH_DEFINES _MBCS)
  target_compile_options(CppStarterProject PRIVATE "/wd4251")
endif()
