cmake_minimum_required(VERSION 3.10)

project(viewer_test LANGUAGES CXX)

enable_testing()

set(CURSES_NEED_NCURSES ON)
find_package(Curses REQUIRED)
find_package(HARC_IFBOX REQUIRED)
add_executable(viewer_test viewer_test.cpp)
target_link_libraries(viewer_test PUBLIC HARC_IFBOX::HARC_IFBOX
                                         ${CURSES_LIBRARIES})

install(CODE "message(\"Install done\")")
