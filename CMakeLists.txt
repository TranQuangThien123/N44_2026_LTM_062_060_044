cmake_minimum_required(VERSION 3.15)
project(DroneWinsock C)

set(CMAKE_C_STANDARD 99)

include_directories(include)

# Link Windows Sockets Library
link_libraries(ws2_32)

add_executable(test_udp src/test_udp.c)
add_executable(telemetry_parser src/telemetry_parser.c)
add_executable(control_uav src/control_uav.c)
add_executable(mission_with_network_metrics src/mission_with_network_metrics.c)
