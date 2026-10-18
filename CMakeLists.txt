cmake_minimum_required(VERSION 3.24)
project(RCTrainerSlice LANGUAGES CXX)

add_executable(RCTrainer WIN32 src/main.cpp src/rc_trainer.rc src/flight_math.h src/flight_dynamics.cpp src/flight_dynamics.h src/flight_test_report.cpp src/flight_test_report.h src/trajectory_diagnostics.cpp src/trajectory_diagnostics.h src/route_pilot.cpp src/route_pilot.h src/troubleshooting_feed.cpp src/troubleshooting_feed.h src/environment_renderer.cpp src/environment_renderer.h src/ground_dynamics.cpp src/ground_dynamics.h src/ground_renderer.cpp src/ground_renderer.h src/propulsion.cpp src/propulsion.h src/aircraft_model.cpp src/aircraft_model.h src/textured_aircraft_renderer.cpp src/textured_aircraft_renderer.h)
target_compile_features(RCTrainer PRIVATE cxx_std_20)
target_compile_definitions(RCTrainer PRIVATE UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN DIRECTINPUT_VERSION=0x0800)
target_link_libraries(RCTrainer PRIVATE dinput8 dxguid winmm comctl32 msimg32 gdiplus)
add_custom_command(TARGET RCTrainer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:RCTrainer>/assets")

add_executable(PpmProbe tools/ppm_probe.cpp)
target_compile_features(PpmProbe PRIVATE cxx_std_20)
target_compile_definitions(PpmProbe PRIVATE UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(PpmProbe PRIVATE winmm)

add_executable(GamepadProbe tools/gamepad_probe.cpp)
target_compile_features(GamepadProbe PRIVATE cxx_std_20)
target_compile_definitions(GamepadProbe PRIVATE UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN DIRECTINPUT_VERSION=0x0800)
target_link_libraries(GamepadProbe PRIVATE dinput8 dxguid)

if(MSVC)
  target_compile_options(RCTrainer PRIVATE /W4 /permissive- /EHsc /utf-8)
  target_compile_options(PpmProbe PRIVATE /W4 /permissive- /EHsc /utf-8)
  target_compile_options(GamepadProbe PRIVATE /W4 /permissive- /EHsc /utf-8)
endif()
