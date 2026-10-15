cmake_minimum_required(VERSION 3.24)
project(RCTrainerSlice LANGUAGES CXX)

add_executable(RCTrainer WIN32 src/main.cpp)
target_compile_features(RCTrainer PRIVATE cxx_std_20)
target_compile_definitions(RCTrainer PRIVATE UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN DIRECTINPUT_VERSION=0x0800)
target_link_libraries(RCTrainer PRIVATE dinput8 dxguid winmm comctl32)

add_executable(PpmProbe tools/ppm_probe.cpp)
target_compile_features(PpmProbe PRIVATE cxx_std_20)
target_compile_definitions(PpmProbe PRIVATE UNICODE _UNICODE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(PpmProbe PRIVATE winmm)

if(MSVC)
  target_compile_options(RCTrainer PRIVATE /W4 /permissive- /EHsc /utf-8)
  target_compile_options(PpmProbe PRIVATE /W4 /permissive- /EHsc /utf-8)
endif()

