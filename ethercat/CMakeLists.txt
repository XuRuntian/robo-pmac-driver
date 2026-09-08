cmake_minimum_required(VERSION 3.20)
project(continuum_ethercat_offline VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(CTest)
option(ENABLE_SANITIZERS "Instrument project code for offline checks" ON)
set(IGH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/build/igh-install" CACHE PATH "Local IgH installation")
find_path(IGH_INCLUDE ecrt.h PATHS "${IGH_PREFIX}/include" NO_DEFAULT_PATH REQUIRED)
find_library(IGH_REAL ethercat PATHS "${IGH_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(IGH_FAKE fakeethercat PATHS "${IGH_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(IGH_RTIPC rtipc PATHS "${IGH_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_library(offline_options INTERFACE)
target_include_directories(offline_options INTERFACE include "${IGH_INCLUDE}" "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_compile_options(offline_options INTERFACE -Wall -Wextra -Wpedantic -Werror -fno-strict-aliasing)
if(ENABLE_SANITIZERS)
  target_compile_options(offline_options INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(offline_options INTERFACE -fsanitize=address,undefined)
endif()

add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/drive_config.hpp"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/import_pmac.py"
    --check --header "${CMAKE_CURRENT_BINARY_DIR}/generated/drive_config.hpp"
  DEPENDS scripts/import_pmac.py config/pmac_drives.json
  VERBATIM)
add_library(igh_config OBJECT src/igh_config.cpp "${CMAKE_CURRENT_BINARY_DIR}/generated/drive_config.hpp")
target_link_libraries(igh_config PRIVATE offline_options)

add_executable(igh_abi_check src/abi_check.cpp)
target_link_libraries(igh_abi_check PRIVATE offline_options "${IGH_REAL}")
add_executable(igh_fake_smoke src/fake_smoke.cpp $<TARGET_OBJECTS:igh_config>)
target_link_libraries(igh_fake_smoke PRIVATE offline_options "${IGH_FAKE}" ${CMAKE_DL_LIBS})
# Keep RtIPC as a direct dependency: executable RUNPATH does not resolve the
# dependencies of libfakeethercat (which does not ship an upstream RUNPATH).
target_link_options(igh_fake_smoke PRIVATE "LINKER:--no-as-needed")
target_link_libraries(igh_fake_smoke PRIVATE "${IGH_RTIPC}")
set_target_properties(igh_fake_smoke PROPERTIES BUILD_RPATH "${IGH_PREFIX}/lib")
add_executable(offline_core_tests tests/core_tests.cpp)
target_link_libraries(offline_core_tests PRIVATE offline_options)
add_executable(host_timing_probe src/timing_probe.cpp)
target_compile_options(host_timing_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME core COMMAND offline_core_tests)
add_test(NAME real_library_abi COMMAND igh_abi_check)
add_test(NAME fake_master_pdo COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_fake.py" "$<TARGET_FILE:igh_fake_smoke>")
add_test(NAME pmac_contract COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/import_pmac.py" --check)
add_test(NAME importer_regression COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_import.py")
set_tests_properties(core real_library_abi fake_master_pdo pmac_contract importer_regression PROPERTIES TIMEOUT 30)
