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
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/esi_config.hpp"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_esi.py"
    --check --header "${CMAKE_CURRENT_BINARY_DIR}/generated/esi_config.hpp"
  DEPENDS scripts/check_esi.py scripts/import_pmac.py config/pmac_drives.json config/esi_review.json
    "config/esi/ISMC_STANDARD_165_默认DC_20220720.xml"
  VERBATIM)
add_library(igh_config OBJECT src/igh_config.cpp "${CMAKE_CURRENT_BINARY_DIR}/generated/drive_config.hpp"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/esi_config.hpp")
target_link_libraries(igh_config PRIVATE offline_options)

add_executable(igh_abi_check src/abi_check.cpp)
target_link_libraries(igh_abi_check PRIVATE offline_options "${IGH_REAL}")
add_executable(igh_disabled_probe src/disabled_probe.cpp $<TARGET_OBJECTS:igh_config>)
target_link_libraries(igh_disabled_probe PRIVATE offline_options "${IGH_REAL}")
add_executable(igh_motion_probe src/motion_probe.cpp $<TARGET_OBJECTS:igh_config>)
target_link_libraries(igh_motion_probe PRIVATE offline_options "${IGH_REAL}")
add_executable(disabled_probe_tests tests/disabled_probe_tests.cpp)
target_link_libraries(disabled_probe_tests PRIVATE offline_options)
add_executable(commissioning_motion_tests tests/commissioning_motion_tests.cpp)
target_link_libraries(commissioning_motion_tests PRIVATE offline_options)
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
add_test(NAME disabled_frame COMMAND disabled_probe_tests)
add_test(NAME commissioning_motion_policy COMMAND commissioning_motion_tests)
add_test(NAME real_library_abi COMMAND igh_abi_check)
add_test(NAME fake_master_pdo COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_fake.py" "$<TARGET_FILE:igh_fake_smoke>")
add_test(NAME pmac_contract COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/import_pmac.py" --check)
add_test(NAME importer_regression COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_import.py")
add_test(NAME reset_scope COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_reset_fault_once.py")
add_test(NAME dictionary_wait COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_dictionary_wait.py")
add_test(NAME esi_contract COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_esi.py" --check)
add_test(NAME esi_regression COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esi.py")
add_test(NAME disabled_scope COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_disabled_scope.py")
add_test(NAME motion_scope COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_motion_scope.py")
set_tests_properties(core disabled_frame commissioning_motion_policy real_library_abi fake_master_pdo pmac_contract importer_regression reset_scope dictionary_wait esi_contract esi_regression disabled_scope motion_scope PROPERTIES TIMEOUT 30)
