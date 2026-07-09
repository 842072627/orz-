cmake_minimum_required(VERSION 3.16)
project(lifelong)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RELEASE")
endif()

include_directories("inc")

# default planner
include_directories("default_planner")

# OptScheduler / custom modules
include_directories("optsvr")
include_directories("optsvr/schedule/GameBalanceOpt")
include_directories("optsvr/executor/GameExecutor")
include_directories("optsvr/planner/GameDirectionalPlanner")

set(SUBMISSION_TRACK "AUTO" CACHE STRING "Submission track: AUTO, SCHEDULER, EXECUTOR, or COMBINED")
string(TOUPPER "${SUBMISSION_TRACK}" SUBMISSION_TRACK)

set(BUILD_OPTSVR_EXECUTOR OFF)
set(BUILD_OPTSVR_PLANNER OFF)

if(SUBMISSION_TRACK STREQUAL "AUTO")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/Executor.cpp" EXECUTOR_SOURCE_TEXT)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/MAPFPlanner.cpp" PLANNER_SOURCE_TEXT)

    if(EXECUTOR_SOURCE_TEXT MATCHES "OriginExecutor|DHExecutor|RoomOptExecutor|RoomExecutor|GameExecutor|IncrementalTpgExecutor|ConservativeIncrementalTpgExecutor")
        set(BUILD_OPTSVR_EXECUTOR ON)
    endif()
    if(PLANNER_SOURCE_TEXT MATCHES "OptPlanner|GameOptPlanner|TurnAwarePlanner|EPIBTPlanner|RandomFlowEPIBTPlanner|IdleYieldPlanner|RoomPlanner|RoomTidalPlanner|MazeTidalPlanner|TestUsefulFlowDecayPlanner|IronFlowDecayPlanner|HardWarehouseFlowDecayPlanner|LacamPIBTPlanner|GameRegionDecompPlanner|GameDirectionalPlanner")
        set(BUILD_OPTSVR_PLANNER ON)
    endif()
elseif(SUBMISSION_TRACK STREQUAL "SCHEDULER")
    set(BUILD_OPTSVR_EXECUTOR OFF)
    set(BUILD_OPTSVR_PLANNER OFF)
elseif(SUBMISSION_TRACK STREQUAL "EXECUTOR")
    set(BUILD_OPTSVR_EXECUTOR ON)
    set(BUILD_OPTSVR_PLANNER OFF)
elseif(SUBMISSION_TRACK STREQUAL "COMBINED")
    set(BUILD_OPTSVR_EXECUTOR ON)
    set(BUILD_OPTSVR_PLANNER ON)
else()
    message(FATAL_ERROR "Unknown SUBMISSION_TRACK=${SUBMISSION_TRACK}")
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp" "default_planner/*.cpp" "optsvr/*.cpp")
if(NOT BUILD_OPTSVR_EXECUTOR)
    list(FILTER SOURCES EXCLUDE REGEX ".*/optsvr/executor/.*\\.cpp$")
endif()
if(NOT BUILD_OPTSVR_PLANNER)
    list(FILTER SOURCES EXCLUDE REGEX ".*/optsvr/planner/.*\\.cpp$")
endif()
file(GLOB DEFAULT_PLANNER_SOURCES "default_planner/*.cpp")

message(STATUS "SUBMISSION_TRACK=${SUBMISSION_TRACK}")
message(STATUS "BUILD_OPTSVR_EXECUTOR=${BUILD_OPTSVR_EXECUTOR}")
message(STATUS "BUILD_OPTSVR_PLANNER=${BUILD_OPTSVR_PLANNER}")

# Find Boost
set(Boost_USE_STATIC_LIBS ON)
add_definitions(-DBOOST_ALL_NO_LIB)
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options filesystem log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Threads (for optsvr multi-threaded helpers)
find_package(Threads REQUIRED)

# Compatibility note:
# The official protected driver.cpp used by the benchmark server still keys off the
# legacy PYTHON macro, while the upgraded local driver uses MAPF_PYTHON_BINDINGS.
# For submission safety we always compile the protected-driver path with PYTHON=0,
# and only enable the upgraded local Python bridge via ENABLE_PYTHON_BINDINGS.
option(PYTHON "Legacy compatibility option kept for compile.sh / old scripts" OFF)
option(ENABLE_PYTHON_BINDINGS "Build embedded and extension Python bindings" OFF)

if(PYTHON AND NOT ENABLE_PYTHON_BINDINGS)
    set(ENABLE_PYTHON_BINDINGS ON)
endif()

set(HAS_PYBIND11 OFF)
if(ENABLE_PYTHON_BINDINGS)
    find_package(pybind11 QUIET)
    if(pybind11_FOUND)
        set(HAS_PYBIND11 ON)
    else()
        message(WARNING "pybind11 not found; Python bindings will be disabled for this build")
    endif()
endif()

if(HAS_PYBIND11)
    include_directories("python/common")

    pybind11_add_module(MAPF "./src/Grid.cpp" "./src/States.cpp" "./python/common/MAPFbinding.cpp")
    target_compile_definitions(MAPF PRIVATE MAPF_SHARED_MODULE)

    file(GLOB PY_BRIDGE_SOURCES "python/common/*.cpp")

    add_executable(lifelong ${PY_BRIDGE_SOURCES} ${SOURCES})
    target_compile_definitions(lifelong PRIVATE MAPF_PYTHON_BINDINGS PYTHON=0)
    target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES} Threads::Threads)
else()
    add_executable(lifelong ${SOURCES})
    target_compile_definitions(lifelong PRIVATE PYTHON=0)
    target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES} Threads::Threads)
endif()

if(COPY_PY_PATH_CONFIG)
    file(COPY "config.json" DESTINATION ${CMAKE_BINARY_DIR})
endif()

add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES
    "${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so"
)

option(BUILD_TESTS "Build planner tests" OFF)
if(BUILD_TESTS)
    enable_testing()
    add_executable(multi_step_plan_test
        tests/multi_step_plan_test.cpp
        ${DEFAULT_PLANNER_SOURCES}
    )
    target_link_libraries(multi_step_plan_test ${Boost_LIBRARIES})
    add_test(NAME multi_step_plan_test COMMAND multi_step_plan_test)

    add_executable(delay_generator_test
        tests/delay_generator_test.cpp
        src/DelayGenerator.cpp
        src/common.cpp
    )
    target_link_libraries(delay_generator_test ${Boost_LIBRARIES})
    add_test(NAME delay_generator_test COMMAND delay_generator_test)

    # Python interface test — needs all sources + bridge sources + pybind11::embed
    # Exclude driver.cpp (has its own main)
    set(TEST_SOURCES ${SOURCES})
    list(FILTER TEST_SOURCES EXCLUDE REGEX ".*driver\\.cpp")
    if(HAS_PYBIND11 AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/python_interface_test.cpp")
        add_executable(python_interface_test
            tests/python_interface_test.cpp
            ${PY_BRIDGE_SOURCES}
            ${TEST_SOURCES}
        )
        target_link_libraries(python_interface_test PRIVATE pybind11::embed ${Boost_LIBRARIES} Threads::Threads)
        add_test(NAME python_interface_test COMMAND python_interface_test
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    endif()
endif()
