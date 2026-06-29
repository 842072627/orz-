cmake_minimum_required(VERSION 3.16)
project(lifelong)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RELEASE")
endif()

include_directories("inc")

# default planner
include_directories("default_planner")

# OptSvr / custom modules
include_directories("optsvr")
include_directories("optsvr/schedule/GreedyHeap")
include_directories("optsvr/schedule/IronZoneSchedule")
include_directories("optsvr/executor/DHExecutor")
include_directories("optsvr/planner/TestUsefulFlowDecay")
include_directories("optsvr/planner/IronFlowDecay")

include_directories("python/common")

file(GLOB_RECURSE SOURCES "src/*.cpp" "default_planner/*.cpp" "optsvr/*.cpp")
file(GLOB PY_BRIDGE_SOURCES "python/common/*.cpp")

# Find Boost
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options filesystem log log_setup)
include_directories(${Boost_INCLUDE_DIRS})

# Threads
find_package(Threads REQUIRED)

# pybind11 (required by driver.cpp)
find_package(pybind11 REQUIRED)

add_executable(lifelong ${PY_BRIDGE_SOURCES} ${SOURCES})
target_compile_definitions(lifelong PRIVATE MAPF_PYTHON_BINDINGS PYTHON=0)
target_link_libraries(lifelong PRIVATE pybind11::embed ${Boost_LIBRARIES} Threads::Threads)
