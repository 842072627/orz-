cmake_minimum_required(VERSION 3.16)
project(lifelong LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

include_directories(
    inc
    default_planner
    optsvr
    optsvr/schedule/GameBalanceOpt
    optsvr/planner/GameDirectionalPlanner
    optsvr/planner/GameDirectionalPlanner/my_planner
    optsvr/executor/IronPlatoonExecutor
)

file(GLOB FRAMEWORK_SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB DEFAULT_PLANNER_SOURCES CONFIGURE_DEPENDS "default_planner/*.cpp")
file(GLOB GAME_PLANNER_SOURCES CONFIGURE_DEPENDS
    "optsvr/planner/GameDirectionalPlanner/*.cpp"
    "optsvr/planner/GameDirectionalPlanner/my_planner/*.cpp")

# The evaluation server restores the protected Python bridge and driver files.
# Include them only when they are present so this Game-only checkout stays small.
set(PY_BRIDGE_SOURCES)
set(HAS_OFFICIAL_PYTHON_BRIDGE OFF)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/python/common/MAPFbinding.cpp")
    find_package(pybind11 REQUIRED)
    include_directories("python/common")
    file(GLOB PY_BRIDGE_SOURCES CONFIGURE_DEPENDS "python/common/*.cpp")
    set(HAS_OFFICIAL_PYTHON_BRIDGE ON)
endif()

add_executable(lifelong
    ${FRAMEWORK_SOURCES}
    ${DEFAULT_PLANNER_SOURCES}
    ${GAME_PLANNER_SOURCES}
    ${PY_BRIDGE_SOURCES}
    optsvr/HeuristicMatrix.cpp
    optsvr/IdleParking.cpp
    optsvr/ScheduleStatsExport.cpp
    optsvr/schedule/GameBalanceOpt/GameBalanceOptScheduler.cpp
    optsvr/executor/IronPlatoonExecutor/IronPlatoonExecutor.cpp
)

add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(Boost 1.49.0 REQUIRED COMPONENTS program_options filesystem log log_setup)
find_package(Threads REQUIRED)
target_compile_definitions(lifelong PRIVATE PYTHON=0)
target_link_libraries(lifelong PRIVATE ${Boost_LIBRARIES} Threads::Threads)

if(HAS_OFFICIAL_PYTHON_BRIDGE)
    target_compile_definitions(lifelong PRIVATE MAPF_PYTHON_BINDINGS)
    target_link_libraries(lifelong PRIVATE pybind11::embed)
endif()
