cmake_minimum_required(VERSION 3.0.2)
project(nmpc_motion_planner)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++14)

# Boost와 관련된 컴포넌트를 찾습니다. python38 컴포넌트를 추가합니다.
find_package(Boost REQUIRED COMPONENTS system python38)
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

# add_library(nmpc_motion_planner INTERFACE)
# target_include_directories(nmpc_motion_planner INTERFACE include)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES nmpc_motion_planner
 CATKIN_DEPENDS roscpp std_msgs gazebo_msgs geometry_msgs sensor_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  src/nmpc_prob.cpp
)


add_executable(nmpc_planner src/nmpc_planner.cpp)
target_link_libraries(nmpc_planner ${catkin_LIBRARIES} ${PROJECT_NAME} casadi)
add_dependencies(nmpc_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

