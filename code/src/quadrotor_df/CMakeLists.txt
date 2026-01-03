cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_df)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(df_quaternion_node
  src/df_quaternion_node.cpp
)
