cmake_minimum_required(VERSION 1.2.4)
project(engien_mgmt)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME} src/engien_mgmt_node.cpp)

cs_install()

cs_export()
