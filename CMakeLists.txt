cmake_minimum_required(VERSION 1.2.4)
project(EngienMgmt)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME} src/main.cpp)

cs_install()

cs_export()
