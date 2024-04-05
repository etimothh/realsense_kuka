cmake_minimum_required(VERSION 3.8)
project(cam_kuka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python files
install(PROGRAMS
  src/image_sender.py
  src/real_pointcloud.py
  src/tcpListener_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

