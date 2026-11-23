cmake_minimum_required(VERSION 3.8)
project(eezybot_description)

find_package(ament_cmake REQUIRED)

# Chỉ có file dữ liệu (URDF, cấu hình RViz, launch) -> cài vào share/
install(DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
