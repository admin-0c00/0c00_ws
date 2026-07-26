cmake_minimum_required(VERSION 3.8)
project(ground_station)

find_package(ament_cmake REQUIRED)

install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(PROGRAMS scripts/start_ground_station.sh scripts/stop_ground_station.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
