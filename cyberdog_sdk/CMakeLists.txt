cmake_minimum_required(VERSION 3.5)
project(cyberdog_sdk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lcm REQUIRED)
find_package(sensor_msgs REQUIRED)


# generate cyber_dog_motor_sdk library
add_library(cyber_dog_motor_sdk SHARED src/CustomInterface.cpp)
target_include_directories(cyber_dog_motor_sdk PUBLIC
                    "include"
)
target_link_libraries(cyber_dog_motor_sdk
  pthread
  lcm
)

install(TARGETS 
  cyber_dog_motor_sdk
  DESTINATION lib)

# compile use code

add_executable(MotorCtrl src/MotorCtrl.cpp)
target_link_libraries(MotorCtrl cyber_dog_motor_sdk)
ament_target_dependencies(MotorCtrl rclcpp sensor_msgs)

install(TARGETS
  MotorCtrl
  DESTINATION lib/${PROJECT_NAME})


add_executable(IkCtrl src/IkCtrl.cpp)
target_link_libraries(IkCtrl cyber_dog_motor_sdk)
ament_target_dependencies(IkCtrl rclcpp sensor_msgs)

install(TARGETS
        IkCtrl
        DESTINATION lib/${PROJECT_NAME})


ament_package()

















