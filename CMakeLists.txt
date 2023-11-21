cmake_minimum_required(VERSION 3.0.2)
project(hex_example)

set(HEX_ROS_VERSION $ENV{ROS_VERSION})
message(STATUS "Your ROS version is ROS${HEX_ROS_VERSION}")

add_definitions(-DHEX_ROS_VERSION=${HEX_ROS_VERSION}) 

if (${HEX_ROS_VERSION} STREQUAL "1")

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
    )

    catkin_package()

    include_directories(
        ../
        ./
        include
        ${catkin_INCLUDE_DIRS}
    )

    add_executable(hex_example
        hex_example/main.cpp
        hex_example/hex_example.cpp
        hex_example/data_interface/ros1_interface.cpp
    )

    target_link_libraries(hex_example
        ${catkin_LIBRARIES}
    )

    install(
        TARGETS hex_example
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros1
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros1
        DESTINATION share/${PROJECT_NAME}/config
    )

elseif (${HEX_ROS_VERSION} STREQUAL "2")

    cmake_minimum_required(VERSION 3.8)
    
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    include_directories(
        ../
        ./
        include
        ${colcon_INCLUDE_DIRS}
    )

    ament_package()

    add_executable(hex_example
        hex_example/main.cpp
        hex_example/hex_example.cpp
        hex_example/data_interface/ros2_interface.cpp
    )

    ament_target_dependencies(hex_example
        rclcpp
        std_msgs
    )

    install(
        TARGETS hex_example
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros2
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros2
        DESTINATION share/${PROJECT_NAME}/config
    )

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
