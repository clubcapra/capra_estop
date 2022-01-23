project(capra_estop)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        )


catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)


add_library(jetsonGPIO_estop include/JetsonXavierGPIO/jetsonGPIO.c include/JetsonXavierGPIO/jetsonGPIO.h)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/estop.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

target_include_directories(${PROJECT_NAME} PRIVATE JetsonXavierGPIO)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} jetsonGPIO_estop)
