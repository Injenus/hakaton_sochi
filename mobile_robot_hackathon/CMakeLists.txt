cmake_minimum_required(VERSION 3.22)

project(uart_connection_by_python)


set (SRC
    src/main.c
)

set (HEADERS  
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)

add_executable(${PROJECT_NAME} ${SRC})