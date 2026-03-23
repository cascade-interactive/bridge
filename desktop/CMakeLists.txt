cmake_minimum_required(VERSION 3.20)

project(SimBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(BRIDGE_SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../shared")

file(GLOB_RECURSE BRIDGE_HEADERS "include/*.hpp")

add_executable(simbridge_desktop
    app/main.cpp
    app/bridge_core.cpp
    app/sockets/UDPSocket.cpp
    app/sockets/SerialPort.cpp
    ${BRIDGE_HEADERS}
)

target_include_directories(simbridge_desktop
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${BRIDGE_SHARED_DIR}"
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES
    app/main.cpp
    app/bridge_core.cpp
    app/sockets/UDPSocket.cpp
    app/sockets/SerialPort.cpp
    ${BRIDGE_HEADERS}
)
