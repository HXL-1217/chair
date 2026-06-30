cmake_minimum_required(VERSION 3.10)

# 库项目名称
project(MotorDevice)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找线程库
find_package(Threads REQUIRED)

# 查找 libmodbus（通过 pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

# 创建静态库
add_library(${PROJECT_NAME} STATIC
    MotorBus.cpp
    MotorDeviceManager.cpp
    OID485MotorBus.cpp
    ZLAC8015MotorBus.cpp
    SCServoMotorBus.cpp
)

# 设置库的包含目录
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${LIBMODBUS_INCLUDE_DIRS}  # 👈 添加 libmodbus 头文件路径
)

# 链接依赖
target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
        Threads::Threads
        SCServo
    PUBLIC
        PkgConfig::LIBMODBUS
)

# # 可选：打印调试信息（确认路径是否正确）
# message(STATUS "Libmodbus include dirs: ${LIBMODBUS_INCLUDE_DIRS}")
# message(STATUS "Libmodbus libraries: ${LIBMODBUS_LIBRARIES}")