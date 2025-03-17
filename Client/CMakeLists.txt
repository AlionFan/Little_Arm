cmake_minimum_required(VERSION 3.16)

project(can_client VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动处理 moc, uic, rcc
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 首先尝试查找Qt6
find_package(Qt6 COMPONENTS Core Gui Widgets Network)

if(Qt6_FOUND)
    message(STATUS "Building with Qt6")
    set(QT_VERSION_MAJOR 6)
else()
    message(STATUS "Qt6 not found, trying Qt5")
    find_package(Qt5 COMPONENTS Core Gui Widgets Network REQUIRED)
    set(QT_VERSION_MAJOR 5)
endif()

# 源文件
set(SOURCES
    src/main.cpp
    src/mainwindow.cpp
)

# 头文件
set(HEADERS
    src/mainwindow.h
)

# UI文件
set(UIS
    src/mainwindow.ui
)

# 创建可执行文件
add_executable(can_client
    ${SOURCES}
    ${HEADERS}
    ${UIS}
)

# 根据找到的Qt版本链接相应的库
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(can_client PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::Network
    )
else()
    target_link_libraries(can_client PRIVATE
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Network
    )
endif()

# 设置包含目录
target_include_directories(can_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 定义Qt版本宏
target_compile_definitions(can_client PRIVATE
    QT_VERSION_MAJOR=${QT_VERSION_MAJOR}
)

# 安装规则
install(TARGETS can_client
    RUNTIME DESTINATION bin
) 