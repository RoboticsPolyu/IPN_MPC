cmake_minimum_required(VERSION 3.1)

project(camera)

include_directories("/home/dengly/文档/项目文件/结构光视觉/工程文件/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/")
link_directories("/home/dengly/文档/项目文件/结构光视觉/工程文件/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.2104.9022/Galaxy_camera/lib/x86_64/")
link_libraries(
    GxGVTL.cti
    CxU3VTL.cti
    libgxiapi.so
)

add_executable(test test.cpp)