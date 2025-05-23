###################################################################
#
# CMake配置文件
#
# 参考资料：
#
# https://www.hahack.com/codes/cmake/
#
# Copyright(c)
#
# Shoujian Zhang, Wuhan University, 2020
###################################################################

# 编译该软件时使用的cmake版本
cmake_minimum_required(VERSION 3.10.2)

# 项目名称
project(gnssLab VERSION 1.2)

# 编译选项
# "Debug" 表明，构建系统时给出中间过程提示
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Eigen矩阵库的位置
include_directories( ${PROJECT_SOURCE_DIR}/thirdparty/eigen-3.4.0/ )

# 添加头文件的路径
include_directories( ${PROJECT_SOURCE_DIR}/lib/ )

# 添加源文件名到变量中
aux_source_directory( ${PROJECT_SOURCE_DIR}/lib/ DIR_LIB_SRCS )

# 根据源文件创建库文件
add_library( gnss SHARED ${DIR_LIB_SRCS}
        lib/NavEphBDS.h
        lib/NavEphBDS.cpp)

link_libraries(ws2_32)

# 创建可执行程序
add_executable(parse_opt examples/exam-1.1-parse_opt.cpp)
target_link_libraries(parse_opt gnss)

#add_executable(parse_config examples/exam-1.2-parse_config.cpp)
#target_link_libraries(parse_config gnss)
#
#add_executable(eigen examples/exam-2.1-eigen.cpp)
#target_link_libraries(eigen gnss)



add_executable(time_convert examples/exam-2.2-time_convert.cpp)
target_link_libraries(time_convert gnss)

add_executable(coord_convert1 examples/exam-2.3-coord_convert1.cpp )
target_link_libraries(coord_convert1 gnss)

add_executable(coord_convert2 examples/exam-2.3-coord_convert2.cpp)
target_link_libraries(coord_convert2 gnss)

add_executable(coord_convert3 examples/exam-2.3-coord_convert3.cpp)
target_link_libraries(coord_convert3 gnss)

add_executable(coord_convert4 examples/exam-2.3-coord_convert4.cpp)
target_link_libraries(coord_convert4 gnss)

add_executable(coord_convert5 examples/exam-2.3-coord_convert5.cpp)
target_link_libraries(coord_convert5 gnss)

add_executable(rinex_obs_struct examples/exam-3.3-rinex_obs_struct.cpp)
target_link_libraries(rinex_obs_struct gnss)

add_executable(rinex_nav_struct examples/exam-3.2-rinex_nav_struct.cpp)
target_link_libraries(rinex_nav_struct gnss)

add_executable(sppif examples/exam-6.1-sppif.cpp)
target_link_libraries(sppif gnss)

add_executable(cs_detect_mw examples/exam-7.1-cs_detect_mw.cpp)
target_link_libraries(cs_detect_mw gnss)

add_executable(rtk_lsq examples/exam-8.4-rtk_lsq.cpp)
target_link_libraries(rtk_lsq gnss)

add_executable(rtk_kal examples/exam-8.5-rtk_kal.cpp)
target_link_libraries(rtk_kal gnss)

add_executable(lambda examples/exam-8.3-lambda.cpp)
target_link_libraries(lambda gnss)

add_executable(tcpDecode examples/tcpDecode.cpp)
target_link_libraries(tcpDecode gnss)



#add_executable(rtk_kal examples/exam-8.4-rtk_kal.cpp)
#target_link_libraries(rtk_kal gnss)