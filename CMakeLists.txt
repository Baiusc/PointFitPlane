# 最低cmake版本3.10
cmake_minimum_required( VERSION 3.10 )
# 设置项目名
project(PointFitPlane
        VERSION 0.0.1
        DESCRIPTION "PointFitPlane"
        HOMEPAGE_URL "https://github.com/Baiusc/PointFitPlane"
        LANGUAGES CXX )

### ----------------环境配置------------------
# 检查C++版本
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # 对于GNU和Clang编译器，使用__cplusplus预定义宏来检查C++版本
    if (NOT CMAKE_CXX_FLAGS MATCHES "-std=c\\+\\+11|-std=gnu\\+\\+11")
        message(FATAL_ERROR "需要使用C++11或更高版本，请设置编译选项：-std=c++11 或更高的标准.")
    endif()
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # 对于MSVC编译器，使用_MSVC_LANG预定义宏来检查C++版本
    if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.0)
        message(FATAL_ERROR "需要使用C++11或更高版本，请设置编译选项：/std:c++11 或更高的标准.")
    endif()
endif()
# 根据编译器类型添加编译选项，提高性能
if(MSVC)
    add_compile_options(/arch:AVX)
elseif(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-mavx)
endif()
# ALLBUILD和INSTALL项目放进文件夹
set_property(GLOBAL PROPERTY USE_FOLDERS YES)
# 设置 .NET FRAMEWORK 框架
# set(CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "4.8")
# 添加子目录
add_subdirectory( libs )
# 头文件和源文件列表
file(GLOB INCLUDE  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCE  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# 设置VS显示头文件
source_group("Header Files" FILES ${INCLUDE})
# 将头文件目录添加到编译器的搜索路径中
# include_directories( include DIRS)
# 将源文件目录添加到编译器的搜索路径中
# aux_source_directory( src DIRS)

### ----------------寻找PCL库------------------
find_package( PCL REQUIRED )
if(PCL_FOUND)
    message("!!!!!!!!PCL found")
    message("!!!!!!!!PCL_ROOT:" ${PCL_ROOT})
	message("!!!!!!!!PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
	message("!!!!!!!!PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
    message("!!!!!!!!boost_Lib:" ${boost_Lib})
    message("VTK_USE_FILE: " ${VTK_USE_FILE})
    # 将PCL头文件目录添加到编译器的搜索路径中
    include_directories( ${PCL_INCLUDE_DIRS} )
    # 将PCL库目录添加到链接器的搜索路径
    link_directories(${PCL_LIBRARY_DIRS})
    # 将PCL宏定义添加到链接器的搜索路径
    add_definitions(${PCL_DEFINITIONS})
else()
    message(FATAL_ERROR "!!!!!!!!PCL not found. Please specify the PCL installation path manually.")
endif()
### ----------------寻找VTK库------------------
find_package(VTK)
if(VTK_FOUND)
    message("VTK found at: ${VTK_DIR}")
    # 包含VTK的使用文件
    include(${VTK_USE_FILE})
else()
    message(FATAL_ERROR "!!!!!!!!VTK not found. Please specify the VTK installation path manually.")
endif()

### ----------------点拟合面，纯C++项目------------------

# 生成对象库文件 .lib
add_library(PointFitPlane STATIC)
# 设置自身源和包含目录
target_sources(PointFitPlane PRIVATE ${SOURCE} ${INCLUDE})
target_include_directories(PointFitPlane PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# 链接第三方库
target_link_libraries(
    PointFitPlane 
    PUBLIC
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
)
# 使用调试库
target_compile_options(PointFitPlane PUBLIC $<$<CONFIG:DEBUG>:/MDd>)











