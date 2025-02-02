cmake_minimum_required(VERSION 3.22.1)
project("ftcrobotcontroller")

add_library(integrator
        include/integrator.h include/integrator.cpp include/RK_45.h include/RK_45.cpp
)

add_library(${CMAKE_PROJECT_NAME} SHARED sim.cpp)

add_definitions(-D_USE_MATH_DEFINES)

if(ANDROID)
    target_link_libraries(${CMAKE_PROJECT_NAME} android log integrator)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${ANDROID_NDK}/sysroot/usr/include)
else()
    find_package(JNI REQUIRED)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${JNI_INCLUDE_DIRS})
    target_link_libraries(${CMAKE_PROJECT_NAME} ${JNI_LIBRARIES} integrator)
endif()