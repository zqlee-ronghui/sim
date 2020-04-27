cmake_minimum_required(VERSION 3.14)
option(BUILD_UNIT_TEST "build unit test" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#find_package(LocalModules REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/util/util.cpp
    src/util/line.cpp)

set(TargetLinkLIBS
    gflags
    glog
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES})

target_link_libraries(${PROJECT_NAME}
    ${TargetLinkLIBS})

if(BUILD_UNIT_TEST)
  find_package(GMock REQUIRED)
  file(GLOB_RECURSE ALL_TESTS "*_test.cpp")
  file(GLOB_RECURSE ALL_LIBRARY_HDRS "src/*.h")
  file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cc" "src/*.cpp")
  list(LENGTH ALL_TESTS ALL_TESTS_CNT)
  if(${ALL_TESTS_CNT} GREATER 0)
    list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
  endif(${ALL_TESTS})

  add_library(${PROJECT_NAME}_test_library
      ${ALL_LIBRARY_SRCS}
      ${ALL_LIBRARY_HDRS}
      ${ProjectProtoSrcs})

  target_link_libraries(${PROJECT_NAME}_test_library
      ${TargetLinkLIBS})

  foreach(ABS_FIL ${ALL_TESTS})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
    # Replace slashes as required for CMP0037.
    string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
    add_executable(${TEST_TARGET_NAME} ${ABS_FIL})
    # Make sure that gmock always includes the correct gtest/gtest.h.
    target_include_directories(${TEST_TARGET_NAME} SYSTEM PRIVATE
        "${GMOCK_INCLUDE_DIRS}")
    target_link_libraries(${TEST_TARGET_NAME} PUBLIC
        ${GMOCK_LIBRARIES}
        ${PROJECT_NAME}_test_library)
    add_test(${TEST_TARGET_NAME} ${TEST_TARGET_NAME})
  endforeach()
endif(BUILD_UNIT_TEST)