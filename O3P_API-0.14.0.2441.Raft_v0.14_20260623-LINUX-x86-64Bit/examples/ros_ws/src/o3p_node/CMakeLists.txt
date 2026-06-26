#------------------------------------------------------------------------------
# Copyright (C) 2026 pmdtechnologies gmbh
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.24)
project(o3p_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp QUIET)  # ROS2 
if(rclcpp_FOUND)
    set(USE_ROS2 TRUE)
    message(STATUS "Using ROS2")
    add_definitions(-DROS2)
    find_package(ament_cmake REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(diagnostic_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(geometry_msgs REQUIRED)
else()
    find_package(catkin QUIET COMPONENTS roscpp sensor_msgs std_msgs std_srvs diagnostic_msgs cv_bridge pluginlib tf) # ROS1
    if(catkin_FOUND)
        set(USE_ROS1 TRUE)
        message(STATUS "Using ROS1")
        add_definitions(-DROS1)
    else()
        message(WARNING "ROS not found. Cannot build the ROS node example.")
        return ()
    endif()
endif()

find_package(OpenCV REQUIRED)

option(O3P_CREATE_CUSTOM_RVIZ_PANEL "Create the custom rviz panel or not." ON)

if(USE_ROS1)
  catkin_package(
                INCLUDE_DIRS ""
                CATKIN_DEPENDS cv_bridge pluginlib rviz
  )
endif()

# Normalize whatever the user passes (or the default) to an absolute path and write it back
# to the cache so a relative value from a previous run can never persist across rebuilds.
set(O3P_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../" CACHE PATH "Path to O3P install directory")
get_filename_component(O3P_DIR "${O3P_DIR}" REALPATH BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(O3P_DIR "${O3P_DIR}" CACHE PATH "Path to O3P install directory" FORCE)
message(STATUS "O3P_DIR resolved to: ${O3P_DIR}")
set(O3P_INCLUDE_DIR "${O3P_DIR}/include")
set(O3P_LIB_DIR "${O3P_DIR}/lib")
set(O3P_LIB "${O3P_LIB_DIR}/libo3p.so")

set (SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/camera_node.cpp")

if(USE_ROS2)
  include_directories (
                    ${O3P_INCLUDE_DIR}
                    include
                    ${OpenCV_INCLUDE_DIRS}
  )
else()
  include_directories(
                      ${O3P_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS}
                      ${catkin_INCLUDE_DIRS}
  )
endif()

link_directories(${O3P_LIB_DIR})

add_executable(camera_node ${SOURCES})

if(USE_ROS2)
  target_link_libraries(camera_node
                        ${O3P_LIB}
                        ${OpenCV_LIBS})
else()
  target_link_libraries(camera_node 
                        ${O3P_LIB}
                        ${catkin_LIBRARIES}
                        ${OpenCV_LIBS})  
endif()

set_target_properties (camera_node PROPERTIES FOLDER ../examples)

set_target_properties(camera_node PROPERTIES
    BUILD_RPATH "${O3P_LIB_DIR}"
    INSTALL_RPATH "${O3P_LIB_DIR}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)

if(USE_ROS2)
  target_link_libraries(camera_node
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    cv_bridge::cv_bridge
    tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS})
  install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Catkin Package: ${CATKIN_PACKAGE_BIN_DESTINATION}")
  install(TARGETS camera_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

if(O3P_CREATE_CUSTOM_RVIZ_PANEL)
  if(USE_ROS2)
    # Jazzy RViz uses Qt5
    find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
    find_package(rviz_common REQUIRED)
    find_package(pluginlib REQUIRED)

    set(QT_WIDGETS_TARGET Qt5::Widgets)

    qt5_wrap_cpp(
      MOCED_HEADER
      "${CMAKE_CURRENT_SOURCE_DIR}/include/o3p_node/o3p_panel.hpp"
    )
  else()
    find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
    find_package(rviz REQUIRED)

    set(QT_WIDGETS_TARGET Qt5::Widgets)

    qt5_wrap_cpp(
      MOCED_HEADER
      "${CMAKE_CURRENT_SOURCE_DIR}/include/o3p_node/o3p_panel.hpp"
    )
  endif()

  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  add_library(
    o3p_panel
    SHARED
    src/o3p_panel.cpp
    ${MOCED_HEADER}
  )

  target_include_directories(o3p_panel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  if(USE_ROS2)
    target_link_libraries(o3p_panel
      ${QT_WIDGETS_TARGET}
      cv_bridge::cv_bridge
      pluginlib::pluginlib
      rviz_common::rviz_common
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
      ${std_srvs_TARGETS}
    )
  else()
    target_link_libraries(o3p_panel
      ${catkin_LIBRARIES}
      ${QT_WIDGETS_TARGET}
      ${OpenCV_LIBRARIES}
    )
  endif()

  install(
    TARGETS o3p_panel
    EXPORT export_o3p_panel
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(
    DIRECTORY include/
    DESTINATION include
  )

  if(USE_ROS1)
    install(
      FILES rviz_common_plugins_ros1.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
  endif()

  if(USE_ROS2)
    install(
      FILES rviz_common_plugins.xml
      DESTINATION share/${PROJECT_NAME}
    )

    ament_export_include_directories(include)
    ament_export_targets(export_o3p_panel)

    pluginlib_export_plugin_description_file(
      rviz_common
      rviz_common_plugins.xml
    )
  endif()
endif()
if(BUILD_TESTING)
  if(USE_ROS2)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

if(USE_ROS2)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
