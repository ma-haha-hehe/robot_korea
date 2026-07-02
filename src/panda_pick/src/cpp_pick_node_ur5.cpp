#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ur_msgs/srv/set_io.hpp>  // 2026-07-02: OnRobot 夹爪走 set_io 服务(pin16)开合
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// ================= 配置 =================
const std::string PLANNING_GROUP = "ur_manipulator"; 
using GripperCommand = control_msgs::action::GripperCommand;
using SetIO = ur_msgs::srv::SetIO;

// 夹爪控制模拟：Robotiq 2F 弧度范围通常是 0.0 (开) 到 0.8 (闭)
const double GRIPPER_OPEN = 0.0;
const double GRIPPER_CLOSE = 0.7; 

struct TaskOffset {
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    double yaw = 0.0;
    double descend = 0.0;
    double slow_final_descend = 0.0;
};

struct PickPlaceTask {
    std::string id = "task";
    TaskOffset pick;
    TaskOffset place;
};

bool publishUrscriptProgram(rclcpp::Node::SharedPtr node,
                            const std::string& topic_name,
                            const std::string& program,
                            double wait_seconds);

struct OnRobotRgConfig;

bool driveGripperOnRobotRg(rclcpp::Node::SharedPtr node,
                           const std::string& topic_name,
                           const OnRobotRgConfig& config,
                           int width_mm,
                           int force_n,
                           double wait_seconds);

template <typename FutureT>
bool waitForFuture(rclcpp::Node::SharedPtr node, FutureT& future, std::chrono::seconds timeout, const std::string& what) {
    if (future.wait_for(timeout) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "%s 超时。", what.c_str());
        return false;
    }
    return true;
}

bool driveGripperAction(rclcpp::Node::SharedPtr node,
                        const std::string& action_name,
                        double position,
                        double max_effort) {
    auto client = rclcpp_action::create_client<GripperCommand>(node, action_name);

    RCLCPP_INFO(node->get_logger(), "等待夹爪 action server: %s", action_name.c_str());
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_WARN(
            node->get_logger(),
            "没有找到夹爪 action server: %s。跳过本次夹爪动作。",
            action_name.c_str());
        return false;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = max_effort;

    RCLCPP_INFO(
        node->get_logger(),
        "发送夹爪目标: position=%.3f, max_effort=%.1f",
        position,
        max_effort);

    auto goal_handle_future = client->async_send_goal(goal);
    if (!waitForFuture(node, goal_handle_future, std::chrono::seconds(5), "发送夹爪目标")) {
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "夹爪目标被 action server 拒绝。");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (!waitForFuture(node, result_future, std::chrono::seconds(10), "等待夹爪执行完成")) {
        return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node->get_logger(), "夹爪动作失败，result code=%d。", static_cast<int>(result.code));
        return false;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "夹爪动作完成: position=%.3f, effort=%.3f, stalled=%s, reached_goal=%s",
        result.result->position,
        result.result->effort,
        result.result->stalled ? "true" : "false",
        result.result->reached_goal ? "true" : "false");
    return true; 
}

bool setDigitalOutput(rclcpp::Node::SharedPtr node,
                      const std::string& service_name,
                      int pin,
                      bool on) {
    auto client = node->create_client<SetIO>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(node->get_logger(), "没有找到 UR I/O service: %s", service_name.c_str());
        return false;
    }

    auto request = std::make_shared<SetIO::Request>();
    request->fun = SetIO::Request::FUN_SET_DIGITAL_OUT;
    request->pin = static_cast<int8_t>(pin);
    request->state = on ? SetIO::Request::STATE_ON : SetIO::Request::STATE_OFF;

    RCLCPP_INFO(
        node->get_logger(),
        "设置 UR digital output: pin=%d, state=%s",
        pin,
        on ? "ON" : "OFF");

    auto response_future = client->async_send_request(request);
    if (!waitForFuture(node, response_future, std::chrono::seconds(3), "等待 UR I/O service 返回")) {
        return false;
    }

    const auto response = response_future.get();
    if (!response->success) {
        RCLCPP_ERROR(node->get_logger(), "UR I/O service 返回失败: pin=%d", pin);
        return false;
    }
    return true;
}

bool driveGripperIo(rclcpp::Node::SharedPtr node,
                    const std::string& service_name,
                    int open_pin,
                    int close_pin,
                    bool open,
                    double pulse_seconds,
                    bool latching) {
    const int active_pin = open ? open_pin : close_pin;
    const int inactive_pin = open ? close_pin : open_pin;
    RCLCPP_INFO(node->get_logger(), "%s夹爪: 使用 UR I/O pin %d", open ? "打开" : "闭合", active_pin);

    bool ok = true;
    ok = setDigitalOutput(node, service_name, inactive_pin, false) && ok;
    ok = setDigitalOutput(node, service_name, active_pin, true) && ok;

    if (pulse_seconds > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(pulse_seconds));
    }

    if (!latching) {
        ok = setDigitalOutput(node, service_name, active_pin, false) && ok;
    }
    return ok;
}

bool driveGripperUrscript(rclcpp::Node::SharedPtr node,
                          const std::string& topic_name,
                          bool open,
                          double wait_seconds) {
    auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::QoS(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (publisher->get_subscription_count() == 0) {
        RCLCPP_WARN(node->get_logger(), "URScript topic 没有订阅者: %s", topic_name.c_str());
    }

    std_msgs::msg::String msg;
    msg.data =
        std::string("def robotiq_gripper_cmd():\n") +
        (open ? "  rq_open_and_wait()\n" : "  rq_close_and_wait()\n") +
        "end\n";

    RCLCPP_INFO(node->get_logger(), "发送 Robotiq URScript: %s", open ? "open" : "close");
    publisher->publish(msg);
    if (wait_seconds > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds));
    }
    return true;
}

bool activateGripperUrscript(rclcpp::Node::SharedPtr node,
                             const std::string& topic_name,
                             double wait_seconds) {
    auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::QoS(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std_msgs::msg::String msg;
    msg.data =
        "def robotiq_gripper_activate():\n"
        "  rq_reset()\n"
        "  sleep(0.5)\n"
        "  rq_activate_and_wait()\n"
        "end\n";

    RCLCPP_INFO(node->get_logger(), "发送 Robotiq 激活脚本。");
    publisher->publish(msg);
    if (wait_seconds > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds));
    }
    return true;
}

bool driveGripper(rclcpp::Node::SharedPtr node,
                  const std::string& mode,
                  const std::string& action_name,
                  const std::string& io_service_name,
                  const std::string& urscript_topic,
                  const OnRobotRgConfig& onrobot_rg_config,
                  int open_pin,
                  int close_pin,
                  double open_position,
                  double close_position,
                  double position,
                  int onrobot_rg_open_width_mm,
                  int onrobot_rg_close_width_mm,
                  int onrobot_rg_force_n,
                  double max_effort,
                  double io_pulse_seconds,
                  bool io_latching,
                  double urscript_wait_seconds) {
    if (mode == "none") {
        RCLCPP_INFO(node->get_logger(), "夹爪控制模式为 none，跳过夹爪动作。");
        return true;
    }

    if (mode == "action") {
        return driveGripperAction(node, action_name, position, max_effort);
    }

    if (mode == "io") {
        const double midpoint = (open_position + close_position) * 0.5;
        const bool open = position <= midpoint;
        return driveGripperIo(
            node,
            io_service_name,
            open_pin,
            close_pin,
            open,
            io_pulse_seconds,
            io_latching);
    }

    if (mode == "urscript") {
        const double midpoint = (open_position + close_position) * 0.5;
        const bool open = position <= midpoint;
        return driveGripperUrscript(node, urscript_topic, open, urscript_wait_seconds);
    }

    if (mode == "onrobot_rg") {
        const double midpoint = (open_position + close_position) * 0.5;
        const bool open = position <= midpoint;
        return driveGripperOnRobotRg(
            node,
            urscript_topic,
            onrobot_rg_config,
            open ? onrobot_rg_open_width_mm : onrobot_rg_close_width_mm,
            onrobot_rg_force_n,
            urscript_wait_seconds);
    }

    if (mode == "robotiq_socket") {
        RCLCPP_ERROR(
            node->get_logger(),
            "robotiq_socket 只支持 joint_pick_place/urscript pipeline，不支持 MoveIt 分步模式。");
        return false;
    }

    RCLCPP_ERROR(
        node->get_logger(),
        "未知夹爪控制模式: %s。请使用 none、action、io、urscript、robotiq_socket 或 onrobot_rg。",
        mode.c_str());
    return false;
}

// ================= 添加大夹爪碰撞保护体 =================
void attachGripperCollision(moveit::planning_interface::MoveGroupInterface& arm, 
                             moveit::planning_interface::PlanningSceneInterface& psi) {
    moveit_msgs::msg::CollisionObject gripper_box;
    gripper_box.header.frame_id = arm.getEndEffectorLink(); 
    gripper_box.id = "robotiq_2f_collision_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    // 保护尺寸：长12cm, 宽12cm, 高23cm (稍微收窄以防自碰撞)
    primitive.dimensions = {0.12, 0.12, 0.23};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.z = 0.115; // 盒子中心偏移
    box_pose.orientation.w = 1.0;

    gripper_box.primitives.push_back(primitive);
    gripper_box.primitive_poses.push_back(box_pose);
    gripper_box.operation = gripper_box.ADD;

    arm.attachObject(gripper_box.id, arm.getEndEffectorLink());
    psi.applyCollisionObject(gripper_box);
}

bool moveToPose(rclcpp::Node::SharedPtr node,
                moveit::planning_interface::MoveGroupInterface& arm,
                const geometry_msgs::msg::Pose& target_pose,
                const std::string& label) {
    RCLCPP_INFO(
        node->get_logger(),
        "%s: target position=(%.4f, %.4f, %.4f), orientation=(%.4f, %.4f, %.4f, %.4f)",
        label.c_str(),
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);

    arm.clearPoseTargets();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "%s 规划失败。", label.c_str());
        return false;
    }

    if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "%s 执行失败。", label.c_str());
        return false;
    }
    return true;
}

bool moveLinearZ(rclcpp::Node::SharedPtr node,
                 moveit::planning_interface::MoveGroupInterface& arm,
                 double dz,
                 const std::string& label) {
    geometry_msgs::msg::Pose target_pose = arm.getCurrentPose().pose;
    target_pose.position.z += dz;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    RCLCPP_INFO(node->get_logger(), "%s: Cartesian path fraction=%.3f", label.c_str(), fraction);
    if (fraction < 0.95) {
        RCLCPP_ERROR(node->get_logger(), "%s 直线路径不足，取消执行。", label.c_str());
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (arm.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "%s 执行失败。", label.c_str());
        return false;
    }
    return true;
}

bool moveOffsetPose(rclcpp::Node::SharedPtr node,
                    moveit::planning_interface::MoveGroupInterface& arm,
                    double dx,
                    double dy,
                    double dz,
                    const std::string& label) {
    geometry_msgs::msg::Pose target_pose = arm.getCurrentPose().pose;
    target_pose.position.x += dx;
    target_pose.position.y += dy;
    target_pose.position.z += dz;
    return moveToPose(node, arm, target_pose, label);
}

std::string formatUrscriptNumber(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

void quaternionToRotationVector(const geometry_msgs::msg::Quaternion& orientation,
                                double& rx,
                                double& ry,
                                double& rz) {
    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);

    if (q.length2() < 1e-12) {
        rx = 0.0;
        ry = 0.0;
        rz = 0.0;
        return;
    }

    q.normalize();
    double angle = q.getAngle();
    tf2::Vector3 axis = q.getAxis();

    constexpr double kPi = 3.14159265358979323846;
    if (angle > kPi) {
        angle -= 2.0 * kPi;
    }

    if (std::abs(angle) < 1e-9 || axis.length2() < 1e-12) {
        rx = 0.0;
        ry = 0.0;
        rz = 0.0;
        return;
    }

    rx = axis.x() * angle;
    ry = axis.y() * angle;
    rz = axis.z() * angle;
}

std::string poseToUrscriptPose(const geometry_msgs::msg::Pose& pose) {
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    quaternionToRotationVector(pose.orientation, rx, ry, rz);

    return std::string("p[") +
        formatUrscriptNumber(pose.position.x) + ", " +
        formatUrscriptNumber(pose.position.y) + ", " +
        formatUrscriptNumber(pose.position.z) + ", " +
        formatUrscriptNumber(rx) + ", " +
        formatUrscriptNumber(ry) + ", " +
        formatUrscriptNumber(rz) + "]";
}

std::optional<std::vector<double>> parseJointList(const std::string& text) {
    if (text.empty()) {
        return std::nullopt;
    }

    std::string cleaned;
    cleaned.reserve(text.size());
    for (const char c : text) {
        if (c == '[' || c == ']') {
            cleaned.push_back(' ');
        } else {
            cleaned.push_back(c);
        }
    }

    std::vector<double> joints;
    std::stringstream stream(cleaned);
    std::string item;
    while (std::getline(stream, item, ',')) {
        std::stringstream value_stream(item);
        double value = 0.0;
        if (!(value_stream >> value)) {
            return std::nullopt;
        }
        joints.push_back(value);
    }

    if (joints.size() != 6) {
        return std::nullopt;
    }
    return joints;
}

std::string jointListToUrscript(const std::vector<double>& joints) {
    std::string output = "[";
    for (size_t i = 0; i < joints.size(); ++i) {
        if (i > 0) {
            output += ", ";
        }
        output += formatUrscriptNumber(joints[i]);
    }
    output += "]";
    return output;
}

double readYamlDouble(const YAML::Node& node, const std::string& key, double default_value) {
    if (!node || !node[key]) {
        return default_value;
    }
    return node[key].as<double>();
}

double readYamlYawRadians(const YAML::Node& node, double default_value) {
    if (!node) {
        return default_value;
    }
    if (node["yaw"]) {
        return node["yaw"].as<double>();
    }
    if (node["yaw_deg"]) {
        constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
        return node["yaw_deg"].as<double>() * kDegreesToRadians;
    }
    return default_value;
}

std::string sanitizeUrscriptText(std::string text) {
    for (char& c : text) {
        if (c == '"' || c == '\\' || c == '\n' || c == '\r') {
            c = '_';
        }
    }
    return text;
}

struct OnRobotRgConfig {
    std::string model = "rg2";
    int max_width_mm = 110;
    int max_force_n = 40;
    int encode_force_divide = 2;
    int encode_force_factor = 111;
};

std::string normalizeOnRobotModel(std::string model) {
    std::transform(model.begin(), model.end(), model.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (model == "rg6") {
        return "rg6";
    }
    return "rg2";
}

OnRobotRgConfig makeOnRobotRgConfig(const std::string& model_text) {
    OnRobotRgConfig config;
    config.model = normalizeOnRobotModel(model_text);
    if (config.model == "rg6") {
        config.max_width_mm = 160;
        config.max_force_n = 120;
        config.encode_force_divide = 5;
        config.encode_force_factor = 161;
    }
    return config;
}

int clampOnRobotWidth(int width_mm, const OnRobotRgConfig& config) {
    return std::clamp(width_mm, 0, config.max_width_mm);
}

int clampOnRobotForce(int force_n, const OnRobotRgConfig& config) {
    return std::clamp(force_n, 0, config.max_force_n);
}

int onRobotRgCommandValue(int width_mm,
                          int force_n,
                          const OnRobotRgConfig& config,
                          bool slave = false,
                          bool depth_compensate = false) {
    width_mm = clampOnRobotWidth(width_mm, config);
    force_n = clampOnRobotForce(force_n, config);

    int value = width_mm * 4;
    value += (force_n / config.encode_force_divide) * 4 * config.encode_force_factor;
    if (slave) {
        value += 16384;
    }
    if (depth_compensate) {
        value += 32768;
    }
    return value;
}

void appendOnRobotRgHelpers(std::ostringstream& script) {
    script << "  def onrobot_rg_wait(seconds):\n";
    script << "    local sync_cnt = 0\n";
    script << "    local sync_target = floor(seconds / 0.008)\n";
    script << "    while sync_cnt < sync_target:\n";
    script << "      sync()\n";
    script << "      sync_cnt = sync_cnt + 1\n";
    script << "    end\n";
    script << "  end\n";

    script << "  def onrobot_rg_bit(input):\n";
    script << "    local i = 0\n";
    script << "    local output = 0\n";
    script << "    while i < 17:\n";
    script << "      set_digital_out(8, True)\n";
    script << "      if input >= 65536:\n";
    script << "        input = input - 65536\n";
    script << "        set_digital_out(9, False)\n";
    script << "      else:\n";
    script << "        set_digital_out(9, True)\n";
    script << "      end\n";
    script << "      if get_digital_in(8):\n";
    script << "        output = 1\n";
    script << "      end\n";
    script << "      sync()\n";
    script << "      set_digital_out(8, False)\n";
    script << "      sync()\n";
    script << "      input = input * 2\n";
    script << "      i = i + 1\n";
    script << "    end\n";
    script << "    return output\n";
    script << "  end\n";

    script << "  def onrobot_rg_powerup():\n";
    script << "    textmsg(\"OnRobot RG powerup\")\n";
    script << "    set_tool_voltage(0)\n";
    script << "    onrobot_rg_wait(2.0)\n";
    script << "    set_digital_out(8, False)\n";
    script << "    set_digital_out(9, False)\n";
    script << "    local pulse = 0\n";
    script << "    while pulse < 4:\n";
    script << "      set_tool_voltage(24)\n";
    script << "      onrobot_rg_wait(0.032)\n";
    script << "      set_tool_voltage(0)\n";
    script << "      onrobot_rg_wait(0.064)\n";
    script << "      pulse = pulse + 1\n";
    script << "    end\n";
    script << "    set_tool_voltage(24)\n";
    script << "    local timeout = 0\n";
    script << "    while get_digital_in(8) == False:\n";
    script << "      timeout = timeout + 1\n";
    script << "      sync()\n";
    script << "      if timeout > 250:\n";
    script << "        textmsg(\"OnRobot RG not responding on DI8\")\n";
    script << "        return False\n";
    script << "      end\n";
    script << "    end\n";
    script << "    timeout = 0\n";
    script << "    while get_digital_in(9):\n";
    script << "      timeout = timeout + 1\n";
    script << "      sync()\n";
    script << "      if timeout > 250:\n";
    script << "        textmsg(\"OnRobot RG not ready on DI9\")\n";
    script << "        return False\n";
    script << "      end\n";
    script << "    end\n";
    script << "    return True\n";
    script << "  end\n";

    script << "  def onrobot_rg_wait_motion(max_seconds):\n";
    script << "    local timeout = 0\n";
    script << "    while get_digital_in(9) == True:\n";
    script << "      timeout = timeout + 1\n";
    script << "      sync()\n";
    script << "      if timeout > 20:\n";
    script << "        break\n";
    script << "      end\n";
    script << "    end\n";
    script << "    timeout = 0\n";
    script << "    local timeout_limit = floor(max_seconds / 0.008)\n";
    script << "    while get_digital_in(9) == False:\n";
    script << "      timeout = timeout + 1\n";
    script << "      sync()\n";
    script << "      if timeout > timeout_limit:\n";
    script << "        break\n";
    script << "      end\n";
    script << "    end\n";
    script << "  end\n";
}

void appendOnRobotRgMove(std::ostringstream& script,
                         const OnRobotRgConfig& config,
                         int width_mm,
                         int force_n,
                         double wait_seconds,
                         const std::string& label) {
    width_mm = clampOnRobotWidth(width_mm, config);
    force_n = clampOnRobotForce(force_n, config);
    const int command = onRobotRgCommandValue(width_mm, force_n, config);
    script << "  if (onrobot_rg_ready):\n";
    script << "    textmsg(\"OnRobot RG " << label << " width="
           << width_mm << " force=" << force_n << "\")\n";
    script << "    onrobot_rg_bit(" << command << ")\n";
    script << "    onrobot_rg_wait_motion("
           << formatUrscriptNumber(std::max(0.0, wait_seconds)) << ")\n";
    script << "  end\n";
}

// 2026-07-02: OnRobot 夹爪的"单数字输出"控制。用户实测:
//   ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO "fun:1 pin:16 state:X"
//   pin 16 = UR tool digital out 0; state 0.0=开(张开), 1.0=关(闭合抓取)。
// 在 URScript 内用 set_tool_digital_out(0, bool) 等价触发, 时序=抓/放到位点, 后接 sleep 给夹爪动作时间。
void appendIoGripperMove(std::ostringstream& script,
                         int tool_dout_index,
                         bool close_state,
                         double wait_seconds,
                         const std::string& label) {
    script << "  textmsg(\"IO gripper " << label << ": tool_out " << tool_dout_index
           << "=" << (close_state ? "1(close)" : "0(open)") << "\")\n";
    script << "  set_tool_digital_out(" << tool_dout_index << ", "
           << (close_state ? "True" : "False") << ")\n";
    script << "  sleep(" << formatUrscriptNumber(std::max(0.0, wait_seconds)) << ")\n";
}

bool driveGripperOnRobotRg(rclcpp::Node::SharedPtr node,
                           const std::string& topic_name,
                           const OnRobotRgConfig& config,
                           int width_mm,
                           int force_n,
                           double wait_seconds) {
    std::ostringstream script;
    script << "def onrobot_rg_cmd():\n";
    appendOnRobotRgHelpers(script);
    script << "  onrobot_rg_ready = onrobot_rg_powerup()\n";
    appendOnRobotRgMove(script, config, width_mm, force_n, wait_seconds, "move");
    script << "end\n";

    return publishUrscriptProgram(node, topic_name, script.str(), std::max(4.0, wait_seconds + 3.0));
}

void appendRobotiqSocketSetVar(std::ostringstream& script,
                               const std::string& indent,
                               const std::string& name,
                               int value) {
    script << indent << "socket_set_var(\"" << name << "\", " << value
           << ", \"gripper_socket\")\n";
    script << indent << "sync()\n";
    script << indent << "rq_ack = socket_read_byte_list(3, \"gripper_socket\")\n";
    script << indent << "sync()\n";
}

void appendRobotiqSocketHelpers(std::ostringstream& script) {
    script << "def rq_socket_set(var_name, var_value):\n";
    script << "  socket_set_var(var_name, var_value, \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "  rq_ack = socket_read_byte_list(3, \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "end\n";

    script << "def rq_socket_get_sta():\n";
    script << "  socket_send_string(\"GET STA\", \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "  rq_value = socket_read_byte_list(1, \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "  return rq_value\n";
    script << "end\n";

    script << "def rq_socket_get_obj():\n";
    script << "  socket_send_string(\"GET OBJ\", \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "  rq_value = socket_read_byte_list(1, \"gripper_socket\")\n";
    script << "  sync()\n";
    script << "  return rq_value\n";
    script << "end\n";

    script << "def rq_socket_is_activated():\n";
    script << "  rq_sta = rq_socket_get_sta()\n";
    script << "  if (rq_sta[0] != 1):\n";
    script << "    return False\n";
    script << "  end\n";
    script << "  if (rq_sta[1] == 51):\n";
    script << "    return True\n";
    script << "  end\n";
    script << "  return False\n";
    script << "end\n";

    script << "def rq_socket_motion_done():\n";
    script << "  rq_obj = rq_socket_get_obj()\n";
    script << "  if (rq_obj[0] != 1):\n";
    script << "    return False\n";
    script << "  end\n";
    script << "  if (rq_obj[1] == 49):\n";
    script << "    return True\n";
    script << "  end\n";
    script << "  if (rq_obj[1] == 50):\n";
    script << "    return True\n";
    script << "  end\n";
    script << "  if (rq_obj[1] == 51):\n";
    script << "    return True\n";
    script << "  end\n";
    script << "  return False\n";
    script << "end\n";
}

void appendRobotiqSocketSetup(std::ostringstream& script,
                              int speed,
                              int force) {
    speed = std::clamp(speed, 0, 255);
    force = std::clamp(force, 0, 255);

    script << "  textmsg(\"Robotiq DCU socket init\")\n";
    script << "  socket_close(\"gripper_socket\")\n";
    script << "  sleep(0.1)\n";
    script << "  robotiq_socket_ok = socket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n";
    script << "  if (robotiq_socket_ok):\n";
    script << "    textmsg(\"Robotiq DCU socket connected\")\n";
    appendRobotiqSocketSetVar(script, "    ", "ACT", 1);
    script << "    sleep(1.0)\n";
    appendRobotiqSocketSetVar(script, "    ", "SPE", speed);
    appendRobotiqSocketSetVar(script, "    ", "FOR", force);
    appendRobotiqSocketSetVar(script, "    ", "GTO", 1);
    script << "  else:\n";
    script << "    textmsg(\"Robotiq DCU socket not connected; gripper skipped\")\n";
    script << "  end\n";
}

void appendRobotiqSocketMove(std::ostringstream& script,
                             int position,
                             double wait_seconds) {
    position = std::clamp(position, 0, 255);

    script << "  if (robotiq_socket_ok):\n";
    appendRobotiqSocketSetVar(script, "    ", "GTO", 1);
    appendRobotiqSocketSetVar(script, "    ", "POS", position);
    script << "    sleep(" << formatUrscriptNumber(std::max(0.0, wait_seconds)) << ")\n";
    script << "  end\n";
}

void appendRobotiqSocketClose(std::ostringstream& script) {
    script << "  if (robotiq_socket_ok):\n";
    script << "    socket_close(\"gripper_socket\")\n";
    script << "  end\n";
}

bool executeRobotiqSocketTest(rclcpp::Node::SharedPtr node,
                              const std::string& topic_name,
                              double gripper_socket_wait_seconds,
                              int gripper_socket_speed,
                              int gripper_socket_force,
                              double wait_seconds) {
    std::ostringstream script;
    script << "def robotiq_socket_test():\n";
    script << "  textmsg(\"Robotiq socket test start\")\n";
    appendRobotiqSocketSetup(script, gripper_socket_speed, gripper_socket_force);
    script << "  textmsg(\"Robotiq open\")\n";
    appendRobotiqSocketMove(script, 0, gripper_socket_wait_seconds);
    script << "  sleep(0.5)\n";
    script << "  textmsg(\"Robotiq close\")\n";
    appendRobotiqSocketMove(script, 255, gripper_socket_wait_seconds);
    script << "  sleep(0.5)\n";
    script << "  textmsg(\"Robotiq open again\")\n";
    appendRobotiqSocketMove(script, 0, gripper_socket_wait_seconds);
    appendRobotiqSocketClose(script);
    script << "  textmsg(\"Robotiq socket test done\")\n";
    script << "end\n";

    RCLCPP_INFO(
        node->get_logger(),
        "发送 Robotiq socket 测试：open -> close -> open，speed=%d, force=%d",
        std::clamp(gripper_socket_speed, 0, 255),
        std::clamp(gripper_socket_force, 0, 255));
    return publishUrscriptProgram(node, topic_name, script.str(), wait_seconds);
}

bool executeOnRobotRgTest(rclcpp::Node::SharedPtr node,
                          const std::string& topic_name,
                          const OnRobotRgConfig& config,
                          int open_width_mm,
                          int close_width_mm,
                          int force_n,
                          double move_wait_seconds,
                          double wait_seconds) {
    std::ostringstream script;
    script << "def onrobot_rg_test():\n";
    appendOnRobotRgHelpers(script);
    script << "  textmsg(\"OnRobot RG test start\")\n";
    script << "  onrobot_rg_ready = onrobot_rg_powerup()\n";
    appendOnRobotRgMove(script, config, open_width_mm, force_n, move_wait_seconds, "open");
    script << "  sleep(0.5)\n";
    appendOnRobotRgMove(script, config, close_width_mm, force_n, move_wait_seconds, "close");
    script << "  sleep(0.5)\n";
    appendOnRobotRgMove(script, config, open_width_mm, force_n, move_wait_seconds, "open again");
    script << "  textmsg(\"OnRobot RG test done\")\n";
    script << "end\n";

    RCLCPP_INFO(
        node->get_logger(),
        "发送 OnRobot RG 测试：model=%s, open=%dmm, close=%dmm, force=%dN",
        config.model.c_str(),
        clampOnRobotWidth(open_width_mm, config),
        clampOnRobotWidth(close_width_mm, config),
        clampOnRobotForce(force_n, config));
    return publishUrscriptProgram(node, topic_name, script.str(), wait_seconds);
}

bool validateTaskOffset(rclcpp::Node::SharedPtr node,
                        const PickPlaceTask& task,
                        const TaskOffset& offset,
                        const std::string& label,
                        double max_xy_offset,
                        double max_z_offset,
                        double max_descend) {
    if (std::abs(offset.dx) > max_xy_offset || std::abs(offset.dy) > max_xy_offset) {
        RCLCPP_ERROR(
            node->get_logger(),
            "任务 %s 的 %s dx/dy 超过安全限制 %.3f m。",
            task.id.c_str(),
            label.c_str(),
            max_xy_offset);
        return false;
    }
    if (std::abs(offset.dz) > max_z_offset) {
        RCLCPP_ERROR(
            node->get_logger(),
            "任务 %s 的 %s dz 超过安全限制 %.3f m。",
            task.id.c_str(),
            label.c_str(),
            max_z_offset);
        return false;
    }
    if (offset.descend <= 0.0 || offset.descend > max_descend) {
        RCLCPP_ERROR(
            node->get_logger(),
            "任务 %s 的 %s descend=%.3f 不合法，必须在 (0, %.3f] m。",
            task.id.c_str(),
            label.c_str(),
            offset.descend,
            max_descend);
        return false;
    }
    return true;
}

bool loadPickPlaceTasks(rclcpp::Node::SharedPtr node,
                        const std::string& task_file,
                        double default_pick_descend,
                        double default_pick_slow_final_descend,
                        double default_place_descend,
                        double default_place_slow_final_descend,
                        double max_xy_offset,
                        double max_z_offset,
                        double max_descend,
                        std::vector<PickPlaceTask>& tasks) {
    tasks.clear();
    if (task_file.empty()) {
        PickPlaceTask task;
        task.id = "default_zero_offset";
        task.pick.descend = default_pick_descend;
        task.pick.slow_final_descend = default_pick_slow_final_descend;
        task.place.descend = default_place_descend;
        task.place.slow_final_descend = default_place_slow_final_descend;
        tasks.push_back(task);
        return true;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(task_file);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "读取 task_file 失败: %s, error=%s", task_file.c_str(), error.what());
        return false;
    }

    const YAML::Node task_nodes = root["tasks"];
    if (!task_nodes || !task_nodes.IsSequence()) {
        RCLCPP_ERROR(node->get_logger(), "task_file 必须包含 sequence: tasks:");
        return false;
    }

    for (std::size_t i = 0; i < task_nodes.size(); ++i) {
        const YAML::Node item = task_nodes[i];
        PickPlaceTask task;
        task.id = item["id"] ? item["id"].as<std::string>() : "task_" + std::to_string(i + 1);

        const YAML::Node pick = item["pick"];
        const YAML::Node place = item["place"];
        if (!pick || !place) {
            RCLCPP_ERROR(node->get_logger(), "任务 %s 缺少 pick 或 place 字段。", task.id.c_str());
            return false;
        }

        task.pick.dx = readYamlDouble(pick, "dx", 0.0);
        task.pick.dy = readYamlDouble(pick, "dy", 0.0);
        task.pick.dz = readYamlDouble(pick, "dz", 0.0);
        task.pick.yaw = readYamlYawRadians(pick, 0.0);
        task.pick.descend = readYamlDouble(pick, "descend", default_pick_descend);
        task.pick.slow_final_descend = readYamlDouble(pick, "slow_final_descend", default_pick_slow_final_descend);

        task.place.dx = readYamlDouble(place, "dx", 0.0);
        task.place.dy = readYamlDouble(place, "dy", 0.0);
        task.place.dz = readYamlDouble(place, "dz", 0.0);
        task.place.yaw = readYamlYawRadians(place, 0.0);
        task.place.descend = readYamlDouble(place, "descend", default_place_descend);
        task.place.slow_final_descend = readYamlDouble(place, "slow_final_descend", default_place_slow_final_descend);

        if (!validateTaskOffset(node, task, task.pick, "pick", max_xy_offset, max_z_offset, max_descend) ||
            !validateTaskOffset(node, task, task.place, "place", max_xy_offset, max_z_offset, max_descend)) {
            return false;
        }
        if (task.pick.slow_final_descend < 0.0 || task.pick.slow_final_descend > task.pick.descend) {
            RCLCPP_ERROR(
                node->get_logger(),
                "任务 %s 的 pick slow_final_descend=%.3f 不合法，必须在 [0, descend] 内。",
                task.id.c_str(),
                task.pick.slow_final_descend);
            return false;
        }
        if (task.place.slow_final_descend < 0.0 || task.place.slow_final_descend > task.place.descend) {
            RCLCPP_ERROR(
                node->get_logger(),
                "任务 %s 的 place slow_final_descend=%.3f 不合法，必须在 [0, descend] 内。",
                task.id.c_str(),
                task.place.slow_final_descend);
            return false;
        }
        RCLCPP_INFO(
            node->get_logger(),
            "加载任务 %s: pick(dx=%.3f, dy=%.3f, dz=%.3f, yaw=%.3f rad, descend=%.3f, slow_final_descend=%.3f), place(dx=%.3f, dy=%.3f, dz=%.3f, yaw=%.3f rad, descend=%.3f, slow_final_descend=%.3f)",
            task.id.c_str(),
            task.pick.dx,
            task.pick.dy,
            task.pick.dz,
            task.pick.yaw,
            task.pick.descend,
            task.pick.slow_final_descend,
            task.place.dx,
            task.place.dy,
            task.place.dz,
            task.place.yaw,
            task.place.descend,
            task.place.slow_final_descend);
        tasks.push_back(task);
    }

    if (tasks.empty()) {
        RCLCPP_ERROR(node->get_logger(), "task_file 没有任何任务。");
        return false;
    }
    return true;
}

// 闭环等待机器人执行完刚发送的 URScript：监听 ur_robot_driver 的
// /io_and_status_controller/robot_program_running（std_msgs/Bool，latched）。
// 流程：等它跳 true（脚本开始执行）→ 再等回 false（执行完毕）即提前返回。
// max_wait_seconds 仅作硬超时上限；若一直观察不到 true（信号语义不符/脚本过短），
// 则回退为固定睡眠 max_wait_seconds，保证与改动前一致的串行安全（零回归）。
void waitForRobotProgramComplete(rclcpp::Node::SharedPtr node, double max_wait_seconds) {
    using namespace std::chrono;
    if (max_wait_seconds <= 0.0) {
        return;
    }

    auto running = std::make_shared<std::atomic<bool>>(false);
    auto have_prog = std::make_shared<std::atomic<bool>>(false);
    // 关节运动状态（用于 program_running 不可用时的兜底）。
    // 用"关节位置在窗口内是否变化"判断运动，而不是关节速度——因为放置末段
    // 是 2mm/s 的极慢下降，关节速度极低会被速度阈值误判为静止，从而过早判定完成、
    // 把下一条脚本提前发给机器人造成串行冲突。位置变化对慢速下降同样敏感。
    auto ref_pos = std::make_shared<std::vector<double>>();  // 仅在回调线程内访问
    auto last_change_t = std::make_shared<std::atomic<double>>(0.0);  // 上次关节位置发生显著变化的时刻
    auto ever_moved = std::make_shared<std::atomic<bool>>(false);

    // 位置变化阈值：远高于编码器噪声(<1e-4)，又远低于慢速下降在窗口内累计的位移。
    const double pos_epsilon = 0.003;  // rad
    // 所需静止窗口（须 > 脚本中夹爪等待造成的臂真静止时长，约 gripper_wait+0.5s）。
    const double still_window =
        std::max(1.0, node->get_parameter_or<double>("urscript_done_still_seconds", 5.0));

    const auto t0 = steady_clock::now();
    auto elapsed = [&]() { return duration<double>(steady_clock::now() - t0).count(); };

    // 发布端为 latched，订阅端用 transient_local 以匹配 QoS 并取到最新状态。
    rclcpp::QoS prog_qos(rclcpp::KeepLast(1));
    prog_qos.transient_local();
    auto prog_sub = node->create_subscription<std_msgs::msg::Bool>(
        "/io_and_status_controller/robot_program_running", prog_qos,
        [running, have_prog](const std_msgs::msg::Bool::SharedPtr msg) {
            running->store(msg->data);
            have_prog->store(true);
        });

    auto js_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [ref_pos, last_change_t, ever_moved, pos_epsilon, &elapsed](
            const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->position.empty()) return;
            const double now = elapsed();
            if (ref_pos->empty()) {
                *ref_pos = msg->position;
                last_change_t->store(now);
                return;
            }
            double maxd = 0.0;
            const std::size_t n = std::min(ref_pos->size(), msg->position.size());
            for (std::size_t i = 0; i < n; ++i)
                maxd = std::max(maxd, std::fabs(msg->position[i] - (*ref_pos)[i]));
            if (maxd > pos_epsilon) {
                // 关节位置发生了显著变化(含慢速下降) → 视为"在动"，刷新参考与时刻。
                *ref_pos = msg->position;
                last_change_t->store(now);
                ever_moved->store(true);
            }
        });

    const double start_timeout = std::min(8.0, max_wait_seconds);  // 等"开始执行"的最长时间
    const double settle_after = 0.3;                               // 完成后小停顿

    // Phase A：等脚本开始执行（program_running -> true）。
    bool prog_started = false;
    while (elapsed() < start_timeout) {
        if (have_prog->load() && running->load()) { prog_started = true; break; }
        std::this_thread::sleep_for(milliseconds(20));
    }

    if (prog_started) {
        // —— 首选：用 program_running 闭环 ——
        RCLCPP_INFO(node->get_logger(),
            "机器人程序开始执行，闭环等待其完成（program_running，上限 %.1fs）…", max_wait_seconds);
        while (elapsed() < max_wait_seconds) {
            if (!running->load()) {
                RCLCPP_INFO(node->get_logger(),
                    "检测到机器人程序结束（实际耗时 %.1fs），提前进入下一步。", elapsed());
                std::this_thread::sleep_for(duration<double>(settle_after));
                return;
            }
            std::this_thread::sleep_for(milliseconds(20));
        }
        RCLCPP_WARN(node->get_logger(),
            "达到 pipeline_wait 上限 %.1fs 仍未检测到程序结束，按超时继续。", max_wait_seconds);
        return;
    }

    // —— 兜底：program_running 不变化，改用关节位置静止检测 ——
    RCLCPP_INFO(node->get_logger(),
        "program_running 未跳变，改用关节位置静止检测（静止窗口 %.1fs，上限 %.1fs）…",
        still_window, max_wait_seconds);
    while (elapsed() < max_wait_seconds) {
        // 必须先观察到机器人确实动过，再要求关节位置连续 still_window 秒无显著变化。
        // 慢速放置下降会持续刷新 last_change_t，因此不会在放置过程中误判完成。
        if (ever_moved->load()) {
            const double still_for = elapsed() - last_change_t->load();
            if (still_for >= still_window) {
                RCLCPP_INFO(node->get_logger(),
                    "关节位置已连续静止 %.1fs（总耗时 %.1fs），判定执行完成，进入下一步。",
                    still_for, elapsed());
                std::this_thread::sleep_for(duration<double>(settle_after));
                return;
            }
        }
        std::this_thread::sleep_for(milliseconds(20));
    }
    RCLCPP_WARN(node->get_logger(),
        "达到 pipeline_wait 上限 %.1fs（静止检测未满足），按超时继续。", max_wait_seconds);
}

bool publishUrscriptProgram(rclcpp::Node::SharedPtr node,
                            const std::string& topic_name,
                            const std::string& program,
                            double wait_seconds) {
    auto publisher = node->create_publisher<std_msgs::msg::String>(topic_name, rclcpp::QoS(1));
    // 2026-07-02: 等到驱动的 urscript_interface 被发现(订阅者出现)再发。
    //   之前固定只等 0.5s, 新起的节点在当前 DDS 环境下常常 0.5s 内没发现驱动 ->
    //   脚本发给"空气"、机器人收不到 -> 臂间歇性不动。改成最多等 10s 直到出现订阅者。
    {
        using namespace std::chrono;
        const auto t0 = steady_clock::now();
        while (publisher->get_subscription_count() == 0 &&
               duration<double>(steady_clock::now() - t0).count() < 10.0) {
            std::this_thread::sleep_for(milliseconds(50));
        }
        if (publisher->get_subscription_count() == 0) {
            RCLCPP_WARN(node->get_logger(),
                "URScript topic 10s 内仍无订阅者: %s (脚本可能发不到机器人, 臂不会动)", topic_name.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "URScript 订阅者已发现, 准备发送。");
            std::this_thread::sleep_for(milliseconds(300));  // 连接建立稳一下
        }
    }

    std_msgs::msg::String msg;
    msg.data = program;
    RCLCPP_INFO(node->get_logger(), "发送 URScript movej/movel pipeline 到: %s", topic_name.c_str());
    publisher->publish(msg);

    // 闭环等待真实执行完成（替代原先的固定 sleep(wait_seconds)，消除轮次间空等）。
    waitForRobotProgramComplete(node, wait_seconds);
    return true;
}

bool executeUrscriptJointPtpPipeline(rclcpp::Node::SharedPtr node,
                                     const std::string& topic_name,
                                     const std::string& point1_joints_text,
                                     const std::string& point2_joints_text,
                                     const std::string& home_joints_text,
                                     bool return_home,
                                     double dwell_seconds,
                                     double movej_acceleration,
                                     double movej_velocity,
                                     double wait_seconds) {
    const auto point1_joints = parseJointList(point1_joints_text);
    const auto point2_joints = parseJointList(point2_joints_text);
    const auto home_joints = parseJointList(home_joints_text);

    if (!point1_joints_text.empty() && !point1_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_pregrasp_joints 格式错误，需要 6 个弧度值。");
        return false;
    }
    if (!point2_joints_text.empty() && !point2_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_preplace_joints 格式错误，需要 6 个弧度值。");
        return false;
    }
    if (!home_joints_text.empty() && !home_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_home_joints 格式错误，需要 6 个弧度值。");
        return false;
    }
    if (!point1_joints && !point2_joints) {
        RCLCPP_ERROR(
            node->get_logger(),
            "joint_ptp 模式至少需要 urscript_pregrasp_joints 或 urscript_preplace_joints 中的一组关节值。");
        return false;
    }

    std::ostringstream script;
    script << "def ur5_simple_joint_ptp():\n";
    script << "  textmsg(\"ur5 simple joint point-to-point start\")\n";
    if (return_home) {
        if (home_joints) {
            script << "  home_joints = " << jointListToUrscript(*home_joints) << "\n";
        } else {
            script << "  home_joints = get_actual_joint_positions()\n";
        }
    }
    if (point1_joints) {
        script << "  movej(" << jointListToUrscript(*point1_joints) << ", a="
               << formatUrscriptNumber(movej_acceleration) << ", v="
               << formatUrscriptNumber(movej_velocity) << ")\n";
        script << "  sleep(" << formatUrscriptNumber(dwell_seconds) << ")\n";
    }
    if (point2_joints) {
        script << "  movej(" << jointListToUrscript(*point2_joints) << ", a="
               << formatUrscriptNumber(movej_acceleration) << ", v="
               << formatUrscriptNumber(movej_velocity) << ")\n";
        script << "  sleep(" << formatUrscriptNumber(dwell_seconds) << ")\n";
    }
    if (return_home) {
        script << "  movej(home_joints, a="
               << formatUrscriptNumber(movej_acceleration) << ", v="
               << formatUrscriptNumber(movej_velocity) << ")\n";
    }
    script << "  textmsg(\"ur5 simple joint point-to-point done\")\n";
    script << "end\n";

    if (point1_joints) {
        RCLCPP_INFO(node->get_logger(), "Joint PTP point1: %s", jointListToUrscript(*point1_joints).c_str());
    }
    if (point2_joints) {
        RCLCPP_INFO(node->get_logger(), "Joint PTP point2: %s", jointListToUrscript(*point2_joints).c_str());
    }
    if (return_home && home_joints) {
        RCLCPP_INFO(node->get_logger(), "Joint PTP home:   %s", jointListToUrscript(*home_joints).c_str());
    } else if (return_home) {
        RCLCPP_INFO(node->get_logger(), "Joint PTP home:   使用程序启动时的当前关节姿态");
    }
    return publishUrscriptProgram(node, topic_name, script.str(), wait_seconds);
}

bool printCurrentJointsForUrscript(rclcpp::Node::SharedPtr node,
                                   moveit::planning_interface::MoveGroupInterface& arm) {
    const std::vector<std::string> names = arm.getJointNames();
    const std::vector<double> values = arm.getCurrentJointValues();

    if (names.size() != values.size() || values.size() != 6) {
        RCLCPP_ERROR(
            node->get_logger(),
            "无法读取 6 个 UR5 关节值：names=%zu, values=%zu",
            names.size(),
            values.size());
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "当前 MoveGroup 关节顺序和值如下，直接用于 URScript movej:");
    for (size_t i = 0; i < values.size(); ++i) {
        RCLCPP_INFO(
            node->get_logger(),
            "  %s = %.15f",
            names[i].c_str(),
            values[i]);
    }

    RCLCPP_INFO(
        node->get_logger(),
        "复制这一行作为 launch 参数值: \"%s\"",
        jointListToUrscript(values).substr(1, jointListToUrscript(values).size() - 2).c_str());
    return true;
}

bool printCurrentPoseForCalibration(rclcpp::Node::SharedPtr node,
                                    moveit::planning_interface::MoveGroupInterface& arm) {
    const geometry_msgs::msg::Pose pose = arm.getCurrentPose().pose;
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);

    if (q.length2() < 1e-12) {
        RCLCPP_ERROR(node->get_logger(), "当前 tool0 四元数无效，无法打印标定位姿。");
        return false;
    }
    q.normalize();

    tf2::Matrix3x3 rotation(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rotation.getRPY(roll, pitch, yaw);

    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    RCLCPP_INFO(node->get_logger(), "当前 tool0 位姿可用于手眼标定采样。frame=%s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(
        node->get_logger(),
        "base_tool_xyz_quat_xyzw: [%.9f, %.9f, %.9f, %.9f, %.9f, %.9f, %.9f]",
        pose.position.x,
        pose.position.y,
        pose.position.z,
        q.x(),
        q.y(),
        q.z(),
        q.w());
    RCLCPP_INFO(
        node->get_logger(),
        "base_tool_xyz_rpy_deg: [%.9f, %.9f, %.9f, %.6f, %.6f, %.6f]",
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll * kRadiansToDegrees,
        pitch * kRadiansToDegrees,
        yaw * kRadiansToDegrees);
    RCLCPP_INFO(node->get_logger(), "T_base_tool:");
    RCLCPP_INFO(
        node->get_logger(),
        "  - [%.9f, %.9f, %.9f, %.9f]",
        rotation[0][0],
        rotation[0][1],
        rotation[0][2],
        pose.position.x);
    RCLCPP_INFO(
        node->get_logger(),
        "  - [%.9f, %.9f, %.9f, %.9f]",
        rotation[1][0],
        rotation[1][1],
        rotation[1][2],
        pose.position.y);
    RCLCPP_INFO(
        node->get_logger(),
        "  - [%.9f, %.9f, %.9f, %.9f]",
        rotation[2][0],
        rotation[2][1],
        rotation[2][2],
        pose.position.z);
    RCLCPP_INFO(node->get_logger(), "  - [0.000000000, 0.000000000, 0.000000000, 1.000000000]");
    return true;
}

// ===== 斜抓(tilt)实验模式: 与主线 joint_pick_place 完全分离 =====
// 读 pick_place_task_tilt.yaml(tilt_pick: grasp_xyz_base/pre_xyz_base/grasp_rotvec_base),
// 沿物体 z 轴方向接近并下探抓取。只做 pick(+回 home), 不做 place。
bool executeTiltPickPipeline(rclcpp::Node::SharedPtr node,
                             const std::string& topic_name,
                             const std::string& gripper_control_mode,
                             const std::string& task_file,
                             const std::string& home_joints_text,
                             int gripper_socket_speed,
                             int gripper_socket_force,
                             double gripper_socket_wait_seconds,
                             double movej_acceleration,
                             double movej_velocity,
                             double movel_acceleration,
                             double movel_velocity,
                             double pipeline_wait_seconds,
                             bool stop_at_pre) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(task_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "tilt: 读取 task_file 失败: %s (%s)", task_file.c_str(), e.what());
        return false;
    }
    const YAML::Node tp = root["tilt_pick"];
    if (!tp) {
        RCLCPP_ERROR(node->get_logger(), "tilt: task_file 缺少 tilt_pick: 段");
        return false;
    }
    auto vec3 = [](const YAML::Node& n) {
        std::vector<double> v;
        if (n) { for (std::size_t i = 0; i < n.size(); ++i) v.push_back(n[i].as<double>()); }
        return v;
    };
    std::vector<double> grasp = vec3(tp["grasp_xyz_base"]);
    std::vector<double> pre = vec3(tp["pre_xyz_base"]);
    std::vector<double> rv = vec3(tp["grasp_rotvec_base"]);
    if (grasp.size() != 3 || pre.size() != 3 || rv.size() != 3) {
        RCLCPP_ERROR(node->get_logger(), "tilt: grasp_xyz_base/pre_xyz_base/grasp_rotvec_base 必须各3个数");
        return false;
    }
    const bool use_socket = gripper_control_mode == "robotiq_socket";

    auto pstr = [&](const std::vector<double>& p) {
        std::ostringstream o;
        o << "p[" << formatUrscriptNumber(p[0]) << ", " << formatUrscriptNumber(p[1]) << ", "
          << formatUrscriptNumber(p[2]) << ", " << formatUrscriptNumber(rv[0]) << ", "
          << formatUrscriptNumber(rv[1]) << ", " << formatUrscriptNumber(rv[2]) << "]";
        return o.str();
    };

    std::ostringstream script;
    script << "def tilt_pick():\n";
    if (use_socket) {
        appendRobotiqSocketSetup(script, gripper_socket_speed, gripper_socket_force);
    }
    script << "  pre_pose = " << pstr(pre) << "\n";
    script << "  grasp_pose = " << pstr(grasp) << "\n";
    if (use_socket) {
        appendRobotiqSocketMove(script, 0, gripper_socket_wait_seconds);  // 张开
    }
    // 用逆解到斜姿态 pre, 然后沿接近轴直线下探到 grasp(pre/grasp 仅差 standoff*接近轴, movel直线即沿斜轴)
    script << "  q_pre = get_inverse_kin(pre_pose, qnear=get_actual_joint_positions())\n";
    script << "  movej(q_pre, a=" << formatUrscriptNumber(movej_acceleration)
           << ", v=" << formatUrscriptNumber(movej_velocity) << ")\n";
    script << "  sleep(0.3)\n";
    if (stop_at_pre) {
        // 只到"观察高度的倾斜姿态"就停, 供肉眼确认倾斜方向(不下探/不闭爪)。
        script << "  textmsg(\"tilt: 已到 pre(观察高度+倾斜), stop_at_pre=on, 不下探\")\n";
    } else {
        script << "  movel(grasp_pose, a=" << formatUrscriptNumber(movel_acceleration)
               << ", v=" << formatUrscriptNumber(movel_velocity) << ")\n";  // 沿斜轴下探
        if (use_socket) {
            appendRobotiqSocketMove(script, 255, gripper_socket_wait_seconds);  // 闭合
        }
        script << "  sleep(0.5)\n";
        script << "  movel(pre_pose, a=" << formatUrscriptNumber(movel_acceleration)
               << ", v=" << formatUrscriptNumber(movel_velocity) << ")\n";  // 沿斜轴抬回
        const auto home = parseJointList(home_joints_text);
        if (home) {
            script << "  movej(" << jointListToUrscript(*home) << ", a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(movej_velocity) << ")\n";
        }
        script << "  textmsg(\"tilt pick done\")\n";
    }
    script << "end\n";

    RCLCPP_INFO(node->get_logger(),
                "tilt: grasp=[%.4f, %.4f, %.4f] rotvec=[%.4f, %.4f, %.4f]",
                grasp[0], grasp[1], grasp[2], rv[0], rv[1], rv[2]);
    if (!publishUrscriptProgram(node, topic_name, script.str(), pipeline_wait_seconds)) {
        return false;
    }
    waitForRobotProgramComplete(node, pipeline_wait_seconds);
    return true;
}

bool executeUrscriptPtpPipeline(rclcpp::Node::SharedPtr node,
                                const std::string& topic_name,
                                const std::string& gripper_control_mode,
                                const geometry_msgs::msg::Pose& pregrasp_pose,
                                const geometry_msgs::msg::Pose& grasp_pose,
                                const geometry_msgs::msg::Pose& preplace_pose,
                                const geometry_msgs::msg::Pose& place_pose,
                                const std::string& pregrasp_joints_text,
                                const std::string& preplace_joints_text,
                                const std::string& home_joints_text,
                                const std::vector<PickPlaceTask>& tasks,
                                bool return_home,
                                double grasp_hover,
                                double place_hover,
                                double movej_acceleration,
                                double movej_velocity,
                                double pick_approach_movej_velocity,
                                double movel_acceleration,
                                double movel_velocity,
                                double pick_approach_movel_velocity,
                                double pick_lift_velocity,
                                double descend_acceleration,
                                double descend_velocity,
                                double place_descend_velocity,
                                bool place_wiggle_enabled,
                                double place_wiggle_xy_amplitude,
                                double place_wiggle_z_amplitude,
                                double place_wiggle_velocity,
                                int place_wiggle_steps,
                                double place_lift_after_release,
                                double gripper_socket_wait_seconds,
                                double gripper_socket_open_wait_seconds,
                                int gripper_socket_speed,
                                int gripper_socket_force,
                                int gripper_socket_release_position,
                                double gripper_socket_release_wait_seconds,
                                const OnRobotRgConfig& onrobot_rg_config,
                                int onrobot_rg_open_width_mm,
                                int onrobot_rg_close_width_mm,
                                int onrobot_rg_pregrasp_width_mm,
                                int onrobot_rg_release_width_mm,
                                int onrobot_rg_force_n,
                                double place_settle_wait_seconds,
                                double wait_seconds) {
    if (gripper_control_mode != "none" &&
        gripper_control_mode != "urscript" &&
        gripper_control_mode != "robotiq_socket" &&
        gripper_control_mode != "onrobot_rg" &&
        gripper_control_mode != "onrobot_io") {
        RCLCPP_WARN(
            node->get_logger(),
            "URScript pipeline 只会内联 none/urscript/robotiq_socket/onrobot_rg/onrobot_io 夹爪模式；当前 gripper_control_mode=%s，本次会跳过夹爪动作。",
            gripper_control_mode.c_str());
    }

    const bool use_robotiq_urscript = gripper_control_mode == "urscript";
    const bool use_robotiq_socket = gripper_control_mode == "robotiq_socket";
    const bool use_onrobot_rg = gripper_control_mode == "onrobot_rg";
    const bool use_onrobot_io = gripper_control_mode == "onrobot_io";
    const int io_gripper_tool_dout =
        std::clamp(node->get_parameter_or<int>("io_gripper_tool_dout", 0), 0, 1);  // pin16=tool DO 0
    const double io_gripper_wait_seconds =
        std::max(0.0, node->get_parameter_or<double>("io_gripper_wait_seconds", 1.0));
    // 抓取前的"半闭合"位置：0=完全张开(原行为)，255=完全闭合。下降前先合到此位置，
    // 下降到位后再完全闭合(255)。用于"先半合→下降→全合"的抓取时序。
    const int gripper_socket_pregrasp_position =
        std::clamp(node->get_parameter_or<int>("gripper_socket_pregrasp_position", 0), 0, 255);
    const double gripper_socket_pregrasp_wait_seconds =
        std::max(0.0, node->get_parameter_or<double>("gripper_socket_pregrasp_wait_seconds", 1.0));
    RCLCPP_INFO(node->get_logger(),
        "[抓取时序] gripper_socket_pregrasp_position=%d (0=下降前完全张开/原行为, >0=下降前先半合到该值, 下降到位后再合到255), pregrasp_wait=%.2fs",
        gripper_socket_pregrasp_position, gripper_socket_pregrasp_wait_seconds);
    const std::string pregrasp = poseToUrscriptPose(pregrasp_pose);
    const std::string grasp = poseToUrscriptPose(grasp_pose);
    const std::string preplace = poseToUrscriptPose(preplace_pose);
    const std::string place = poseToUrscriptPose(place_pose);
    const auto pregrasp_joints = parseJointList(pregrasp_joints_text);
    const auto preplace_joints = parseJointList(preplace_joints_text);
    const auto home_joints = parseJointList(home_joints_text);

    if (!pregrasp_joints_text.empty() && !pregrasp_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_pregrasp_joints 格式错误，需要 6 个弧度值，例如: 0.0,-1.57,1.57,-1.57,-1.57,0.0");
        return false;
    }
    if (!preplace_joints_text.empty() && !preplace_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_preplace_joints 格式错误，需要 6 个弧度值，例如: 0.0,-1.57,1.57,-1.57,-1.57,0.0");
        return false;
    }
    if (!home_joints_text.empty() && !home_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_home_joints 格式错误，需要 6 个弧度值，例如: 0.0,-1.57,1.57,-1.57,-1.57,0.0");
        return false;
    }

    std::ostringstream script;
    script << "def ur5_pick_place_pipeline():\n";
    script << "  textmsg(\"ur5 normal pick and place pipeline start\")\n";
    if (return_home) {
        if (home_joints) {
            script << "  home_joints = " << jointListToUrscript(*home_joints) << "\n";
        } else if (pregrasp_joints) {
            script << "  home_joints = " << jointListToUrscript(*pregrasp_joints) << "\n";
        } else {
            script << "  home_joints = get_actual_joint_positions()\n";
        }
    }
    if (use_robotiq_urscript) {
        script << "  rq_reset()\n";
        script << "  sleep(0.5)\n";
        script << "  rq_activate_and_wait()\n";
    } else if (use_robotiq_socket) {
        appendRobotiqSocketSetup(script, gripper_socket_speed, gripper_socket_force);
    } else if (use_onrobot_rg) {
        appendOnRobotRgHelpers(script);
        script << "  onrobot_rg_ready = onrobot_rg_powerup()\n";
    }
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        const PickPlaceTask& task = tasks[i];
        const std::string index = std::to_string(i);
        const double pick_descend = task.pick.descend > 0.0 ? task.pick.descend : grasp_hover;
        const double pick_slow_final_descend =
            std::clamp(task.pick.slow_final_descend, 0.0, pick_descend);
        const double pick_fast_descend = pick_descend - pick_slow_final_descend;
        const double place_descend = task.place.descend > 0.0 ? task.place.descend : place_hover;
        const double place_slow_final_descend =
            std::clamp(task.place.slow_final_descend, 0.0, place_descend);
        const double place_fast_descend = place_descend - place_slow_final_descend;

        script << "  textmsg(\"task " << sanitizeUrscriptText(task.id) << " start\")\n";
        if (pregrasp_joints) {
            std::vector<double> pick_joints = *pregrasp_joints;
            pick_joints[5] += task.pick.yaw;
            script << "  movej(" << jointListToUrscript(pick_joints) << ", a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(pick_approach_movej_velocity) << ")\n";
            script << "  pick_pre_" << index << " = get_actual_tcp_pose()\n";
        } else {
            script << "  pick_pre_" << index << " = " << pregrasp << "\n";
            script << "  q_pick_pre_" << index << " = get_inverse_kin(pick_pre_" << index
                   << ", qnear=get_actual_joint_positions())\n";
            script << "  movej(q_pick_pre_" << index << ", a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(pick_approach_movej_velocity) << ")\n";
        }

        script << "  pick_pre_" << index << "[0] = pick_pre_" << index << "[0] + "
               << formatUrscriptNumber(task.pick.dx) << "\n";
        script << "  pick_pre_" << index << "[1] = pick_pre_" << index << "[1] + "
               << formatUrscriptNumber(task.pick.dy) << "\n";
        script << "  pick_pre_" << index << "[2] = pick_pre_" << index << "[2] + "
               << formatUrscriptNumber(task.pick.dz) << "\n";
        if (!pregrasp_joints) {
            script << "  pick_pre_" << index << "[5] = pick_pre_" << index << "[5] + "
                   << formatUrscriptNumber(task.pick.yaw) << "\n";
        }
        script << "  movel(pick_pre_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(pick_approach_movel_velocity) << ")\n";

        if (use_robotiq_urscript) {
            script << "  rq_open_and_wait()\n";
        } else if (use_robotiq_socket) {
            // 下降前先半闭合(若 pregrasp_position>0)；=0 时等价于原来的完全张开。
            appendRobotiqSocketMove(
                script,
                gripper_socket_pregrasp_position,
                gripper_socket_pregrasp_position > 0 ? gripper_socket_pregrasp_wait_seconds
                                                     : gripper_socket_open_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_pregrasp_width_mm,
                onrobot_rg_force_n,
                gripper_socket_open_wait_seconds,
                "pregrasp");
        } else if (use_onrobot_io) {
            appendIoGripperMove(script, io_gripper_tool_dout, false, io_gripper_wait_seconds, "pregrasp open");
        }
        script << "  sleep(0.2)\n";
        if (pick_slow_final_descend > 0.0001 && pick_fast_descend > 0.0001) {
            script << "  pick_slow_start_" << index << " = p[pick_pre_" << index
                   << "[0], pick_pre_" << index << "[1], pick_pre_" << index << "[2] - "
                   << formatUrscriptNumber(pick_fast_descend)
                   << ", pick_pre_" << index << "[3], pick_pre_" << index << "[4], pick_pre_"
                   << index << "[5]]\n";
            script << "  movel(pick_slow_start_" << index << ", a="
                   << formatUrscriptNumber(descend_acceleration) << ", v="
                   << formatUrscriptNumber(descend_velocity) << ")\n";
        }
        script << "  pick_down_" << index << " = p[pick_pre_" << index << "[0], pick_pre_" << index
               << "[1], pick_pre_" << index << "[2] - " << formatUrscriptNumber(pick_descend)
               << ", pick_pre_" << index << "[3], pick_pre_" << index << "[4], pick_pre_"
               << index << "[5]]\n";
        script << "  movel(pick_down_" << index << ", a="
               << formatUrscriptNumber(descend_acceleration) << ", v="
               << formatUrscriptNumber(
                      pick_slow_final_descend > 0.0001 ? place_descend_velocity : descend_velocity)
               << ")\n";
        if (use_robotiq_urscript) {
            script << "  rq_close_and_wait()\n";
        } else if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, 255, gripper_socket_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_close_width_mm,
                onrobot_rg_force_n,
                gripper_socket_wait_seconds,
                "close");
        } else if (use_onrobot_io) {
            appendIoGripperMove(script, io_gripper_tool_dout, true, io_gripper_wait_seconds, "grasp close");
        }
        script << "  sleep(0.5)\n";
        script << "  movel(pick_pre_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(pick_lift_velocity) << ")\n";

        if (preplace_joints) {
            std::vector<double> place_joints = *preplace_joints;
            place_joints[5] += task.place.yaw;
            script << "  movej(" << jointListToUrscript(place_joints) << ", a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(movej_velocity) << ")\n";
            script << "  place_pre_" << index << " = get_actual_tcp_pose()\n";
        } else {
            script << "  place_pre_" << index << " = " << preplace << "\n";
            script << "  q_place_pre_" << index << " = get_inverse_kin(place_pre_" << index
                   << ", qnear=get_actual_joint_positions())\n";
            script << "  movej(q_place_pre_" << index << ", a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(movej_velocity) << ")\n";
        }

        script << "  place_pre_" << index << "[0] = place_pre_" << index << "[0] + "
               << formatUrscriptNumber(task.place.dx) << "\n";
        script << "  place_pre_" << index << "[1] = place_pre_" << index << "[1] + "
               << formatUrscriptNumber(task.place.dy) << "\n";
        script << "  place_pre_" << index << "[2] = place_pre_" << index << "[2] + "
               << formatUrscriptNumber(task.place.dz) << "\n";
        if (!preplace_joints) {
            script << "  place_pre_" << index << "[5] = place_pre_" << index << "[5] + "
                   << formatUrscriptNumber(task.place.yaw) << "\n";
        }
        script << "  movel(place_pre_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(movel_velocity) << ")\n";

        if (place_fast_descend > 0.0001) {
            script << "  place_slow_start_" << index << " = p[place_pre_" << index
                   << "[0], place_pre_" << index << "[1], place_pre_" << index << "[2] - "
                   << formatUrscriptNumber(place_fast_descend)
                   << ", place_pre_" << index << "[3], place_pre_" << index << "[4], place_pre_"
                   << index << "[5]]\n";
            script << "  movel(place_slow_start_" << index << ", a="
                   << formatUrscriptNumber(descend_acceleration) << ", v="
                   << formatUrscriptNumber(descend_velocity) << ")\n";
        }
        script << "  place_down_" << index << " = p[place_pre_" << index << "[0], place_pre_" << index
               << "[1], place_pre_" << index << "[2] - " << formatUrscriptNumber(place_descend)
               << ", place_pre_" << index << "[3], place_pre_" << index << "[4], place_pre_"
               << index << "[5]]\n";
        const bool use_place_wiggle =
            place_wiggle_enabled && place_slow_final_descend > 0.002 && place_wiggle_steps >= 4;
        if (use_place_wiggle) {
            const double xy_amp = std::clamp(place_wiggle_xy_amplitude, 0.0, 0.002);
            const double requested_wiggle_velocity =
                std::clamp(place_wiggle_velocity, 0.001, 0.050);
            const double wiggle_dt =
                xy_amp > 0.000001 ? std::clamp(xy_amp / requested_wiggle_velocity, 0.020, 0.100) : 0.020;
            const double wiggle_velocity =
                xy_amp > 0.000001 ? xy_amp / wiggle_dt : requested_wiggle_velocity;
            (void)place_wiggle_z_amplitude;
            if (xy_amp > 0.000001 && place_descend_velocity > 0.000001) {
                script << "  place_wiggle_target_z_" << index << " = place_down_" << index << "[2]\n";
                script << "  place_wiggle_phase_" << index << " = 0\n";
                script << "  place_wiggle_now_" << index << " = get_actual_tcp_pose()\n";
                script << "  while place_wiggle_now_" << index << "[2] > place_wiggle_target_z_"
                       << index << " + 0.00005:\n";
                script << "    place_wiggle_dt_" << index << " = " << formatUrscriptNumber(wiggle_dt) << "\n";
                script << "    place_wiggle_remain_" << index << " = place_wiggle_now_" << index
                       << "[2] - place_wiggle_target_z_" << index << "\n";
                script << "    if place_wiggle_remain_" << index << " < "
                       << formatUrscriptNumber(place_descend_velocity) << " * place_wiggle_dt_"
                       << index << ":\n";
                script << "      place_wiggle_dt_" << index << " = place_wiggle_remain_" << index
                       << " / " << formatUrscriptNumber(place_descend_velocity) << "\n";
                script << "    end\n";
                script << "    if place_wiggle_phase_" << index << " == 0:\n";
                script << "      speedl([" << formatUrscriptNumber(wiggle_velocity)
                       << ", 0.0, -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 1:\n";
                script << "      speedl([-" << formatUrscriptNumber(wiggle_velocity)
                       << ", 0.0, -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 2:\n";
                script << "      speedl([-" << formatUrscriptNumber(wiggle_velocity)
                       << ", 0.0, -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 3:\n";
                script << "      speedl([" << formatUrscriptNumber(wiggle_velocity)
                       << ", 0.0, -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 4:\n";
                script << "      speedl([0.0, " << formatUrscriptNumber(wiggle_velocity)
                       << ", -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 5:\n";
                script << "      speedl([0.0, -" << formatUrscriptNumber(wiggle_velocity)
                       << ", -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    elif place_wiggle_phase_" << index << " == 6:\n";
                script << "      speedl([0.0, -" << formatUrscriptNumber(wiggle_velocity)
                       << ", -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    else:\n";
                script << "      speedl([0.0, " << formatUrscriptNumber(wiggle_velocity)
                       << ", -" << formatUrscriptNumber(place_descend_velocity)
                       << ", 0.0, 0.0, 0.0], " << formatUrscriptNumber(descend_acceleration)
                       << ", place_wiggle_dt_" << index << ")\n";
                script << "    end\n";
                script << "    place_wiggle_phase_" << index << " = place_wiggle_phase_" << index << " + 1\n";
                script << "    if place_wiggle_phase_" << index << " >= 8:\n";
                script << "      place_wiggle_phase_" << index << " = 0\n";
                script << "    end\n";
                script << "    place_wiggle_now_" << index << " = get_actual_tcp_pose()\n";
                script << "  end\n";
                script << "  stopl(" << formatUrscriptNumber(descend_acceleration) << ")\n";
            }
        }
        script << "  movel(place_down_" << index << ", a="
               << formatUrscriptNumber(descend_acceleration) << ", v="
               << formatUrscriptNumber(place_descend_velocity) << ")\n";
        script << "  sync()\n";
        script << "  sleep(" << formatUrscriptNumber(std::max(0.0, place_settle_wait_seconds)) << ")\n";
        if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, gripper_socket_release_position, gripper_socket_release_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_release_width_mm,
                onrobot_rg_force_n,
                gripper_socket_release_wait_seconds,
                "release");
        } else if (use_onrobot_io) {
            appendIoGripperMove(script, io_gripper_tool_dout, false, io_gripper_wait_seconds, "place release");
        }
        const double release_lift = place_lift_after_release > 0.0001
            ? std::clamp(place_lift_after_release, 0.0, place_descend)
            : place_descend;
        script << "  place_release_lift_" << index << " = p[place_down_" << index
               << "[0], place_down_" << index << "[1], place_down_" << index << "[2] + "
               << formatUrscriptNumber(release_lift)
               << ", place_down_" << index << "[3], place_down_" << index << "[4], place_down_"
               << index << "[5]]\n";
        script << "  movel(place_release_lift_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(movel_velocity) << ")\n";
        script << "  sync()\n";
        if (use_robotiq_urscript) {
            script << "  rq_open_and_wait()\n";
        } else if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, 0, gripper_socket_open_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_open_width_mm,
                onrobot_rg_force_n,
                gripper_socket_open_wait_seconds,
                "open");
        } else if (use_onrobot_io) {
            appendIoGripperMove(script, io_gripper_tool_dout, false, io_gripper_wait_seconds, "open");
        }
        if (return_home) {
            script << "  movej(home_joints, a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(movej_velocity) << ")\n";
            script << "  sleep(0.5)\n";
        }
        script << "  textmsg(\"task " << sanitizeUrscriptText(task.id) << " done\")\n";
    }
    if (use_robotiq_socket) {
        appendRobotiqSocketClose(script);
    }
    script << "  textmsg(\"ur5 normal pick and place pipeline done\")\n";
    script << "end\n";

    RCLCPP_INFO(node->get_logger(), "URScript pregrasp: %s", pregrasp.c_str());
    RCLCPP_INFO(node->get_logger(), "URScript grasp:    %s", grasp.c_str());
    RCLCPP_INFO(node->get_logger(), "URScript preplace: %s", preplace.c_str());
    RCLCPP_INFO(node->get_logger(), "URScript place:    %s", place.c_str());
    if (return_home && home_joints) {
        RCLCPP_INFO(node->get_logger(), "URScript home:     %s", jointListToUrscript(*home_joints).c_str());
    } else if (return_home && pregrasp_joints) {
        RCLCPP_INFO(node->get_logger(), "URScript home:     使用 pregrasp_joints 作为观察/default pose");
    } else if (return_home) {
        RCLCPP_INFO(node->get_logger(), "URScript home:     使用程序启动时的当前关节姿态");
    }
    // 把完整 URScript 落盘，便于核对夹爪/运动指令顺序（调试用）。
    {
        std::ofstream dump("/tmp/last_urscript.txt", std::ios::trunc);
        if (dump) {
            dump << script.str();
            RCLCPP_INFO(node->get_logger(), "完整 URScript 已写入 /tmp/last_urscript.txt");
        }
    }
    return publishUrscriptProgram(node, topic_name, script.str(), wait_seconds);
}


//new mode disassemble_pick_place  
bool executeDisassemblePickPlacePipeline(rclcpp::Node::SharedPtr node,
                                         const std::string& topic_name,
                                         const std::string& gripper_control_mode,
                                         const std::string& pull_joints_text,
                                         const std::string& drop_joints_text,
                                         const std::string& home_joints_text,
                                         const std::vector<PickPlaceTask>& tasks,
                                         bool return_home,
                                         double movej_acceleration,
                                         double movej_velocity,
                                         double movel_acceleration,
                                         double movel_velocity,
                                         double pick_lift_velocity,
                                         double descend_acceleration,
                                         double descend_velocity,
                                         double place_descend_velocity,
                                         double disassemble_extra_lift,
                                         double disassemble_place_offset_x,
                                         double disassemble_place_offset_y,
                                         double disassemble_place_offset_z,
                                         double gripper_socket_wait_seconds,
                                         double gripper_socket_open_wait_seconds,
                                         int gripper_socket_speed,
                                         int gripper_socket_force,
                                         int gripper_socket_release_position,
                                         double gripper_socket_release_wait_seconds,
                                         const OnRobotRgConfig& onrobot_rg_config,
                                         int onrobot_rg_open_width_mm,
                                         int onrobot_rg_close_width_mm,
                                         int onrobot_rg_release_width_mm,
                                         int onrobot_rg_force_n,
                                         double place_settle_wait_seconds,
                                         double wait_seconds) {
    if (gripper_control_mode != "none" &&
        gripper_control_mode != "robotiq_socket" &&
        gripper_control_mode != "onrobot_rg") {
        RCLCPP_WARN(
            node->get_logger(),
            "disassemble_pick_place 只内联 none/robotiq_socket/onrobot_rg 夹爪模式；当前 gripper_control_mode=%s，本次会跳过夹爪动作。",
            gripper_control_mode.c_str());
    }

    const bool use_robotiq_socket = gripper_control_mode == "robotiq_socket";
    const bool use_onrobot_rg = gripper_control_mode == "onrobot_rg";
    const auto pull_joints = parseJointList(pull_joints_text);
    const auto drop_joints = parseJointList(drop_joints_text);
    const auto home_joints = parseJointList(home_joints_text);

    if (!pull_joints) {
        RCLCPP_ERROR(node->get_logger(), "disassemble_pick_place 需要 urscript_pregrasp_joints=6个弧度值作为拔取姿态。");
        return false;
    }
    if (!drop_joints) {
        RCLCPP_ERROR(node->get_logger(), "disassemble_pick_place 需要 urscript_preplace_joints=6个弧度值作为放置姿态。");
        return false;
    }
    if (!home_joints_text.empty() && !home_joints) {
        RCLCPP_ERROR(node->get_logger(), "urscript_home_joints 格式错误，需要 6 个弧度值。");
        return false;
    }

    std::ostringstream script;
    script << "def ur5_disassemble_pick_place():\n";
    script << "  textmsg(\"ur5 disassemble pick place start\")\n";
    if (return_home) {
        if (home_joints) {
            script << "  home_joints = " << jointListToUrscript(*home_joints) << "\n";
        } else {
            script << "  home_joints = get_actual_joint_positions()\n";
        }
    }
    if (use_robotiq_socket) {
        appendRobotiqSocketSetup(script, gripper_socket_speed, gripper_socket_force);
    } else if (use_onrobot_rg) {
        appendOnRobotRgHelpers(script);
        script << "  onrobot_rg_ready = onrobot_rg_powerup()\n";
    }

    for (std::size_t i = 0; i < tasks.size(); ++i) {
        const PickPlaceTask& task = tasks[i];
        const std::string index = std::to_string(i);
        script << "  textmsg(\"disassemble task " << sanitizeUrscriptText(task.id) << " start\")\n";

        std::vector<double> pick_joints = *pull_joints;
        pick_joints[5] += task.pick.yaw;
        script << "  movej(" << jointListToUrscript(pick_joints) << ", a="
               << formatUrscriptNumber(movej_acceleration) << ", v="
               << formatUrscriptNumber(movej_velocity) << ")\n";
        script << "  dis_pick_" << index << " = get_actual_tcp_pose()\n";
        script << "  dis_pick_" << index << "[0] = dis_pick_" << index << "[0] + "
               << formatUrscriptNumber(task.pick.dx) << "\n";
        script << "  dis_pick_" << index << "[1] = dis_pick_" << index << "[1] + "
               << formatUrscriptNumber(task.pick.dy) << "\n";
        script << "  dis_pick_" << index << "[2] = dis_pick_" << index << "[2] + "
               << formatUrscriptNumber(task.pick.dz) << "\n";
        script << "  movel(dis_pick_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(movel_velocity) << ")\n";

        if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, 0, gripper_socket_open_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_open_width_mm,
                onrobot_rg_force_n,
                gripper_socket_open_wait_seconds,
                "open");
        }

        if (task.pick.descend > 0.0001) {
            script << "  dis_grasp_" << index << " = p[dis_pick_" << index << "[0], dis_pick_" << index
                   << "[1], dis_pick_" << index << "[2] - " << formatUrscriptNumber(task.pick.descend)
                   << ", dis_pick_" << index << "[3], dis_pick_" << index << "[4], dis_pick_"
                   << index << "[5]]\n";
            script << "  movel(dis_grasp_" << index << ", a="
                   << formatUrscriptNumber(descend_acceleration) << ", v="
                   << formatUrscriptNumber(descend_velocity) << ")\n";
        } else {
            script << "  dis_grasp_" << index << " = dis_pick_" << index << "\n";
        }

        if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, 255, gripper_socket_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_close_width_mm,
                onrobot_rg_force_n,
                gripper_socket_wait_seconds,
                "close");
        }
        script << "  sleep(0.3)\n";
        script << "  movel(dis_pick_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(pick_lift_velocity) << ")\n";
        if (disassemble_extra_lift > 0.0001) {
            script << "  dis_carry_" << index << " = p[dis_pick_" << index << "[0], dis_pick_" << index
                   << "[1], dis_pick_" << index << "[2] + " << formatUrscriptNumber(disassemble_extra_lift)
                   << ", dis_pick_" << index << "[3], dis_pick_" << index << "[4], dis_pick_"
                   << index << "[5]]\n";
            script << "  movel(dis_carry_" << index << ", a="
                   << formatUrscriptNumber(movel_acceleration) << ", v="
                   << formatUrscriptNumber(pick_lift_velocity) << ")\n";
        }

        std::vector<double> place_joints = *drop_joints;
        place_joints[5] += task.place.yaw;
        script << "  movej(" << jointListToUrscript(place_joints) << ", a="
               << formatUrscriptNumber(movej_acceleration) << ", v="
               << formatUrscriptNumber(movej_velocity) << ")\n";
        script << "  dis_place_" << index << " = get_actual_tcp_pose()\n";
        script << "  dis_place_" << index << "[0] = dis_place_" << index << "[0] + "
               << formatUrscriptNumber(task.place.dx + disassemble_place_offset_x) << "\n";
        script << "  dis_place_" << index << "[1] = dis_place_" << index << "[1] + "
               << formatUrscriptNumber(task.place.dy + disassemble_place_offset_y) << "\n";
        script << "  dis_place_" << index << "[2] = dis_place_" << index << "[2] + "
               << formatUrscriptNumber(task.place.dz + disassemble_place_offset_z) << "\n";
        script << "  movel(dis_place_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(movel_velocity) << ")\n";

        if (task.place.descend > 0.0001) {
            script << "  dis_place_down_" << index << " = p[dis_place_" << index << "[0], dis_place_" << index
                   << "[1], dis_place_" << index << "[2] - " << formatUrscriptNumber(task.place.descend)
                   << ", dis_place_" << index << "[3], dis_place_" << index << "[4], dis_place_"
                   << index << "[5]]\n";
            script << "  movel(dis_place_down_" << index << ", a="
                   << formatUrscriptNumber(descend_acceleration) << ", v="
                   << formatUrscriptNumber(place_descend_velocity) << ")\n";
        } else {
            script << "  dis_place_down_" << index << " = dis_place_" << index << "\n";
        }

        script << "  sync()\n";
        script << "  sleep(" << formatUrscriptNumber(std::max(0.0, place_settle_wait_seconds)) << ")\n";
        if (use_robotiq_socket) {
            appendRobotiqSocketMove(script, gripper_socket_release_position, gripper_socket_release_wait_seconds);
            appendRobotiqSocketMove(script, 0, gripper_socket_open_wait_seconds);
        } else if (use_onrobot_rg) {
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_release_width_mm,
                onrobot_rg_force_n,
                gripper_socket_release_wait_seconds,
                "release");
            appendOnRobotRgMove(
                script,
                onrobot_rg_config,
                onrobot_rg_open_width_mm,
                onrobot_rg_force_n,
                gripper_socket_open_wait_seconds,
                "open");
        }
        script << "  movel(dis_place_" << index << ", a="
               << formatUrscriptNumber(movel_acceleration) << ", v="
               << formatUrscriptNumber(movel_velocity) << ")\n";
        if (disassemble_extra_lift > 0.0001) {
            script << "  dis_place_clear_" << index << " = p[dis_place_" << index << "[0], dis_place_" << index
                   << "[1], dis_place_" << index << "[2] + " << formatUrscriptNumber(disassemble_extra_lift)
                   << ", dis_place_" << index << "[3], dis_place_" << index << "[4], dis_place_"
                   << index << "[5]]\n";
            script << "  movel(dis_place_clear_" << index << ", a="
                   << formatUrscriptNumber(movel_acceleration) << ", v="
                   << formatUrscriptNumber(movel_velocity) << ")\n";
        }
        if (return_home) {
            script << "  movej(home_joints, a="
                   << formatUrscriptNumber(movej_acceleration) << ", v="
                   << formatUrscriptNumber(movej_velocity) << ")\n";
        }
        script << "  textmsg(\"disassemble task " << sanitizeUrscriptText(task.id) << " done\")\n";
    }

    if (use_robotiq_socket) {
        appendRobotiqSocketClose(script);
    }
    script << "  textmsg(\"ur5 disassemble pick place done\")\n";
    script << "end\n";

    RCLCPP_INFO(node->get_logger(), "Disassemble pull joints: %s", jointListToUrscript(*pull_joints).c_str());
    RCLCPP_INFO(node->get_logger(), "Disassemble drop joints: %s", jointListToUrscript(*drop_joints).c_str());
    {
        std::ofstream dump("/tmp/last_urscript.txt", std::ios::trunc);
        if (dump) {
            dump << script.str();
            RCLCPP_INFO(node->get_logger(), "完整拆卸 URScript 已写入 /tmp/last_urscript.txt");
        }
    }
    return publishUrscriptProgram(node, topic_name, script.str(), wait_seconds);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 1. 初始化节点，开启参数自动声明
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("ur5_lego_executor", node_options);
    const std::string gripper_control_mode =
        node->get_parameter_or<std::string>("gripper_control_mode", "none");
    const std::string gripper_action_name =
        node->get_parameter_or<std::string>("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd");
    const std::string gripper_io_service_name =
        node->get_parameter_or<std::string>("gripper_io_service_name", "/io_and_status_controller/set_io");
    const std::string gripper_urscript_topic =
        node->get_parameter_or<std::string>("gripper_urscript_topic", "/urscript_interface/script_command");
    const double gripper_open_position =
        node->get_parameter_or<double>("gripper_open_position", GRIPPER_OPEN);
    const double gripper_close_position =
        node->get_parameter_or<double>("gripper_close_position", GRIPPER_CLOSE);
    const double gripper_max_effort =
        node->get_parameter_or<double>("gripper_max_effort", 80.0);
    const int gripper_io_open_pin =
        node->get_parameter_or<int>("gripper_io_open_pin", 16);
    const int gripper_io_close_pin =
        node->get_parameter_or<int>("gripper_io_close_pin", 17);
    const double gripper_io_pulse_seconds =
        node->get_parameter_or<double>("gripper_io_pulse_seconds", 0.5);
    const bool gripper_io_latching =
        node->get_parameter_or<bool>("gripper_io_latching", false);
    const double gripper_urscript_wait_seconds =
        node->get_parameter_or<double>("gripper_urscript_wait_seconds", 4.0);
    const double gripper_socket_open_wait_seconds =
        node->get_parameter_or<double>("gripper_socket_open_wait_seconds", gripper_urscript_wait_seconds);
    const int gripper_socket_speed =
        std::clamp(node->get_parameter_or<int>("gripper_socket_speed", 180), 0, 255);
    const int gripper_socket_force =
        std::clamp(
            node->get_parameter_or<int>(
                "gripper_socket_force",
                static_cast<int>(std::round(gripper_max_effort))),
            0,
            255);
    const int gripper_socket_release_position =
        std::clamp(node->get_parameter_or<int>("gripper_socket_release_position", 140), 0, 255);
    const double gripper_socket_release_wait_seconds =
        node->get_parameter_or<double>("gripper_socket_release_wait_seconds", 0.5);
    const std::string onrobot_rg_model =
        node->get_parameter_or<std::string>("onrobot_rg_model", "rg2");
    const OnRobotRgConfig onrobot_rg_config = makeOnRobotRgConfig(onrobot_rg_model);
    const int onrobot_rg_open_width_mm =
        clampOnRobotWidth(node->get_parameter_or<int>("onrobot_rg_open_width_mm", 90), onrobot_rg_config);
    const int onrobot_rg_close_width_mm =
        clampOnRobotWidth(node->get_parameter_or<int>("onrobot_rg_close_width_mm", 12), onrobot_rg_config);
    const int onrobot_rg_pregrasp_width_mm =
        clampOnRobotWidth(
            node->get_parameter_or<int>("onrobot_rg_pregrasp_width_mm", onrobot_rg_open_width_mm),
            onrobot_rg_config);
    const int onrobot_rg_release_width_mm =
        clampOnRobotWidth(node->get_parameter_or<int>("onrobot_rg_release_width_mm", 45), onrobot_rg_config);
    const int onrobot_rg_force_n =
        clampOnRobotForce(node->get_parameter_or<int>("onrobot_rg_force_n", 20), onrobot_rg_config);
    const std::string motion_control_mode =
        node->get_parameter_or<std::string>("motion_control_mode", "joint_pick_place");
    const double urscript_movej_acceleration =
        node->get_parameter_or<double>("urscript_movej_acceleration", 2.0);
    const double urscript_movej_velocity =
        node->get_parameter_or<double>("urscript_movej_velocity", 1.2);
    const double urscript_pick_approach_movej_velocity =
        node->get_parameter_or<double>("urscript_pick_approach_movej_velocity", urscript_movej_velocity);
    const double urscript_movel_acceleration =
        node->get_parameter_or<double>("urscript_movel_acceleration", 1.0);
    const double urscript_movel_velocity =
        node->get_parameter_or<double>("urscript_movel_velocity", 0.4);
    const double urscript_pick_approach_movel_velocity =
        node->get_parameter_or<double>("urscript_pick_approach_movel_velocity", urscript_movel_velocity);
    const double urscript_pick_lift_velocity =
        node->get_parameter_or<double>("urscript_pick_lift_velocity", 0.14);
    const double urscript_descend_acceleration =
        node->get_parameter_or<double>("urscript_descend_acceleration", 0.15);
    const double urscript_descend_velocity =
        node->get_parameter_or<double>("urscript_descend_velocity", 0.10);
    const double urscript_place_descend_velocity =
        node->get_parameter_or<double>("urscript_place_descend_velocity", 0.002);
    const bool urscript_place_wiggle_enabled =
        node->get_parameter_or<bool>("urscript_place_wiggle_enabled", true);
    const double urscript_place_wiggle_xy_amplitude =
        node->get_parameter_or<double>("urscript_place_wiggle_xy_amplitude", 0.0006);
    const double urscript_place_wiggle_z_amplitude =
        node->get_parameter_or<double>("urscript_place_wiggle_z_amplitude", 0.00025);
    const double urscript_place_wiggle_velocity =
        node->get_parameter_or<double>("urscript_place_wiggle_velocity", 0.012);
    const int urscript_place_wiggle_steps =
        node->get_parameter_or<int>("urscript_place_wiggle_steps", 12);
    const double urscript_disassemble_extra_lift =
        node->get_parameter_or<double>("urscript_disassemble_extra_lift", 0.0);
    const double urscript_disassemble_place_offset_x =
        node->get_parameter_or<double>("urscript_disassemble_place_offset_x", 0.0);
    const double urscript_disassemble_place_offset_y =
        node->get_parameter_or<double>("urscript_disassemble_place_offset_y", 0.0);
    const double urscript_disassemble_place_offset_z =
        node->get_parameter_or<double>("urscript_disassemble_place_offset_z", 0.0);
    const double urscript_place_lift_after_release =
        node->get_parameter_or<double>("urscript_place_lift_after_release", 0.0);
    const double urscript_place_slow_final_descend =
        node->get_parameter_or<double>("urscript_place_slow_final_descend", 0.020);
    const double urscript_place_settle_wait_seconds =
        node->get_parameter_or<double>("urscript_place_settle_wait_seconds", 0.5);
    const double urscript_pipeline_wait_seconds =
        node->get_parameter_or<double>("urscript_pipeline_wait_seconds", 50.0);
    const std::string task_file =
        node->get_parameter_or<std::string>("task_file", "");
    const double task_max_xy_offset =
        node->get_parameter_or<double>("task_max_xy_offset", 0.15);
    const double task_max_z_offset =
        node->get_parameter_or<double>("task_max_z_offset", 0.15);
    const double task_max_descend =
        node->get_parameter_or<double>("task_max_descend", 0.30);
    const std::string urscript_pregrasp_joints =
        node->get_parameter_or<std::string>("urscript_pregrasp_joints", "");
    const std::string urscript_preplace_joints =
        node->get_parameter_or<std::string>("urscript_preplace_joints", "");
    const std::string urscript_home_joints =
        node->get_parameter_or<std::string>("urscript_home_joints", "");
    const bool joint_ptp_return_home =
        node->get_parameter_or<bool>("joint_ptp_return_home", true);
    const double joint_ptp_dwell_seconds =
        node->get_parameter_or<double>("joint_ptp_dwell_seconds", 1.0);
    const double fixed_grasp_x =
        node->get_parameter_or<double>("fixed_grasp_x", 0.03);
    const double fixed_grasp_y =
        node->get_parameter_or<double>("fixed_grasp_y", 0.3529);
    const double fixed_grasp_z =
        node->get_parameter_or<double>("fixed_grasp_z", 0.90);
    const double fixed_grasp_hover =
        node->get_parameter_or<double>("fixed_grasp_hover", 0.15);
    const double fixed_place_x =
        node->get_parameter_or<double>("fixed_place_x", 0.20);
    const double fixed_place_y =
        node->get_parameter_or<double>("fixed_place_y", 0.3529);
    const double fixed_place_z =
        node->get_parameter_or<double>("fixed_place_z", 0.90);
    const double fixed_place_hover =
        node->get_parameter_or<double>("fixed_place_hover", 0.15);
    const double fixed_qx =
        node->get_parameter_or<double>("fixed_grasp_qx", -0.1779);
    const double fixed_qy =
        node->get_parameter_or<double>("fixed_grasp_qy", 0.7564);
    const double fixed_qz =
        node->get_parameter_or<double>("fixed_grasp_qz", 0.6179);
    const double fixed_qw =
        node->get_parameter_or<double>("fixed_grasp_qw", -0.1200);
    const bool use_current_pose_as_pregrasp =
        node->get_parameter_or<bool>("use_current_pose_as_pregrasp", false);
    const bool use_cartesian_descent =
        node->get_parameter_or<bool>("use_cartesian_descent", false);

    // 2. 使用多线程执行器以处理回调
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 3. 按需实例化 MoveGroup(MoveIt)。2026-07-02: URScript 抓放模式根本不用 MoveIt,
    //    而无条件构造 MoveGroupInterface 会去连 move_group / 订阅 /tf, 在 DDS 环境不佳时会刷
    //    "sequence size exceeds remaining buffer" 卡死、URScript 发不出去→臂不动。只在真正需要
    //    MoveIt 的模式(print_pose/print_joints/moveit 及 use_current_pose_as_pregrasp)才构造。
    const bool need_moveit = !(
        (motion_control_mode == "gripper_test" ||
         motion_control_mode == "joint_pick_place_tilt" ||
         motion_control_mode == "joint_ptp" ||
         motion_control_mode == "joint_pick_place" ||
         motion_control_mode == "urscript_ptp" ||
         motion_control_mode == "urscript" ||
         motion_control_mode == "onrobot_io" ||
         motion_control_mode == "disassemble_pick_place")
        && !use_current_pose_as_pregrasp);
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_ptr;
    [[maybe_unused]] std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> psi_ptr;
    if (need_moveit) {
        arm_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);
        psi_ptr = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
        if (!arm_ptr->setEndEffectorLink("tool0")) {
            RCLCPP_WARN(node->get_logger(), "无法将末端 link 设置为 tool0，将使用 MoveIt 默认末端 link。");
        }
        // 安全限制
        arm_ptr->setPlanningTime(15.0);
        arm_ptr->setMaxVelocityScalingFactor(0.15); // 15% 速度测试
        arm_ptr->setMaxAccelerationScalingFactor(0.1);

        RCLCPP_INFO(node->get_logger(), "等待 2 秒以同步参数服务器...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm_ptr->getPlanningFrame().c_str());
        RCLCPP_INFO(node->get_logger(), "End effector link: %s", arm_ptr->getEndEffectorLink().c_str());
    } else {
        RCLCPP_INFO(node->get_logger(),
            "URScript 模式(%s): 跳过 MoveIt 初始化(不连 move_group / 不订阅 /tf), 规避 DDS sequence-size 卡死。",
            motion_control_mode.c_str());
    }

    if (motion_control_mode == "print_joints") {
        const bool ok = printCurrentJointsForUrscript(node, *arm_ptr);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    if (motion_control_mode == "print_pose") {
        const bool ok = printCurrentPoseForCalibration(node, *arm_ptr);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    if (motion_control_mode == "gripper_test") {
        bool ok = false;
        if (gripper_control_mode == "onrobot_rg") {
            ok = executeOnRobotRgTest(
                node,
                gripper_urscript_topic,
                onrobot_rg_config,
                onrobot_rg_open_width_mm,
                onrobot_rg_close_width_mm,
                onrobot_rg_force_n,
                gripper_urscript_wait_seconds,
                18.0);
        } else {
            ok = executeRobotiqSocketTest(
                node,
                gripper_urscript_topic,
                gripper_urscript_wait_seconds,
                gripper_socket_speed,
                gripper_socket_force,
                15.0);
        }
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    // 斜抓(tilt)实验模式: 独立分派, 在 loadPickPlaceTasks 之前(tilt 用不同 yaml 格式)。
    if (motion_control_mode == "joint_pick_place_tilt") {
        const bool tilt_stop_at_pre =
            node->get_parameter_or<bool>("tilt_stop_at_pre", false);
        RCLCPP_INFO(node->get_logger(),
                    "斜抓(tilt)模式: 沿物体 z 轴接近+下探抓取(仅pick)。stop_at_pre=%s",
                    tilt_stop_at_pre ? "true" : "false");
        const bool ok = executeTiltPickPipeline(
            node,
            gripper_urscript_topic,
            gripper_control_mode,
            task_file,
            urscript_home_joints,
            gripper_socket_speed,
            gripper_socket_force,
            gripper_socket_open_wait_seconds,
            urscript_movej_acceleration,
            urscript_movej_velocity,
            urscript_movel_acceleration,
            urscript_movel_velocity,
            urscript_pipeline_wait_seconds,
            tilt_stop_at_pre);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    // 先关闭夹爪碰撞体，确认 UR5 本体的 5cm 位姿规划稳定后再逐步加回。
    // attachGripperCollision(arm, psi);

    RCLCPP_INFO(node->get_logger(), "开始固定前方抓取 pipeline...");
    RCLCPP_INFO(node->get_logger(), "Motion control mode: %s", motion_control_mode.c_str());
    if (motion_control_mode == "moveit" && gripper_control_mode == "urscript") {
        activateGripperUrscript(node, gripper_urscript_topic, gripper_urscript_wait_seconds);
    }

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.position.x = fixed_grasp_x;
    grasp_pose.position.y = fixed_grasp_y;
    grasp_pose.position.z = fixed_grasp_z;
    grasp_pose.orientation.x = fixed_qx;
    grasp_pose.orientation.y = fixed_qy;
    grasp_pose.orientation.z = fixed_qz;
    grasp_pose.orientation.w = fixed_qw;

    geometry_msgs::msg::Pose place_pose;
    place_pose.position.x = fixed_place_x;
    place_pose.position.y = fixed_place_y;
    place_pose.position.z = fixed_place_z;
    place_pose.orientation = grasp_pose.orientation;

    geometry_msgs::msg::Pose pregrasp_pose;
    if (use_current_pose_as_pregrasp) {
        pregrasp_pose = arm_ptr->getCurrentPose().pose;
        grasp_pose = pregrasp_pose;
        grasp_pose.position.z -= fixed_grasp_hover;
        RCLCPP_INFO(
            node->get_logger(),
            "使用当前 tool0 位姿作为 pregrasp: position=(%.4f, %.4f, %.4f), orientation=(%.4f, %.4f, %.4f, %.4f)",
            pregrasp_pose.position.x,
            pregrasp_pose.position.y,
            pregrasp_pose.position.z,
            pregrasp_pose.orientation.x,
            pregrasp_pose.orientation.y,
            pregrasp_pose.orientation.z,
            pregrasp_pose.orientation.w);
    } else {
        pregrasp_pose = grasp_pose;
        pregrasp_pose.position.z += fixed_grasp_hover;
    }

    geometry_msgs::msg::Pose preplace_pose = place_pose;
    preplace_pose.position.z += fixed_place_hover;

    std::vector<PickPlaceTask> pick_place_tasks;
    if (!loadPickPlaceTasks(
            node,
            task_file,
            fixed_grasp_hover,
            0.0,
            fixed_place_hover,
            urscript_place_slow_final_descend,
            task_max_xy_offset,
            task_max_z_offset,
            task_max_descend,
            pick_place_tasks)) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "已加载 %zu 个 pick/place 任务。", pick_place_tasks.size());

    if (motion_control_mode == "joint_ptp") {
        RCLCPP_INFO(
            node->get_logger(),
            "使用简单关节点到点模式：movej 到 point1/point2，然后回到 home。");
        const bool ok = executeUrscriptJointPtpPipeline(
            node,
            gripper_urscript_topic,
            urscript_pregrasp_joints,
            urscript_preplace_joints,
            urscript_home_joints,
            joint_ptp_return_home,
            joint_ptp_dwell_seconds,
            urscript_movej_acceleration,
            urscript_movej_velocity,
            urscript_pipeline_wait_seconds);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    if (motion_control_mode == "joint_pick_place" ||
        motion_control_mode == "urscript_ptp" ||
        motion_control_mode == "urscript") {
        RCLCPP_INFO(
            node->get_logger(),
            "使用 URScript 标准抓放：movej 到 pregrasp/preplace，movel 直线下降/抬起。");
        const bool ok = executeUrscriptPtpPipeline(
            node,
            gripper_urscript_topic,
            gripper_control_mode,
            pregrasp_pose,
            grasp_pose,
            preplace_pose,
            place_pose,
            urscript_pregrasp_joints,
            urscript_preplace_joints,
            urscript_home_joints,
            pick_place_tasks,
            joint_ptp_return_home,
            fixed_grasp_hover,
            fixed_place_hover,
            urscript_movej_acceleration,
            urscript_movej_velocity,
            urscript_pick_approach_movej_velocity,
            urscript_movel_acceleration,
            urscript_movel_velocity,
            urscript_pick_approach_movel_velocity,
            urscript_pick_lift_velocity,
            urscript_descend_acceleration,
            urscript_descend_velocity,
            urscript_place_descend_velocity,
            urscript_place_wiggle_enabled,
            urscript_place_wiggle_xy_amplitude,
            urscript_place_wiggle_z_amplitude,
            urscript_place_wiggle_velocity,
            urscript_place_wiggle_steps,
            urscript_place_lift_after_release,
            gripper_urscript_wait_seconds,
            gripper_socket_open_wait_seconds,
            gripper_socket_speed,
            gripper_socket_force,
            gripper_socket_release_position,
            gripper_socket_release_wait_seconds,
            onrobot_rg_config,
            onrobot_rg_open_width_mm,
            onrobot_rg_close_width_mm,
            onrobot_rg_pregrasp_width_mm,
            onrobot_rg_release_width_mm,
            onrobot_rg_force_n,
            urscript_place_settle_wait_seconds,
            urscript_pipeline_wait_seconds);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    if (motion_control_mode == "disassemble_pick_place") {
        RCLCPP_INFO(
            node->get_logger(),
            "使用拆卸抓放：到拔取姿态 -> 闭合夹爪 -> 抬起 -> 到丢放姿态 -> 慢速下降 -> 松开。");
        const bool ok = executeDisassemblePickPlacePipeline(
            node,
            gripper_urscript_topic,
            gripper_control_mode,
            urscript_pregrasp_joints,
            urscript_preplace_joints,
            urscript_home_joints,
            pick_place_tasks,
            joint_ptp_return_home,
            urscript_movej_acceleration,
            urscript_movej_velocity,
            urscript_movel_acceleration,
            urscript_movel_velocity,
            urscript_pick_lift_velocity,
            urscript_descend_acceleration,
            urscript_descend_velocity,
            urscript_place_descend_velocity,
            urscript_disassemble_extra_lift,
            urscript_disassemble_place_offset_x,
            urscript_disassemble_place_offset_y,
            urscript_disassemble_place_offset_z,
            gripper_urscript_wait_seconds,
            gripper_socket_open_wait_seconds,
            gripper_socket_speed,
            gripper_socket_force,
            gripper_socket_release_position,
            gripper_socket_release_wait_seconds,
            onrobot_rg_config,
            onrobot_rg_open_width_mm,
            onrobot_rg_close_width_mm,
            onrobot_rg_release_width_mm,
            onrobot_rg_force_n,
            urscript_place_settle_wait_seconds,
            urscript_pipeline_wait_seconds);
        rclcpp::shutdown();
        spin_thread.join();
        return ok ? 0 : 1;
    }

    if (motion_control_mode != "moveit") {
        RCLCPP_ERROR(
            node->get_logger(),
            "未知 motion_control_mode: %s。请使用 print_joints、print_pose、gripper_test、joint_ptp、joint_pick_place、disassemble_pick_place、urscript_ptp 或 moveit。",
            motion_control_mode.c_str());
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    if (use_current_pose_as_pregrasp) {
        RCLCPP_INFO(node->get_logger(), "1. 当前已经在 pregrasp，跳过全局 pose 规划。");
    } else {
        if (!moveToPose(node, *arm_ptr, pregrasp_pose, "1. 移动到 pregrasp")) {
            rclcpp::shutdown();
            spin_thread.join();
            return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(), "先打开夹爪...");
    driveGripper(
        node,
        gripper_control_mode,
        gripper_action_name,
        gripper_io_service_name,
        gripper_urscript_topic,
        onrobot_rg_config,
        gripper_io_open_pin,
        gripper_io_close_pin,
        gripper_open_position,
        gripper_close_position,
        gripper_open_position,
        onrobot_rg_open_width_mm,
        onrobot_rg_close_width_mm,
        onrobot_rg_force_n,
        gripper_max_effort,
        gripper_io_pulse_seconds,
        gripper_io_latching,
        gripper_urscript_wait_seconds);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const bool descent_ok = use_cartesian_descent ?
        moveLinearZ(node, *arm_ptr, -fixed_grasp_hover, "2. 从 pregrasp 垂直下降到 grasp") :
        moveOffsetPose(node, *arm_ptr, 0.0, 0.0, -fixed_grasp_hover, "2. 从 pregrasp 下降到 grasp");
    if (!descent_ok) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "闭合夹爪...");
    driveGripper(
        node,
        gripper_control_mode,
        gripper_action_name,
        gripper_io_service_name,
        gripper_urscript_topic,
        onrobot_rg_config,
        gripper_io_open_pin,
        gripper_io_close_pin,
        gripper_open_position,
        gripper_close_position,
        gripper_close_position,
        onrobot_rg_open_width_mm,
        onrobot_rg_close_width_mm,
        onrobot_rg_force_n,
        gripper_max_effort,
        gripper_io_pulse_seconds,
        gripper_io_latching,
        gripper_urscript_wait_seconds);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    const bool lift_ok = use_cartesian_descent ?
        moveLinearZ(node, *arm_ptr, fixed_grasp_hover, "3. 抓取后垂直抬起") :
        moveOffsetPose(node, *arm_ptr, 0.0, 0.0, fixed_grasp_hover, "3. 抓取后抬起");
    if (!lift_ok) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!moveToPose(node, *arm_ptr, preplace_pose, "4. 移动到 preplace")) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const bool place_down_ok = use_cartesian_descent ?
        moveLinearZ(node, *arm_ptr, -fixed_place_hover, "5. 从 preplace 垂直下降到 place") :
        moveOffsetPose(node, *arm_ptr, 0.0, 0.0, -fixed_place_hover, "5. 从 preplace 下降到 place");
    if (!place_down_ok) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "在 place 位置打开夹爪...");
    driveGripper(
        node,
        gripper_control_mode,
        gripper_action_name,
        gripper_io_service_name,
        gripper_urscript_topic,
        onrobot_rg_config,
        gripper_io_open_pin,
        gripper_io_close_pin,
        gripper_open_position,
        gripper_close_position,
        gripper_open_position,
        onrobot_rg_open_width_mm,
        onrobot_rg_close_width_mm,
        onrobot_rg_force_n,
        gripper_max_effort,
        gripper_io_pulse_seconds,
        gripper_io_latching,
        gripper_urscript_wait_seconds);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const bool place_up_ok = use_cartesian_descent ?
        moveLinearZ(node, *arm_ptr, fixed_place_hover, "6. 放置后垂直抬回 preplace") :
        moveOffsetPose(node, *arm_ptr, 0.0, 0.0, fixed_place_hover, "6. 放置后抬回 preplace");
    if (!place_up_ok) {
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "固定前方抓放 pipeline 完成。");

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
