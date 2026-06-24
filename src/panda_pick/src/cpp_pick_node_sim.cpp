#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <vector>
#include <thread>
#include <fstream>
#include <sstream>
#include <regex>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.hpp>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using namespace std::chrono;

// ================= 配置 =================
const std::string SCENE_XML_FILE = "/home/i6user/Desktop/robot_lego/src/mj_bridge/mj_bridge/scene.xml";
const std::string PLAN_FILE      = "/home/i6user/Desktop/robot_lego/src/panda_pick/src/plan.yaml";
const std::string MESH_PATH      = "/home/i6user/Desktop/robot_lego/FoundationPose/meshes/";

const double ASSEMBLY_ORIGIN_X = 0.35;
const double ASSEMBLY_ORIGIN_Y = 0.35;
const double ASSEMBLY_ORIGIN_Z = 0.046; //change 0.04 to 0.046 because of baseplate

const double GRIPPER_OFFSET = 0.1234;
const double HOVER = 0.15;

const double GRIPPER_OPEN = 0.04;
const double GRIPPER_CLOSE = 0.001;

// ================= 评测数据结构 =================
struct EvalMetrics {
    bool success = true;
    double compute_time = 0.0;
    double manip_time = 0.0;
    double max_accuracy_error = 0.0;
    int collision_count = 0;
    int stability_violations = 0;
};

// ================= 数据结构 =================
struct SceneBrick {
    std::string body_name;
    std::string brick_type;
    geometry_msgs::msg::Pose scene_pose;
};

struct PlanStep {
    std::string raw_name;
    std::string brick_type;
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose place_pose;
};

struct Task {
    std::string name;
    std::string mesh_file;
    geometry_msgs::msg::Pose brick_pose;
    geometry_msgs::msg::Pose gripper_pick;
    geometry_msgs::msg::Pose place_pose;
};

// ================= 工具函数 =================
std::vector<double> parse_numbers(const std::string& s) {
    std::vector<double> nums;
    std::stringstream ss(s);
    double v;
    while (ss >> v) nums.push_back(v);
    return nums;
}

geometry_msgs::msg::Quaternion multiply_quat(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2
) {
    geometry_msgs::msg::Quaternion q;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return q;
}

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw_rad) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_rad * 0.5);
    q.w = std::cos(yaw_rad * 0.5);
    return q;
}

std::string infer_brick_type_from_name(const std::string& name) {
    if (name.find("2x2") != std::string::npos) return "brick_2x2";
    if (name.find("4x2") != std::string::npos || name.find("2x4") != std::string::npos) return "brick_4x2";
    return "";
}

std::string infer_mesh_from_type(const std::string& type) {
    return (type == "brick_4x2") ? "LEGO_Duplo_brick_4x2.stl" : "LEGO_Duplo_brick_2x2.stl";
}

geometry_msgs::msg::Quaternion quat_wxyz_to_xyzw(const std::vector<double>& q) {
    geometry_msgs::msg::Quaternion out;
    if (q.size() != 4) {
        out.x = 0.0;
        out.y = 0.0;
        out.z = 0.0;
        out.w = 1.0;
        return out;
    }

    out.x = q[1];
    out.y = q[2];
    out.z = q[3];
    out.w = q[0];
    return out;
}

geometry_msgs::msg::Quaternion quat_xyzw_from_yaml(const YAML::Node& node) {
    geometry_msgs::msg::Quaternion q;
    q.x = node[0].as<double>();
    q.y = node[1].as<double>();
    q.z = node[2].as<double>();
    q.w = node[3].as<double>();
    return q;
}

shape_msgs::msg::Mesh load_stl_mesh(const std::string& file_path) {
    std::string uri = (file_path.substr(0, 7) != "file://")
        ? "file://" + file_path
        : file_path;

    shapes::Mesh* m = shapes::createMeshFromResource(uri);
    if (m == nullptr) {
        throw std::runtime_error("无法加载 STL: " + uri);
    }

    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(m, mesh_msg);
    shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    for (auto& vertex : mesh.vertices) {
        vertex.x *= 0.001;
        vertex.y *= 0.001;
        vertex.z *= 0.001;
    }

    delete m;
    return mesh;
}

// ================= 读取 scene.xml =================
std::vector<SceneBrick> read_scene_bricks_from_xml(const std::string& xml_file) {
    std::ifstream ifs(xml_file);
    if (!ifs.is_open()) {
        throw std::runtime_error("无法打开 scene.xml: " + xml_file);
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string content = buffer.str();

    std::regex body_regex(
        R"xxx(<body\s+name="([^"]*brick[^"]*)"\s+pos="([^"]+)"\s+quat="([^"]+)")xxx"
    );

    std::vector<SceneBrick> bricks;

    auto begin = std::sregex_iterator(content.begin(), content.end(), body_regex);
    auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it) {
        std::smatch m = *it;

        SceneBrick b;
        b.body_name = m[1].str();
        b.brick_type = infer_brick_type_from_name(b.body_name);

        auto pos = parse_numbers(m[2].str());
        auto quat = parse_numbers(m[3].str());

        if (pos.size() != 3) continue;

        b.scene_pose.position.x = pos[0];
        b.scene_pose.position.y = pos[1];
        b.scene_pose.position.z = pos[2];
        b.scene_pose.orientation = quat_wxyz_to_xyzw(quat);

        bricks.push_back(b);
    }

    return bricks;
}

// ================= 读取 plan.yaml =================
std::vector<PlanStep> read_plan_steps(const std::string& plan_file) {
    YAML::Node root = YAML::LoadFile(plan_file);
    std::vector<PlanStep> steps;

    YAML::Node task_nodes;

    if (root["tasks"]) {
        task_nodes = root["tasks"];
    } else if (root["tasksh"]) {
        task_nodes = root["tasksh"];
    } else {
        throw std::runtime_error("plan.yaml 中找不到 tasks 或 tasksh");
    }

    for (const auto& n : task_nodes) {
        PlanStep s;

        s.raw_name = n["name"].as<std::string>();
        s.brick_type = infer_brick_type_from_name(s.raw_name);

        if (s.brick_type.empty()) {
            throw std::runtime_error("无法从任务名推断 brick type: " + s.raw_name);
        }

        s.pick_pose.orientation = quat_xyzw_from_yaml(n["pick"]["orientation"]);

        const auto place_pos = n["place"]["pos"];

        s.place_pose.position.x = ASSEMBLY_ORIGIN_X + place_pos[0].as<double>();
        s.place_pose.position.y = ASSEMBLY_ORIGIN_Y + place_pos[1].as<double>();
        s.place_pose.position.z = ASSEMBLY_ORIGIN_Z + place_pos[2].as<double>() + 0.0095;

        std::cout << "[PLAN_PLACE] "
          << s.raw_name
          << " place_world = "
          << s.place_pose.position.x << ", "
          << s.place_pose.position.y << ", "
          << s.place_pose.position.z
          << std::endl;

        s.place_pose.orientation = quat_xyzw_from_yaml(n["place"]["orientation"]);

        steps.push_back(s);
    }

    return steps;
}

// ================= scene + plan 匹配 =================
std::vector<Task> build_tasks_from_scene_and_plan(
    const std::vector<SceneBrick>& scene_bricks,
    const std::vector<PlanStep>& plan_steps
) {
    std::vector<Task> tasks;
    std::unordered_set<std::string> used_scene_bodies;

    for (const auto& step : plan_steps) {
        const SceneBrick* chosen = nullptr;

        for (const auto& b : scene_bricks) {
            if (used_scene_bodies.count(b.body_name)) continue;

            if (b.brick_type == step.brick_type) {
                chosen = &b;
                break;
            }
        }

        if (!chosen) {
            throw std::runtime_error("匹配失败: " + step.brick_type);
        }

        Task t;
        t.name = chosen->body_name;
        t.mesh_file = infer_mesh_from_type(chosen->brick_type);
        t.brick_pose = chosen->scene_pose;

        t.gripper_pick.position = chosen->scene_pose.position;

        auto q_offset = yaw_to_quat(-M_PI / 4.0);
        t.gripper_pick.orientation = multiply_quat(step.pick_pose.orientation, q_offset);

        t.place_pose = step.place_pose;
        t.place_pose.orientation = multiply_quat(step.place_pose.orientation, q_offset);

        tasks.push_back(t);
        used_scene_bodies.insert(chosen->body_name);
    }

    return tasks;
}

// ================= 夹爪动作：等待 action 完成 =================
bool driveGripperAction(rclcpp::Node::SharedPtr node, double pos) {
    auto client = rclcpp_action::create_client<FollowJT>(
        node,
        "/mj_panda_hand_controller/follow_joint_trajectory"
    );

    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Hand action server not available");
        return false;
    }

    FollowJT::Goal goal;
    goal.trajectory.joint_names = {
        "panda_finger_joint1",
        "panda_finger_joint2"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {pos, pos};
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    goal.trajectory.points.push_back(point);

    auto goal_handle_future = client->async_send_goal(goal);

    if (goal_handle_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send hand goal");
        return false;
    }

    auto goal_handle = goal_handle_future.get();

    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Hand goal was rejected");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);

    if (result_future.wait_for(std::chrono::seconds(4)) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(), "Hand action result timeout");
        return false;
    }

    auto wrapped_result = result_future.get();

    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node->get_logger(), "Hand action failed");
        return false;
    }

    return true;
}

// ================= 垂直直线运动 =================
bool move_linear(
    moveit::planning_interface::MoveGroupInterface& arm,
    double z_delta,
    double speed_scale = 0.1
) {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose target = arm.getCurrentPose().pose;
    target.position.z += z_delta;
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;

    double fraction = arm.computeCartesianPath(
        waypoints,
        0.005,
        0.0,
        trajectory_msg,
        false
    );

    if (fraction < 0.95) {
        RCLCPP_WARN(
            rclcpp::get_logger("lego_batch_executor"),
            "Cartesian path fraction too low: %.3f",
            fraction
        );
        return false;
    }

    robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;

    if (!totg.computeTimeStamps(rt, speed_scale, speed_scale)) {
        RCLCPP_WARN(
            rclcpp::get_logger("lego_batch_executor"),
            "Time parameterization failed"
        );
        return false;
    }

    rt.getRobotTrajectoryMsg(trajectory_msg);

    return arm.execute(trajectory_msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

// ================= 允许碰撞 =================

void add_table_collision(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& frame_id
) {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = frame_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;

    // MuJoCo table:
    // <body name="table" pos="0.40 0 0.02">
    //   <geom type="box" size="0.30 0.50 0.02"/>
    // </body>
    //
    // MuJoCo size 是半尺寸
    // 所以真实尺寸是 0.60 x 1.00 x 0.04
    primitive.dimensions = {0.60, 1.00, 0.04};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.40;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.02;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    psi.applyCollisionObject(table);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void add_brick_collision_at_pose(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& frame_id,
    const std::string& brick_id,
    const std::string& mesh_file,
    const geometry_msgs::msg::Pose& pose
) {
    moveit_msgs::msg::CollisionObject brick;
    brick.id = brick_id;
    brick.header.frame_id = frame_id;

    brick.meshes.push_back(load_stl_mesh(MESH_PATH + mesh_file));
    brick.mesh_poses.push_back(pose);
    brick.operation = brick.ADD;

    psi.applyCollisionObject(brick);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

// ================= 单个任务执行 =================
bool execute_single_task(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface& arm,
    moveit::planning_interface::PlanningSceneInterface& psi,
    const Task& task,
    EvalMetrics& metrics
) {
    auto m_start = high_resolution_clock::now();


    
    // 1. 添加物体到 MoveIt planning scene
    //moveit_msgs::msg::CollisionObject brick;
    //brick.id = task.name;
    //brick.header.frame_id = arm.getPlanningFrame();
    //brick.meshes.push_back(load_stl_mesh(MESH_PATH + task.mesh_file));
    //brick.mesh_poses.push_back(task.brick_pose);
    //brick.operation = brick.ADD;

    //psi.applyCollisionObject(brick);
    //allow_all_collisions(node, task.name);
    add_brick_collision_at_pose(
        psi,
        arm.getPlanningFrame(),
        task.name,
        task.mesh_file,
        task.brick_pose
    );

    //allow_gripper_touch_object(node, task.name);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. 移动到抓取 hover 位姿
    geometry_msgs::msg::Pose p_hover = task.gripper_pick;
    p_hover.position.z += (GRIPPER_OFFSET + HOVER);

    arm.setPoseTarget(p_hover);

    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        metrics.collision_count++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 3. 先张开夹爪，并等待完成
    if (!driveGripperAction(node, GRIPPER_OPEN)) {
        metrics.collision_count++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 4. 垂直下降，必须完整下降成功
    if (!move_linear(arm, -0.165, 0.1)) {
        metrics.stability_violations++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 5. 闭合夹爪，并等待 close_hold action 返回
    if (!driveGripperAction(node, GRIPPER_CLOSE)) {
        metrics.collision_count++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 6. MoveIt attach object
    // arm.attachObject(task.name, "panda_hand");
    std::vector<std::string> touch_links = {
        "panda_hand",
        "panda_leftfinger",
        "panda_rightfinger",
        "panda_link8"
    };

    arm.attachObject(task.name, "panda_hand", touch_links);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 7. 抬起
    if (!move_linear(arm, HOVER, 0.5)) {
        metrics.stability_violations++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 8. 移动到放置 hover 位姿
    geometry_msgs::msg::Pose p_place = task.place_pose;
    p_place.position.z += (GRIPPER_OFFSET + HOVER);

    arm.setPoseTarget(p_place);

    if (arm.move() != moveit::core::MoveItErrorCode::SUCCESS) {
        metrics.collision_count++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 9. 放置下降
    if (!move_linear(arm, -HOVER, 0.05)) {
        metrics.stability_violations++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 10. 计算执行精度
    auto curr_pose = arm.getCurrentPose().pose;

    double err = std::sqrt(
        std::pow(curr_pose.position.x - task.place_pose.position.x, 2) +
        std::pow(curr_pose.position.y - task.place_pose.position.y, 2)
    );

    metrics.max_accuracy_error = std::max(metrics.max_accuracy_error, err);

    // 11. detach + 张开
    arm.detachObject(task.name);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 放置完成后，把这块积木重新加入 MoveIt planning scene。
    // 位置使用 task.place_pose。
    // 后续任务会把它当作障碍物避开。
    add_brick_collision_at_pose(
        psi,
        arm.getPlanningFrame(),
        task.name,
        task.mesh_file,
        task.place_pose
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    if (!driveGripperAction(node, GRIPPER_OPEN)) {
        metrics.collision_count++;
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 12. 抬起
    if (!move_linear(arm, HOVER, 0.5)) {
        metrics.stability_violations++;
        return false;
    }

    auto m_end = high_resolution_clock::now();
    metrics.manip_time += duration<double>(m_end - m_start).count();

    return true;
}

// ================= 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared(
        "lego_batch_executor",
        node_options
    );

    EvalMetrics final_metrics;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinning_thread([&executor]() {
        executor.spin();
    });

    try {
        moveit::planning_interface::MoveGroupInterface arm(node, "panda_arm");
        moveit::planning_interface::PlanningSceneInterface psi;
        add_table_collision(psi, arm.getPlanningFrame());


        arm.setPlanningTime(10.0);
        arm.setMaxVelocityScalingFactor(1.0);
        arm.setMaxAccelerationScalingFactor(1.0);

        auto c_start = high_resolution_clock::now();

        auto scene_bricks = read_scene_bricks_from_xml(SCENE_XML_FILE);
        auto plan_steps = read_plan_steps(PLAN_FILE);
        auto tasks = build_tasks_from_scene_and_plan(scene_bricks, plan_steps);

        auto c_end = high_resolution_clock::now();
        final_metrics.compute_time = duration<double>(c_end - c_start).count();

        for (size_t i = 0; i < tasks.size(); ++i) {
            RCLCPP_INFO(
                node->get_logger(),
                "Executing task %zu / %zu: %s",
                i + 1,
                tasks.size(),
                tasks[i].name.c_str()
            );

            if (!execute_single_task(node, arm, psi, tasks[i], final_metrics)) {
                final_metrics.success = false;
                break;
            }
        }

        std::cout << "\n[METRICS_START]" << std::endl;
        std::cout << "success: " << (final_metrics.success ? 1 : 0) << std::endl;
        std::cout << "compute_time: " << final_metrics.compute_time << std::endl;
        std::cout << "manip_time: " << final_metrics.manip_time << std::endl;
        std::cout << "accuracy_error: " << final_metrics.max_accuracy_error << std::endl;
        std::cout << "stability_violation: " << (final_metrics.stability_violations > 0 ? 1 : 0) << std::endl;
        std::cout << "collision: " << (final_metrics.collision_count > 0 ? 1 : 0) << std::endl;
        std::cout << "[METRICS_END]" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;

        std::cout << "\n[METRICS_START]" << std::endl;
        std::cout << "success: 0" << std::endl;
        std::cout << "compute_time: 0" << std::endl;
        std::cout << "manip_time: 0" << std::endl;
        std::cout << "accuracy_error: 0" << std::endl;
        std::cout << "stability_violation: 0" << std::endl;
        std::cout << "collision: 1" << std::endl;
        std::cout << "[METRICS_END]" << std::endl;
    }

    rclcpp::shutdown();

    if (spinning_thread.joinable()) {
        spinning_thread.join();
    }

    return 0;
}