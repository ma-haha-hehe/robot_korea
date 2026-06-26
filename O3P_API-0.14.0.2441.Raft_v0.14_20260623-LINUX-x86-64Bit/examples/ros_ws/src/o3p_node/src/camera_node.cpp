/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include <o3p/Align.hpp>
#include <o3p/o3p.hpp>

#ifdef ROS1
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
typedef ros::Publisher FramePublisher;
typedef ros::Publisher ImuPublisher;
typedef ros::Publisher PointCloudPublisher;
typedef ros::Publisher CameraInfoPublisher;
#else
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

typedef rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr FramePublisher;
typedef rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuPublisher;
typedef rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloudPublisher;
typedef rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr CameraInfoPublisher;
#endif

namespace {

// Rotation that maps optical-frame coordinates (X right, Y down, Z forward) to the
// parent body frame (X forward, Y left, Z up), row-major. This is the rotation of
// the fixed body->optical static transform, quaternion (x, y, z, w) =
// (-0.5, 0.5, -0.5, 0.5). Used to express the inter-sensor extrinsic (which the
// device reports between optical frames) as a transform between body frames.
constexpr float kOpticalToBody[9] = {0.f, 0.f, 1.f, -1.f, 0.f, 0.f, 0.f, -1.f, 0.f};

// Mechanical offset from the depth (ToF) sensor optical centre to the centre of
// the camera's rear/back plate, taken from the CAD assembly of the camera
// (Creo model 000_CM23YD501_ASM, STEP export 8A-23YD501-D01-...-VD03). In the CAD
// frame the origin coincides with the IRS2875 ToF image-area centre and +Z points
// out of the lens (so CAD axes match the optical convention X right, Y down,
// Z forward). Measured component centres (mm):
//   ToF image area (IMAGE_AREA_IRS2875): (0.000, 0.000,  1.383)
//   back plate outer rear face centre  : (11.200, 0.000, -17.093)
// giving a depth->back-plate offset of (11.200, 0.000, -18.476) mm in optical
// axes. Expressed in the body camera_frame (X forward, Y left, Z up), where
// forward = optical_Z, left = -optical_X, up = -optical_Y, that becomes the
// translation below (in metres). The RGB sensor sits at CAD (-16.559, 0.110,
// 0.398) mm, i.e. on the opposite side of the depth sensor from the back-plate
// centre; flip the sign of the second (left) component if your unit's optical
// X axis is mirrored relative to this CAD model.
constexpr double kBackPlateOffsetBody[3] = {-0.018476, -0.011200, 0.0};

// C = A * B for row-major 3x3 matrices.
inline void mat3Mul(const float a[9], const float b[9], float c[9]) {
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
            c[r * 3 + col] = a[r * 3 + 0] * b[0 * 3 + col] +
                             a[r * 3 + 1] * b[1 * 3 + col] +
                             a[r * 3 + 2] * b[2 * 3 + col];
        }
    }
}

// o = A * v for a row-major 3x3 matrix and a 3-vector.
inline void mat3Vec(const float a[9], const float v[3], float o[3]) {
    for (int r = 0; r < 3; ++r) {
        o[r] = a[r * 3 + 0] * v[0] + a[r * 3 + 1] * v[1] + a[r * 3 + 2] * v[2];
    }
}

// Convert a row-major 3x3 rotation matrix to a quaternion (x, y, z, w).
inline void quatFromMat(const float r[9], double q[4]) {
    const double m00 = r[0], m01 = r[1], m02 = r[2];
    const double m10 = r[3], m11 = r[4], m12 = r[5];
    const double m20 = r[6], m21 = r[7], m22 = r[8];
    const double trace = m00 + m11 + m22;
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0; // s = 4*w
        q[3] = 0.25 * s;
        q[0] = (m21 - m12) / s;
        q[1] = (m02 - m20) / s;
        q[2] = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0; // s = 4*x
        q[3] = (m21 - m12) / s;
        q[0] = 0.25 * s;
        q[1] = (m01 + m10) / s;
        q[2] = (m02 + m20) / s;
    } else if (m11 > m22) {
        double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0; // s = 4*y
        q[3] = (m02 - m20) / s;
        q[0] = (m01 + m10) / s;
        q[1] = 0.25 * s;
        q[2] = (m12 + m21) / s;
    } else {
        double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0; // s = 4*z
        q[3] = (m10 - m01) / s;
        q[0] = (m02 + m20) / s;
        q[1] = (m12 + m21) / s;
        q[2] = 0.25 * s;
    }
}

} // namespace

class CameraNode {
  public:
    CameraNode() {
#ifdef ROS1
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        // Serial number of the device to open. Empty selects the first available camera.
        pnh.param<std::string>("serial_no", serial_no, "");
        // Topic/frame prefix, allows running several nodes for several cameras.
        pnh.param<std::string>("camera_name", camera_name, "camera");
        // Path to a recording (.bag) to play back instead of opening a live camera. Empty uses a live camera.
        pnh.param<std::string>("recording", recording, "");
        // Whether to loop the playback once the end of the recording is reached.
        pnh.param<bool>("loop", loop_playback, true);
        // Whether to delay frame delivery according to the recorded timestamps.
        pnh.param<bool>("use_timestamps", use_timestamps, true);
        // If the device is not found at startup, keep retrying for this many seconds.
        // A negative value (default) means retry indefinitely. 0 means exit immediately.
        pnh.param<double>("wait_for_device_timeout", wait_for_device_timeout, -1.0);
        // After a USB disconnect, wait this many seconds before reconnecting.
        // A negative value (default) means keep trying indefinitely. 0 means no reconnect.
        pnh.param<double>("reconnect_timeout", reconnect_timeout, -1.0);
        camera_frame = camera_name + "_frame";
        // Camera image/point-cloud data is expressed in the optical convention
        // (X right, Y down, Z forward) as required by ROS REP-103, in per-sensor
        // optical frames linked to body frames by static transforms :
        //   camera_frame (link, X fwd/Y left/Z up)
        //   |-- camera_depth_frame -- camera_depth_optical_frame
        //   |-- camera_color_frame -- camera_color_optical_frame
        //   `-- camera_back_plate (centre of the rear plate, from CAD)
        // The depth->color offset comes from the device ToF-to-RGB extrinsics.
        camera_depth_frame = camera_name + "_depth_frame";
        camera_depth_optical_frame = camera_name + "_depth_optical_frame";
        camera_color_frame = camera_name + "_color_frame";
        camera_color_optical_frame = camera_name + "_color_optical_frame";
        camera_back_plate_frame = camera_name + "_back_plate";
        imu_frame = camera_name + "_imu";

        pc_publisher = nh.advertise<sensor_msgs::PointCloud2>(camera_name + "/frame_pc", 10);
        depth_publisher = nh.advertise<sensor_msgs::Image>(camera_name + "/frame_depth", 10);
        depth_raw_publisher = nh.advertise<sensor_msgs::Image>(camera_name + "/depth/image_raw", 10);
        depth_camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(camera_name + "/depth/camera_info", 10);
        aligned_depth_publisher = nh.advertise<sensor_msgs::Image>(camera_name + "/aligned_depth_to_color/image_raw", 10);
        aligned_depth_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(camera_name + "/aligned_depth_to_color/camera_info", 10);
        ir_publisher = nh.advertise<sensor_msgs::Image>(camera_name + "/frame_ir", 10);
        rgb_publisher = nh.advertise<sensor_msgs::Image>(camera_name + "/frame_rgb", 10);
        camera_info_publisher = nh.advertise<sensor_msgs::CameraInfo>(camera_name + "/camera_info", 10);
        metadata_publisher = nh.advertise<std_msgs::String>(camera_name + "/metadata", 10);

        accel_publisher = nh.advertise<sensor_msgs::Imu>(camera_name + "/imu/accel", 10);
        gyro_publisher = nh.advertise<sensor_msgs::Imu>(camera_name + "/imu/gyro", 10);

        // Serial number is published on a latched topic so subscribers that connect
        // later (e.g. the rviz panel) still receive it. The /diagnostics topic carries
        // the serial number as the hardware_id.
        serial_publisher = nh.advertise<std_msgs::String>(camera_name + "/serial_number", 1, true);
        diagnostics_publisher = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

        enable_ai_service = nh.advertiseService(camera_name + "/enable_ai", &CameraNode::enableAICallback, this);

        // Service that returns the device serial number (and other device info)
        device_info_service = nh.advertiseService(camera_name + "/device_info", &CameraNode::deviceInfoCallback, this);

        // Hardware reset: stops, re-initialises and restarts the pipeline
        hw_reset_service = nh.advertiseService(camera_name + "/hw_reset", &CameraNode::hwResetCallback, this);

        // Camera use case parameters (preset, fps, hdr, range) are published on a
        // latched topic so the rviz panel always receives the current state and the
        // list of available options. Commands to change them arrive on set_parameters.
        parameters_publisher = nh.advertise<std_msgs::String>(camera_name + "/parameters", 1, true);
        set_parameters_subscriber =
            nh.subscribe(camera_name + "/set_parameters", 10, &CameraNode::setParametersCallback, this);

        // Generic extended-option (config) access. Requests arrive on set_config as
        // "get=<key>" or "set=<key>=<value>"; the resulting value is published on the
        // latched config_result topic as "<key>=<value>".
        config_result_publisher = nh.advertise<std_msgs::String>(camera_name + "/config_result", 1, true);
        set_config_subscriber =
            nh.subscribe(camera_name + "/set_config", 10, &CameraNode::setConfigCallback, this);

#else
        node = std::make_shared<rclcpp::Node>("o3p_node");

        // Serial number of the device to open. Empty selects the first available camera.
        serial_no = node->declare_parameter<std::string>("serial_no", "");
        // Topic/frame prefix, allows running several nodes for several cameras.
        camera_name = node->declare_parameter<std::string>("camera_name", "camera");
        // Path to a recording (.bag) to play back instead of opening a live camera. Empty uses a live camera.
        recording = node->declare_parameter<std::string>("recording", "");
        // Whether to loop the playback once the end of the recording is reached.
        loop_playback = node->declare_parameter<bool>("loop", true);
        // Whether to delay frame delivery according to the recorded timestamps.
        use_timestamps = node->declare_parameter<bool>("use_timestamps", true);
        // If the device is not found at startup, keep retrying for this many seconds.
        // A negative value (default) means retry indefinitely. 0 means exit immediately.
        wait_for_device_timeout = node->declare_parameter<double>("wait_for_device_timeout", -1.0);
        // After a USB disconnect, wait this many seconds before reconnecting.
        // A negative value (default) means keep trying indefinitely. 0 means no reconnect.
        reconnect_timeout = node->declare_parameter<double>("reconnect_timeout", -1.0);
        camera_frame = camera_name + "_frame";
        // Camera image/point-cloud data is expressed in the optical convention
        // (X right, Y down, Z forward) as required by ROS REP-103, in per-sensor
        // optical frames linked to body frames by static transforms:
        //   camera_frame (link, X fwd/Y left/Z up)
        //   |-- camera_depth_frame -- camera_depth_optical_frame
        //   |-- camera_color_frame -- camera_color_optical_frame
        //   `-- camera_back_plate (centre of the rear plate, from CAD)
        // The depth->color offset comes from the device ToF-to-RGB extrinsics and is
        // published once the extrinsics have been read (see broadcastStaticTransforms).
        camera_depth_frame = camera_name + "_depth_frame";
        camera_depth_optical_frame = camera_name + "_depth_optical_frame";
        camera_color_frame = camera_name + "_color_frame";
        camera_color_optical_frame = camera_name + "_color_optical_frame";
        camera_back_plate_frame = camera_name + "_back_plate";
        imu_frame = camera_name + "_imu";

        // The TF tree is broadcast from broadcastStaticTransforms() after the device
        // extrinsics have been read.
        static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        pc_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name + "/frame_pc", 10);
        depth_publisher = node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/frame_depth", 10);
        depth_raw_publisher = node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/depth/image_raw", 10);
        depth_camera_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/depth/camera_info", 10);
        aligned_depth_publisher = node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/aligned_depth_to_color/image_raw", 10);
        aligned_depth_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/aligned_depth_to_color/camera_info", 10);
        ir_publisher = node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/frame_ir", 10);
        rgb_publisher = node->create_publisher<sensor_msgs::msg::Image>(camera_name + "/frame_rgb", 10);
        camera_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name + "/camera_info", 10);
        metadata_publisher = node->create_publisher<std_msgs::msg::String>(camera_name + "/metadata", 10);

        accel_publisher = node->create_publisher<sensor_msgs::msg::Imu>(camera_name + "/imu/accel", 10);
        gyro_publisher = node->create_publisher<sensor_msgs::msg::Imu>(camera_name + "/imu/gyro", 10);

        // Serial number is published on a latched (transient_local) topic so subscribers
        // that connect later (e.g. the rviz panel) still receive it. The /diagnostics
        // topic carries the serial number as the hardware_id.
        serial_publisher = node->create_publisher<std_msgs::msg::String>(
            camera_name + "/serial_number", rclcpp::QoS(1).transient_local());
        diagnostics_publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        enable_ai_service = node->create_service<std_srvs::srv::SetBool>(
            camera_name + "/enable_ai",
            std::bind(&CameraNode::enableAICallback, this, std::placeholders::_1, std::placeholders::_2));

        // Service that returns the device serial number (and other device info)
        device_info_service = node->create_service<std_srvs::srv::Trigger>(
            camera_name + "/device_info",
            std::bind(&CameraNode::deviceInfoCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Hardware reset: stops, re-initialises and restarts the pipeline
        hw_reset_service = node->create_service<std_srvs::srv::Empty>(
            camera_name + "/hw_reset",
            std::bind(&CameraNode::hwResetCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Camera use case parameters (preset, fps, hdr, range) are published on a
        // latched (transient_local) topic so the rviz panel always receives the current
        // state and the list of available options. Commands to change them arrive on
        // set_parameters.
        parameters_publisher = node->create_publisher<std_msgs::msg::String>(
            camera_name + "/parameters", rclcpp::QoS(1).transient_local());
        set_parameters_subscriber = node->create_subscription<std_msgs::msg::String>(
            camera_name + "/set_parameters", 10,
            std::bind(&CameraNode::setParametersCallback, this, std::placeholders::_1));

        // Generic extended-option (config) access. Requests arrive on set_config as
        // "get=<key>" or "set=<key>=<value>"; the resulting value is published on the
        // latched (transient_local) config_result topic as "<key>=<value>".
        config_result_publisher = node->create_publisher<std_msgs::msg::String>(
            camera_name + "/config_result", rclcpp::QoS(1).transient_local());
        set_config_subscriber = node->create_subscription<std_msgs::msg::String>(
            camera_name + "/set_config", 10,
            std::bind(&CameraNode::setConfigCallback, this, std::placeholders::_1));
#endif
        o3p::PipelineConfig config;
        if (!recording.empty()) {
            config.setPlaybackFile(recording);
            config.setLoopPlayback(loop_playback);
            config.setUseTimestampsPlayback(use_timestamps);
        } else if (!serial_no.empty()) {
            config.enableDevice(serial_no);
        }
        // Store config so reconnect / hw_reset can re-use it without re-parsing params.
        pipeline_config = config;

        try {
            p.init(config);
        } catch (std::exception &e) {
            std::cerr << "Error initializing camera : " << e.what() << std::endl;
        }

        // Build the CameraInfo for the color and depth streams once, before the
        // pipeline is started. Reading the active profile exchanges messages with the
        // device, so it must not run concurrently with the streaming thread. It is
        // published alongside every RGB/depth frame so the rviz "Camera" display
        // (which requires CameraInfo) can render the image instead of staying blank.
        buildColorCameraInfo();
        buildDepthCameraInfo();

        // Derive the body-frame depth->color transform from the ToF-to-RGB extrinsics
        // read in buildDepthCameraInfo(), then publish the full static TF tree
        // (ROS 2; for ROS 1 the tree is broadcast every cycle in publish_frames()).
        computeColorExtrinsic();
#ifndef ROS1
        broadcastStaticTransforms();
#endif

        // Read the device serial number (and other info) and expose it as a readable parameter, on a latched topic,
        // via the device_info service and as the /diagnostics hardware_id.
        buildDeviceInfo();

        // Configure and start the pipeline
        p.start();

        // Publish the initial use case parameters (preset/fps/hdr/range) so the rviz
        // panel can show the current state as soon as it subscribes.
        publishParameters();
    }

    void publish_loop() {
#ifdef ROS1
        while (ros::ok()) {
            if (reset_requested.exchange(false)) {
                reconnect();
            }
            try {
                publish_frames();
            } catch (const o3p::UsbDisconnectedException &) {
                ROS_WARN("Camera disconnected. Attempting to reconnect...");
                if (reconnect_timeout >= 0.0) {
                    reconnect();
                } else {
                    // Indefinite retry until reconnect succeeds or ROS shuts down.
                    while (ros::ok()) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        try {
                            reconnect();
                            break;
                        } catch (...) {
                            ROS_WARN("Reconnect failed, retrying...");
                        }
                    }
                }
            } catch (const std::exception &e) {
                // A single dropped/failed frame should not bring the node down;
                // log it and keep going so streaming resumes on the next frame.
                ROS_WARN("Skipping frame: %s", e.what());
            }
            ros::spinOnce();
        }
#else
        while (rclcpp::ok()) {
            if (reset_requested.exchange(false)) {
                reconnect();
            }
            try {
                publish_frames();
                // Spinning/sleeping touch the ROS context. If SIGINT arrives mid-loop
                // the context is invalidated and these throw "failed to create guard
                // condition"; break out cleanly in that case instead of letting the
                // error propagate and report a failure exit code.
                if (!rclcpp::ok()) {
                    break;
                }
                rclcpp::spin_some(node);
                if (!rclcpp::ok()) {
                    break;
                }
            } catch (const o3p::UsbDisconnectedException &) {
                RCLCPP_WARN(node->get_logger(), "Camera disconnected. Attempting to reconnect...");
                if (reconnect_timeout >= 0.0) {
                    // One reconnect attempt, then give up if reconnect_timeout == 0
                    // or retry for the configured duration.
                    auto deadline = std::chrono::steady_clock::now() +
                                    std::chrono::duration<double>(reconnect_timeout);
                    bool ok = false;
                    do {
                        try {
                            reconnect();
                            ok = true;
                            break;
                        } catch (...) {
                        }
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
                    if (!ok) {
                        RCLCPP_ERROR(node->get_logger(), "Reconnect timed out. Exiting.");
                        break;
                    }
                } else {
                    // Indefinite retry until reconnect succeeds or ROS shuts down.
                    while (rclcpp::ok()) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        try {
                            reconnect();
                            break;
                        } catch (...) {
                            RCLCPP_WARN(node->get_logger(), "Reconnect failed, retrying...");
                        }
                    }
                }
            } catch (const std::exception &e) {
                if (!rclcpp::ok()) {
                    // Shutting down; the exception is from the invalidated context.
                    break;
                }
                // A single dropped/failed frame should not bring the node down;
                // log it and keep going so streaming resumes on the next frame.
                RCLCPP_WARN(node->get_logger(), "Skipping frame: %s", e.what());
            }
        }
#endif
    }

  private:
    // Build the CameraInfo message for the color stream, scaling the camera
    // intrinsics to the resolution at which the RGB frame is actually published.
    void buildColorCameraInfo() {
        o3p::Intrinsics intr{};
        uint16_t srcWidth = 0;
        uint16_t srcHeight = 0;
        bool found = false;

        try {
            o3p::PipelineProfile profile = p.getActiveProfile();
            for (const auto &stream : profile.getStreams()) {
                if (stream->streamType() != o3p::O3P_STREAM_COLOR) {
                    continue;
                }
                auto videoStream = std::dynamic_pointer_cast<o3p::VideoStream>(stream);
                if (videoStream) {
                    intr = videoStream->getIntrinsics();
                    srcWidth = videoStream->width();
                    srcHeight = videoStream->height();
                    found = true;
                }
                break;
            }
        } catch (const std::exception &e) {
#ifdef ROS1
            ROS_WARN("Could not read color intrinsics: %s", e.what());
#else
            RCLCPP_WARN(node->get_logger(), "Could not read color intrinsics: %s", e.what());
#endif
        }

        // Keep the native (unscaled) color intrinsics for use in the
        // aligned_depth_to_color projection.
        if (found) {
            color_intr_native = intr;
        }

#ifdef ROS1
        camera_info_msg = sensor_msgs::CameraInfo();
        camera_info_msg.header.frame_id = camera_color_optical_frame;
        camera_info_msg.width = srcWidth;
        camera_info_msg.height = srcHeight;
        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.D = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]};
        camera_info_msg.K = {intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0};
        camera_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg.P = {intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0};
#else
        camera_info_msg = sensor_msgs::msg::CameraInfo();
        camera_info_msg.header.frame_id = camera_color_optical_frame;
        camera_info_msg.width = srcWidth;
        camera_info_msg.height = srcHeight;
        camera_info_msg.distortion_model = "plumb_bob";
        camera_info_msg.d = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]};
        camera_info_msg.k = {intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0};
        camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg.p = {intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0};
#endif
    }

    // Build the CameraInfo for the depth/ToF stream and capture the intrinsics and
    // extrinsics needed for aligned_depth_to_color projection.
    void buildDepthCameraInfo() {
        o3p::Intrinsics intr{};
        o3p::Extrinsics extr{};
        uint16_t srcWidth = 0;
        uint16_t srcHeight = 0;
        bool found = false;

        try {
            o3p::PipelineProfile profile = p.getActiveProfile();
            std::shared_ptr<o3p::VideoStream> colorStream;
            std::shared_ptr<o3p::VideoStream> depthStream;

            for (const auto &stream : profile.getStreams()) {
                auto vs = std::dynamic_pointer_cast<o3p::VideoStream>(stream);
                if (!vs) {
                    continue;
                }
                if (stream->streamType() == o3p::O3P_STREAM_DEPTH) {
                    depthStream = vs;
                } else if (stream->streamType() == o3p::O3P_STREAM_COLOR) {
                    colorStream = vs;
                }
            }

            if (depthStream) {
                intr = depthStream->getIntrinsics();
                srcWidth = depthStream->width();
                srcHeight = depthStream->height();
                found = true;

                // Extrinsics: transform from depth (ToF) camera space to RGB camera space.
                // Used to project depth pixels onto the color image plane.
                if (colorStream) {
                    extr = depthStream->getExtrinsicsTo(*colorStream);
                } else {
                    extr = depthStream->getExtrinsics();
                }
            }
        } catch (const std::exception &e) {
#ifdef ROS1
            ROS_WARN("Could not read depth intrinsics: %s", e.what());
#else
            RCLCPP_WARN(node->get_logger(), "Could not read depth intrinsics: %s", e.what());
#endif
        }

        if (found) {
            depth_intr = intr;
            tof_to_rgb_extrinsics = extr;
        }

#ifdef ROS1
        depth_camera_info_msg = sensor_msgs::CameraInfo();
        depth_camera_info_msg.header.frame_id = camera_depth_optical_frame;
        depth_camera_info_msg.width = srcWidth;
        depth_camera_info_msg.height = srcHeight;
        depth_camera_info_msg.distortion_model = "plumb_bob";
        depth_camera_info_msg.D = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]};
        depth_camera_info_msg.K = {intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0};
        depth_camera_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        depth_camera_info_msg.P = {intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0};
#else
        depth_camera_info_msg = sensor_msgs::msg::CameraInfo();
        depth_camera_info_msg.header.frame_id = camera_depth_optical_frame;
        depth_camera_info_msg.width = srcWidth;
        depth_camera_info_msg.height = srcHeight;
        depth_camera_info_msg.distortion_model = "plumb_bob";
        depth_camera_info_msg.d = {intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]};
        depth_camera_info_msg.k = {intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0};
        depth_camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        depth_camera_info_msg.p = {intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0};
#endif
    }

    // Derive the body-frame depth->color static transform from the device ToF-to-RGB
    // extrinsics. The device reports the extrinsics between the optical frames
    // (p_color_optical = R * p_depth_optical + t); this converts that into a transform
    // between the body frames camera_depth_frame and camera_color_frame, using the
    // fixed optical<->body rotation. The result is stored as a translation/quaternion
    // and reused by the ROS 1 and ROS 2 TF broadcasts.
    void computeColorExtrinsic() {
        // The device reports the extrinsic as p_color_optical = R_e * p_depth_optical
        // + t_e, with R_e stored column-major (rotation[col*3 + row] = R_e[row][col]).
        // Reading that column-major array straight into a row-major matrix therefore
        // yields the transpose R_e^T directly, which is exactly what the derivation
        // below needs.
        float reT[9];
        for (int i = 0; i < 9; ++i) {
            reT[i] = tof_to_rgb_extrinsics.rotation[i];
        }

        // R_opt^T (transpose of the optical->body rotation), row-major.
        float optT[9];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                optT[r * 3 + c] = kOpticalToBody[c * 3 + r];
            }
        }

        // R_{depth_frame<-color_frame} = R_opt * R_e^T * R_opt^T
        float tmp[9];
        float rBody[9];
        mat3Mul(kOpticalToBody, reT, tmp);
        mat3Mul(tmp, optT, rBody);

        // t_{depth_frame<-color_frame} = -R_opt * R_e^T * t_e
        float reTt[3];
        mat3Vec(reT, tof_to_rgb_extrinsics.translation, reTt);
        float tBody[3];
        mat3Vec(kOpticalToBody, reTt, tBody);
        color_extr_translation[0] = -static_cast<double>(tBody[0]);
        color_extr_translation[1] = -static_cast<double>(tBody[1]);
        color_extr_translation[2] = -static_cast<double>(tBody[2]);

        quatFromMat(rBody, color_extr_quaternion);
    }

#ifndef ROS1
    // Publish the full static TF tree:
    //   map -> camera_frame (link)
    //   camera_frame -> camera_depth_frame -> camera_depth_optical_frame
    //   camera_frame -> camera_color_frame -> camera_color_optical_frame
    //   camera_frame -> camera_back_plate (centre of the rear plate, from CAD)
    void broadcastStaticTransforms() {
        const rclcpp::Time stamp = node->get_clock()->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        auto makeTransform = [&](const std::string &parent, const std::string &child,
                                 double tx, double ty, double tz, double qx, double qy,
                                 double qz, double qw) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = parent;
            tf.child_frame_id = child;
            tf.transform.translation.x = tx;
            tf.transform.translation.y = ty;
            tf.transform.translation.z = tz;
            tf.transform.rotation.x = qx;
            tf.transform.rotation.y = qy;
            tf.transform.rotation.z = qz;
            tf.transform.rotation.w = qw;
            return tf;
        };

        // Body->optical rotation, quaternion (x, y, z, w) = (-0.5, 0.5, -0.5, 0.5).
        const double ox = -0.5, oy = 0.5, oz = -0.5, ow = 0.5;

        transforms.push_back(makeTransform("map", camera_frame, 0, 0, 0, 0, 0, 0, 1));
        transforms.push_back(
            makeTransform(camera_frame, camera_depth_frame, 0, 0, 0, 0, 0, 0, 1));
        transforms.push_back(makeTransform(camera_depth_frame, camera_depth_optical_frame,
                                           0, 0, 0, ox, oy, oz, ow));
        transforms.push_back(makeTransform(
            camera_frame, camera_color_frame, color_extr_translation[0],
            color_extr_translation[1], color_extr_translation[2],
            color_extr_quaternion[0], color_extr_quaternion[1], color_extr_quaternion[2],
            color_extr_quaternion[3]));
        transforms.push_back(makeTransform(camera_color_frame, camera_color_optical_frame,
                                           0, 0, 0, ox, oy, oz, ow));

        // camera_frame -> camera_back_plate: centre of the rear plate, taken from
        // the CAD assembly. Body-frame offset (X fwd, Y left, Z up), aligned with
        // camera_frame (identity rotation).
        transforms.push_back(makeTransform(
            camera_frame, camera_back_plate_frame, kBackPlateOffsetBody[0],
            kBackPlateOffsetBody[1], kBackPlateOffsetBody[2], 0, 0, 0, 1));

        static_broadcaster->sendTransform(transforms);
    }
#endif

    // Build and publish a JSON metadata string for the given frame.  The fields match
    // the O3P MetadataType enum.
    void publishMetadata(const o3p::Frame &frame) {
        std::ostringstream ss;
        ss << "{";
        bool first = true;

        auto appendUint32 = [&](o3p::MetadataType type, const char *key) {
            if (frame.supportsMetadata(type)) {
                if (!first)
                    ss << ",";
                ss << "\"" << key << "\":" << frame.getMetadata<uint32_t>(type);
                first = false;
            }
        };
        auto appendUint64 = [&](o3p::MetadataType type, const char *key) {
            if (frame.supportsMetadata(type)) {
                if (!first)
                    ss << ",";
                ss << "\"" << key << "\":" << frame.getMetadata<uint64_t>(type);
                first = false;
            }
        };
        auto appendFloat = [&](o3p::MetadataType type, const char *key) {
            if (frame.supportsMetadata(type)) {
                if (!first)
                    ss << ",";
                ss << "\"" << key << "\":" << frame.getMetadata<float>(type);
                first = false;
            }
        };

        appendUint64(o3p::O3P_METADATA_TIMESTAMP, "timestamp");
        appendUint32(o3p::O3P_METADATA_FRAME_NUMBER, "frame_number");
        appendUint32(o3p::O3P_METADATA_EXPOSURE_TIME_1, "exposure_time_1");
        appendUint32(o3p::O3P_METADATA_EXPOSURE_TIME_2, "exposure_time_2");
        appendFloat(o3p::O3P_METADATA_TEMPERATURE, "temperature");
        appendFloat(o3p::O3P_METADATA_ANALOG_GAIN, "analog_gain");
        appendUint32(o3p::O3P_METADATA_WHITE_BALANCE, "white_balance");

        ss << "}";

#ifdef ROS1
        std_msgs::String msg;
        msg.data = ss.str();
        metadata_publisher.publish(msg);
#else
        std_msgs::msg::String msg;
        msg.data = ss.str();
        metadata_publisher->publish(msg);
#endif
    }

    // Stop, re-initialise and restart the pipeline using the stored config.
    // Also rebuilds all cached CameraInfo / intrinsics since stream parameters may
    // have changed after a reconnect (e.g. new firmware).
    void reconnect() {
        try {
            p.stop();
        } catch (...) {
        }
        p.init(pipeline_config);
        buildColorCameraInfo();
        buildDepthCameraInfo();
        // Re-derive and re-publish the depth->color transform; extrinsics may differ
        // after re-initialising the device.
        computeColorExtrinsic();
#ifndef ROS1
        broadcastStaticTransforms();
#endif
        buildDeviceInfo();
        p.start();
        publishParameters();
    }

#ifdef ROS1
    bool hwResetCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
        reset_requested.store(true);
        return true;
    }
#else
    void hwResetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>) {
        reset_requested.store(true);
    }
#endif

    // device. The serial number is cached for the diagnostics topic, written back to
    // the "serial_no" parameter and published once on the latched serial_number topic.
    void buildDeviceInfo() {
        device_info_str.clear();
        try {
            std::shared_ptr<o3p::Device> device = p.getDevice();
            if (device) {
                if (device->supportsInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER)) {
                    serial_number = device->getInfo(o3p::O3P_CAMERA_INFO_SERIAL_NUMBER);
                }
                // Collect every supported info field so the device_info service can
                // return it (serial number, firmware version, ...).
                for (int i = 0; i < o3p::O3P_CAMERA_INFO_COUNT; ++i) {
                    const auto info = static_cast<o3p::CameraInfo>(i);
                    if (device->supportsInfo(info)) {
                        device_info_str += o3p::cameraInfoToString(info) + ": " + device->getInfo(info) + "\n";
                    }
                }
            }
        } catch (const std::exception &e) {
#ifdef ROS1
            ROS_WARN("Could not read device info: %s", e.what());
#else
            RCLCPP_WARN(node->get_logger(), "Could not read device info: %s", e.what());
#endif
        }

#ifdef ROS1
        ROS_INFO("Connected to camera with S/N: %s", serial_number.c_str());
        // Reflect the actual serial number in the parameter so it can be read back
        // with `rosparam get`.
        ros::param::set("~serial_no", serial_number);

        std_msgs::String serial_msg;
        serial_msg.data = serial_number;
        serial_publisher.publish(serial_msg);
#else
        RCLCPP_INFO(node->get_logger(), "Connected to camera with S/N: %s", serial_number.c_str());
        // Reflect the actual serial number in the parameter so it can be read back
        // with `ros2 param get`.
        node->set_parameter(rclcpp::Parameter("serial_no", serial_number));

        std_msgs::msg::String serial_msg;
        serial_msg.data = serial_number;
        serial_publisher->publish(serial_msg);
#endif
    }

    // Publish the device serial number (and stream health) on the /diagnostics topic
    // with the serial number as the hardware_id.
    void publish_diagnostics() {
#ifdef ROS1
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status;
        status.name = camera_name + ": Device";
        status.hardware_id = serial_number;
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        status.message = "Connected";
        diagnostic_msgs::KeyValue kv;
        kv.key = "Serial Number";
        kv.value = serial_number;
        status.values.push_back(kv);
        array.status.push_back(status);
        diagnostics_publisher.publish(array);
#else
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = node->get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = camera_name + ": Device";
        status.hardware_id = serial_number;
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Connected";
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "Serial Number";
        kv.value = serial_number;
        status.values.push_back(kv);
        array.status.push_back(status);
        diagnostics_publisher->publish(array);
#endif
    }

#ifdef ROS1
    bool deviceInfoCallback(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = !serial_number.empty();
        res.message = device_info_str.empty() ? std::string("No device info available") : device_info_str;
        return true;
    }
#else
    void deviceInfoCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = !serial_number.empty();
        response->message = device_info_str.empty() ? std::string("No device info available") : device_info_str;
    }
#endif

#ifdef ROS1
    bool enableAICallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        bool success = p.enableAI(req.data);
        res.success = success;
        res.message = success ? (req.data ? "AI enabled" : "AI disabled") : "Failed to toggle AI";
        return true;
    }
#else
    void enableAICallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        bool success = p.enableAI(request->data);
        response->success = success;
        response->message = success ? (request->data ? "AI enabled" : "AI disabled") : "Failed to toggle AI";
    }
#endif

    // Build a simple line-based "key=value" description of the current use case
    // parameters (preset list, fps, hdr, range) together with the available options
    // and publish it on the latched parameters topic. The rviz panel parses it to
    // fill its comboboxes and show the current state. Lists use '|' as separator.
    void publishParameters() {
        std::ostringstream ss;

        try {
            ss << "presets=";
            bool first = true;
            for (const auto &preset : p.getAvailablePresets()) {
                if (!first) {
                    ss << "|";
                }
                ss << preset.name();
                first = false;
            }
            ss << "\n";
        } catch (...) {
        }

        try {
            ss << "fps=" << p.getFPS() << "\n";
        } catch (...) {
        }
        try {
            ss << "fps_all=";
            bool first = true;
            for (const auto &fps : p.getAllAvailableFPS()) {
                if (!first) {
                    ss << "|";
                }
                ss << fps;
                first = false;
            }
            ss << "\n";
        } catch (...) {
        }

        try {
            ss << "hdr=" << (p.getHDR() ? 1 : 0) << "\n";
        } catch (...) {
        }

        try {
            ss << "range=" << p.getMeasurementRange() << "\n";
        } catch (...) {
        }
        try {
            ss << "range_all=";
            bool first = true;
            for (const auto &range : p.getAllAvailableMeasurementRanges()) {
                if (!first) {
                    ss << "|";
                }
                ss << range;
                first = false;
            }
            ss << "\n";
        } catch (...) {
        }

#ifdef ROS1
        std_msgs::String msg;
        msg.data = ss.str();
        parameters_publisher.publish(msg);
#else
        std_msgs::msg::String msg;
        msg.data = ss.str();
        parameters_publisher->publish(msg);
#endif
    }

    // Apply a single "key=value" parameter command coming from the rviz panel.
    // A change to fps/hdr/range is applied through setCameraParameters together with
    // the other current values to avoid intermediate invalid states. After applying
    // (or rejecting) the command the actual state is re-published so the panel always
    // reflects what the device really uses.
    void applyParameterCommand(const std::string &command) {
        const auto pos = command.find('=');
        if (pos == std::string::npos) {
            return;
        }
        const std::string key = command.substr(0, pos);
        const std::string value = command.substr(pos + 1);

        try {
            if (key == "preset") {
                for (const auto &preset : p.getAvailablePresets()) {
                    if (preset.name() == value) {
                        p.loadPreset(preset);
                        break;
                    }
                }
            } else {
                unsigned fps = p.getFPS();
                bool hdr = p.getHDR();
                std::string range = p.getMeasurementRange();

                if (key == "fps") {
                    fps = static_cast<unsigned>(std::stoul(value));
                } else if (key == "hdr") {
                    hdr = (value == "1" || value == "true");
                } else if (key == "range") {
                    range = value;
                } else {
                    return;
                }

                p.setCameraParameters(fps, hdr, range);
            }
        } catch (const std::exception &e) {
#ifdef ROS1
            ROS_WARN("Could not apply parameter '%s': %s", command.c_str(), e.what());
#else
            RCLCPP_WARN(node->get_logger(), "Could not apply parameter '%s': %s", command.c_str(), e.what());
#endif
        }

        publishParameters();
    }

#ifdef ROS1
    void setParametersCallback(const std_msgs::String::ConstPtr &msg) {
        if (msg) {
            applyParameterCommand(msg->data);
        }
    }
#else
    void setParametersCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
            applyParameterCommand(msg->data);
        }
    }
#endif

    // Apply a single extended-option (config) command coming from the rviz panel and
    // publish the resulting value on the latched config_result topic.
    // The command is either "get=<key>" or "set=<key>=<value>". This uses the same
    // pipeline calls (getConfigValue / setConfigValue) as the O3pViewer config dialog.
    void applyConfigCommand(const std::string &command) {
        const auto modePos = command.find('=');
        if (modePos == std::string::npos) {
            return;
        }
        const std::string mode = command.substr(0, modePos);
        const std::string rest = command.substr(modePos + 1);

        std::string key;
        std::string result;

        try {
            if (mode == "set") {
                const auto valPos = rest.find('=');
                if (valPos == std::string::npos) {
                    return;
                }
                key = rest.substr(0, valPos);
                const std::string value = rest.substr(valPos + 1);
                p.setConfigValue(key, value);
                result = p.getConfigValue(key);
            } else if (mode == "get") {
                key = rest;
                result = p.getConfigValue(key);
            } else {
                return;
            }
        } catch (const std::exception &e) {
#ifdef ROS1
            ROS_WARN("Could not apply config command '%s': %s", command.c_str(), e.what());
#else
            RCLCPP_WARN(node->get_logger(), "Could not apply config command '%s': %s", command.c_str(), e.what());
#endif
            publishConfigResult(key, std::string("ERROR: ") + e.what());
            return;
        }

        publishConfigResult(key, result.empty() ? std::string("NA") : result);
    }

    void publishConfigResult(const std::string &key, const std::string &value) {
        const std::string data = key + "=" + value;
#ifdef ROS1
        std_msgs::String msg;
        msg.data = data;
        config_result_publisher.publish(msg);
#else
        std_msgs::msg::String msg;
        msg.data = data;
        config_result_publisher->publish(msg);
#endif
    }

#ifdef ROS1
    void setConfigCallback(const std_msgs::String::ConstPtr &msg) {
        if (msg) {
            applyConfigCommand(msg->data);
        }
    }
#else
    void setConfigCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg) {
            applyConfigCommand(msg->data);
        }
    }
#endif

    void publish_frames() {
        // Report V4L2 dropped frames (Linux live camera only) once per minute.
        auto _now = std::chrono::steady_clock::now();
        if (_now - m_lastDroppedFrameReport >= std::chrono::minutes(1)) {
            m_lastDroppedFrameReport = _now;
            uint64_t dropped = p.getAndResetDroppedFrames();
            if (dropped > 0) {
#ifdef ROS1
                ROS_WARN("%lu frame(s) dropped in the last minute (incomplete USB transfers)",
                         static_cast<unsigned long>(dropped));
#else
                RCLCPP_WARN(node->get_logger(),
                            "%lu frame(s) dropped in the last minute (incomplete USB transfers)",
                            static_cast<unsigned long>(dropped));
#endif
            }
        }

        o3p::FrameSet frames = p.waitForFrames();

        // POINT CLOUD
        auto pc = frames.getPointCloudFrame();
        if (pc) {
#ifdef ROS1
            // Broadcast the full TF tree every cycle. ROS 1 has no
            // latched static transform broadcaster here, so the tree is re-sent with
            // each frame.
            ros::Time now = ros::Time::now();
            // Body->optical rotation, quaternion (x, y, z, w) = (-0.5, 0.5, -0.5, 0.5).
            tf::Quaternion q_optical(-0.5, 0.5, -0.5, 0.5);

            tf::Transform t_link;
            t_link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            t_link.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            br.sendTransform(tf::StampedTransform(t_link, now, "map", camera_frame));

            tf::Transform t_depth;
            t_depth.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            t_depth.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            br.sendTransform(tf::StampedTransform(t_depth, now, camera_frame, camera_depth_frame));

            tf::Transform t_depth_opt;
            t_depth_opt.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            t_depth_opt.setRotation(q_optical);
            br.sendTransform(
                tf::StampedTransform(t_depth_opt, now, camera_depth_frame, camera_depth_optical_frame));

            // camera_frame -> camera_color_frame from the device ToF-to-RGB extrinsics.
            tf::Transform t_color;
            t_color.setOrigin(tf::Vector3(color_extr_translation[0], color_extr_translation[1],
                                          color_extr_translation[2]));
            t_color.setRotation(tf::Quaternion(color_extr_quaternion[0], color_extr_quaternion[1],
                                               color_extr_quaternion[2], color_extr_quaternion[3]));
            br.sendTransform(tf::StampedTransform(t_color, now, camera_frame, camera_color_frame));

            tf::Transform t_color_opt;
            t_color_opt.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            t_color_opt.setRotation(q_optical);
            br.sendTransform(
                tf::StampedTransform(t_color_opt, now, camera_color_frame, camera_color_optical_frame));

            // camera_frame -> camera_back_plate: centre of the rear plate (from CAD).
            tf::Transform t_back;
            t_back.setOrigin(tf::Vector3(kBackPlateOffsetBody[0], kBackPlateOffsetBody[1],
                                         kBackPlateOffsetBody[2]));
            t_back.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
            br.sendTransform(tf::StampedTransform(t_back, now, camera_frame, camera_back_plate_frame));

            sensor_msgs::PointCloud2 msg_pc;
            msg_pc.header.stamp = now;
            msg_pc.header.frame_id = camera_depth_optical_frame;
            msg_pc.width = pc->m_width;
            msg_pc.height = pc->m_height;
            msg_pc.is_bigendian = false;
            msg_pc.is_dense = false;
            msg_pc.point_step = static_cast<uint32_t>(4 * sizeof(float));
            msg_pc.row_step = static_cast<uint32_t>(4 * sizeof(float) * pc->m_width);
            int numPoints = pc->m_width * pc->m_height;

            sensor_msgs::PointCloud2Modifier modifier(msg_pc);
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "conf", 1, sensor_msgs::PointField::FLOAT32);
            float *cloudPtr = reinterpret_cast<float *>(&msg_pc.data[0]);
            ::memcpy(cloudPtr, pc->m_data, 4 * sizeof(float) * numPoints);

            pc_publisher.publish(msg_pc);
#else
            sensor_msgs::msg::PointCloud2 msg_pc;
            msg_pc.header.stamp = node->get_clock()->now();

            msg_pc.header.frame_id = camera_depth_optical_frame;
            msg_pc.width = pc->m_width;
            msg_pc.height = pc->m_height;
            msg_pc.is_dense = false;
            int numPoints = pc->m_width * pc->m_height;

            msg_pc.point_step = static_cast<uint32_t>(4 * sizeof(float));
            msg_pc.row_step = static_cast<uint32_t>(4 * sizeof(float) * pc->m_width);
            msg_pc.data.resize(4 * sizeof(float) * numPoints);

            sensor_msgs::PointCloud2Modifier modifier(msg_pc);
            modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                          "conf", 1, sensor_msgs::msg::PointField::FLOAT32);
            float *cloudPtr = reinterpret_cast<float *>(&msg_pc.data[0]);
            ::memcpy(cloudPtr, pc->m_data, 4 * sizeof(float) * numPoints);

            pc_publisher->publish(msg_pc);
#endif
        }

        // DEPTH
        auto depth = frames.getDepthFrame();

        auto width = depth->getWidth();
        auto height = depth->getHeight();

        cv::Mat zImageRGB;

        if (width > 0 && height > 0) {
            cv::Mat zImage = cv::Mat(height, width, CV_16UC1, (uint16_t *)depth->m_data);

            // Convert to colored depth image for viewing purposes only
            cv::Mat zImage8;
            cv::convertScaleAbs(zImage, zImage8, 1.0);
            cv::applyColorMap(zImage8, zImageRGB, cv::COLORMAP_JET);
        }

#ifdef ROS1
        auto msg_depth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", zImageRGB).toImageMsg();
        depth_publisher.publish(msg_depth);

        // Raw 16UC1 depth in mm + its CameraInfo.
        if (width > 0 && height > 0) {
            cv::Mat zRaw(height, width, CV_16UC1, (uint16_t *)depth->m_data);
            std_msgs::Header depth_header;
            depth_header.frame_id = camera_depth_optical_frame;
            depth_header.stamp = ros::Time::now();
            auto msg_depth_raw = cv_bridge::CvImage(depth_header, "16UC1", zRaw).toImageMsg();
            depth_raw_publisher.publish(msg_depth_raw);
            depth_camera_info_msg.header.stamp = depth_header.stamp;
            depth_camera_info_publisher.publish(depth_camera_info_msg);
        }
        publishMetadata(*depth);
#else
        auto header = std_msgs::msg::Header();
        header.stamp = node->get_clock()->now();
        header.frame_id = camera_depth_optical_frame; // depth/IR optical frame (vital to foxy)

        auto msg_depth = cv_bridge::CvImage(header, "bgr8", zImageRGB).toImageMsg();
        depth_publisher->publish(*msg_depth);

        // Raw 16UC1 depth in mm + its CameraInfo.
        if (width > 0 && height > 0) {
            cv::Mat zRaw(height, width, CV_16UC1, (uint16_t *)depth->m_data);
            auto msg_depth_raw = cv_bridge::CvImage(header, "16UC1", zRaw).toImageMsg();
            depth_raw_publisher->publish(*msg_depth_raw);
            depth_camera_info_msg.header.stamp = header.stamp;
            depth_camera_info_publisher->publish(depth_camera_info_msg);
        }
        publishMetadata(*depth);
#endif
        // IR
        auto ir = frames.getInfraredFrame();
        width = ir->getWidth();
        height = ir->getHeight();

        cv::Mat irImageBGR;

        if (width > 0 && height > 0) {
            cv::Mat irImage = cv::Mat(height, width, CV_8UC1, (uint8_t *)ir->m_data);
            cv::cvtColor(irImage, irImageBGR, cv::COLOR_GRAY2BGR);
        }

#ifdef ROS1
        auto msg_ir = cv_bridge::CvImage(std_msgs::Header(), "bgr8", irImageBGR).toImageMsg();
        ir_publisher.publish(msg_ir);
#else
        auto msg_ir = cv_bridge::CvImage(header, "bgr8", irImageBGR).toImageMsg();
        ir_publisher->publish(*msg_ir);
#endif

        // RGB
        auto color = frames.getColorFrame();

        width = color->getWidth();
        height = color->getHeight();

        cv::Mat picBGR;

        // The color frame can be delivered in different pixel formats; m_bitsPerPixel
        // tells which one. Always decoding it as NV12 (as before) corrupts frames that
        // arrive as RGB/BGRA and shows up as green/pink flicker. Decode according to
        // the actual format and only when the buffer is complete.
        const size_t numPixels = static_cast<size_t>(width) * height;
        const size_t expectedSize = numPixels * color->m_bitsPerPixel / 8;
        if (width > 0 && height > 0 && color->m_data != nullptr && expectedSize > 0 &&
            color->m_dataCopy.size() >= expectedSize) {
            if (color->m_bitsPerPixel == 12) {
                // NV12 (YUV 4:2:0): full plane is height*3/2 rows of width bytes.
                cv::Mat picYV12(height * 3 / 2, width, CV_8UC1, color->m_data);
                cv::cvtColor(picYV12, picBGR, cv::COLOR_YUV2BGR_NV12);
            } else if (color->m_bitsPerPixel == 24) {
                // Packed RGB.
                cv::Mat rgb(height, width, CV_8UC3, color->m_data);
                cv::cvtColor(rgb, picBGR, cv::COLOR_RGB2BGR);
            } else if (color->m_bitsPerPixel == 32) {
                // Packed BGRA.
                cv::Mat bgra(height, width, CV_8UC4, color->m_data);
                cv::cvtColor(bgra, picBGR, cv::COLOR_BGRA2BGR);
            }
        }

        if (!picBGR.empty()) {
            // Cache the last valid frame. Color frames can arrive at a lower rate
            // than the publish loop, so reusing the last good image keeps the topic
            // steady instead of alternating with blank/garbage frames (flicker).
            last_rgb = picBGR;
        }

        // Publish the last valid frame (and matching CameraInfo) so the topic always
        // carries a complete image once the first color frame has been received.
        if (!last_rgb.empty()) {
#ifdef ROS1
            std_msgs::Header rgb_header;
            rgb_header.stamp = ros::Time::now();
            rgb_header.frame_id = camera_color_optical_frame;
            auto msg_rgb = cv_bridge::CvImage(rgb_header, "bgr8", last_rgb).toImageMsg();
            rgb_publisher.publish(msg_rgb);

            camera_info_msg.header.stamp = rgb_header.stamp;
            camera_info_publisher.publish(camera_info_msg);
#else
            std_msgs::msg::Header color_header;
            color_header.stamp = header.stamp;
            color_header.frame_id = camera_color_optical_frame;
            auto msg_rgb = cv_bridge::CvImage(color_header, "bgr8", last_rgb).toImageMsg();
            rgb_publisher->publish(*msg_rgb);

            camera_info_msg.header.stamp = color_header.stamp;
            camera_info_publisher->publish(camera_info_msg);
#endif
        }

        // Only run the expensive color-to-depth alignment when something is subscribed.
#ifdef ROS1
        const bool aligned_needed = aligned_depth_publisher.getNumSubscribers() > 0 ||
                                    aligned_depth_info_publisher.getNumSubscribers() > 0;
#else
        const bool aligned_needed = aligned_depth_publisher->get_subscription_count() > 0 ||
                                    aligned_depth_info_publisher->get_subscription_count() > 0;
#endif
        if (aligned_needed) {
            o3p::alignColorToDepth(frames, tof_to_rgb_extrinsics, color_intr_native, depth_intr);
            auto aligned_color = frames.getColorFrame();
            auto alignedW = aligned_color->getWidth();
            auto alignedH = aligned_color->getHeight();
            const size_t alignedExpected =
                static_cast<size_t>(alignedW) * alignedH * aligned_color->m_bitsPerPixel / 8;
            cv::Mat alignedBGR;
            if (alignedW > 0 && alignedH > 0 && aligned_color->m_data != nullptr &&
                alignedExpected > 0 && aligned_color->m_dataCopy.size() >= alignedExpected) {
                if (aligned_color->m_bitsPerPixel == 12) {
                    cv::Mat picYV12(alignedH * 3 / 2, alignedW, CV_8UC1, aligned_color->m_data);
                    cv::cvtColor(picYV12, alignedBGR, cv::COLOR_YUV2BGR_NV12);
                } else if (aligned_color->m_bitsPerPixel == 24) {
                    cv::Mat rgb(alignedH, alignedW, CV_8UC3, aligned_color->m_data);
                    cv::cvtColor(rgb, alignedBGR, cv::COLOR_RGB2BGR);
                } else if (aligned_color->m_bitsPerPixel == 32) {
                    cv::Mat bgra(alignedH, alignedW, CV_8UC4, aligned_color->m_data);
                    cv::cvtColor(bgra, alignedBGR, cv::COLOR_BGRA2BGR);
                }
            }
            if (!alignedBGR.empty()) {
#ifdef ROS1
                std_msgs::Header aligned_header;
                aligned_header.frame_id = camera_depth_optical_frame;
                aligned_header.stamp = ros::Time::now();
                auto msg_aligned = cv_bridge::CvImage(aligned_header, "bgr8", alignedBGR).toImageMsg();
                aligned_depth_publisher.publish(msg_aligned);
                depth_camera_info_msg.header.stamp = aligned_header.stamp;
                aligned_depth_info_publisher.publish(depth_camera_info_msg);
#else
                std_msgs::msg::Header aligned_header;
                aligned_header.stamp = header.stamp;
                aligned_header.frame_id = camera_depth_optical_frame;
                auto msg_aligned = cv_bridge::CvImage(aligned_header, "bgr8", alignedBGR).toImageMsg();
                aligned_depth_publisher->publish(*msg_aligned);
                depth_camera_info_msg.header.stamp = aligned_header.stamp;
                aligned_depth_info_publisher->publish(depth_camera_info_msg);
#endif
            }
        }

        // IMU
        auto imu = frames.getImuFrame();

#ifdef ROS1
        sensor_msgs::Imu msg_imu_accel;
        msg_imu_accel.header.stamp = ros::Time::now();
        msg_imu_accel.header.frame_id = imu_frame;
        msg_imu_accel.linear_acceleration.x = imu->acceleration[0];
        msg_imu_accel.linear_acceleration.y = imu->acceleration[1];
        msg_imu_accel.linear_acceleration.z = imu->acceleration[2];
        // orientation unknown
        msg_imu_accel.orientation_covariance[0] = -1.0;

        sensor_msgs::Imu msg_imu_gyro;
        msg_imu_gyro.header.stamp = ros::Time().fromNSec(imu->angularTimestamp);
        msg_imu_gyro.header.frame_id = imu_frame;
        msg_imu_gyro.angular_velocity.x = imu->angularVelocity[0];
        msg_imu_gyro.angular_velocity.y = imu->angularVelocity[1];
        msg_imu_gyro.angular_velocity.z = imu->angularVelocity[2];
        // orientation unknown
        msg_imu_gyro.orientation_covariance[0] = -1.0;

        accel_publisher.publish(msg_imu_accel);
        gyro_publisher.publish(msg_imu_gyro);
#else
        sensor_msgs::msg::Imu msg_imu_accel;
        msg_imu_accel.header.stamp = node->get_clock()->now();
        msg_imu_accel.header.frame_id = imu_frame;
        msg_imu_accel.linear_acceleration.x = imu->acceleration[0];
        msg_imu_accel.linear_acceleration.y = imu->acceleration[1];
        msg_imu_accel.linear_acceleration.z = imu->acceleration[2];
        // orientation unknown
        msg_imu_accel.orientation_covariance[0] = -1.0;

        sensor_msgs::msg::Imu msg_imu_gyro;
        // The device angular timestamp is a raw counter that can exceed what a ROS
        // time stamp (int32 seconds field) can represent, which would overflow into a
        // negative time and crash any consumer constructing an rclcpp::Time from it.
        // Fall back to the node clock when the value is out of the representable range.
        constexpr uint64_t kMaxStampNanos = 2147483647ull * 1000000000ull;
        if (imu->angularTimestamp > 0 && imu->angularTimestamp < kMaxStampNanos) {
            msg_imu_gyro.header.stamp = rclcpp::Time(static_cast<int64_t>(imu->angularTimestamp));
        } else {
            msg_imu_gyro.header.stamp = node->get_clock()->now();
        }
        msg_imu_gyro.header.frame_id = imu_frame;
        msg_imu_gyro.angular_velocity.x = imu->angularVelocity[0];
        msg_imu_gyro.angular_velocity.y = imu->angularVelocity[1];
        msg_imu_gyro.angular_velocity.z = imu->angularVelocity[2];
        // orientation unknown
        msg_imu_gyro.orientation_covariance[0] = -1.0;

        accel_publisher->publish(msg_imu_accel);
        gyro_publisher->publish(msg_imu_gyro);
#endif

        // Publish the serial number / device health on /diagnostics every cycle.
        publish_diagnostics();
    }

    o3p::Pipeline p;
    o3p::PipelineConfig pipeline_config; // stored for reconnect / hw_reset

    std::string serial_no;
    std::string camera_name;
    std::string recording;
    bool loop_playback{true};
    bool use_timestamps{true};
    double wait_for_device_timeout{-1.0};
    double reconnect_timeout{-1.0};
    std::string camera_frame;
    std::string camera_depth_frame;
    std::string camera_depth_optical_frame;
    std::string camera_color_frame;
    std::string camera_color_optical_frame;
    std::string camera_back_plate_frame;
    std::string imu_frame;

    // Serial number of the connected device and the full device info string returned
    // by the device_info service.
    std::string serial_number;
    std::string device_info_str;

    // Intrinsics/extrinsics cached at init for aligned_depth_to_color projection.
    o3p::Intrinsics depth_intr{};
    o3p::Intrinsics color_intr_native{};
    o3p::Extrinsics tof_to_rgb_extrinsics{};

    // Body-frame camera_frame -> camera_color_frame transform derived from the
    // ToF-to-RGB extrinsics (translation in meters, quaternion as x, y, z, w).
    double color_extr_translation[3]{0.0, 0.0, 0.0};
    double color_extr_quaternion[4]{0.0, 0.0, 0.0, 1.0};

    // Set to true by the hw_reset service callback; the publish loop picks it up
    // and performs the reset synchronously to avoid concurrent pipeline access.
    std::atomic<bool> reset_requested{false};

    // Timestamp of the last dropped-frame report. Used to log V4L2 dropped frames
    // (Linux live camera only) once per minute when the count is above zero.
    std::chrono::steady_clock::time_point m_lastDroppedFrameReport{std::chrono::steady_clock::now()};

    PointCloudPublisher pc_publisher;
    FramePublisher depth_publisher;
    FramePublisher depth_raw_publisher;
    FramePublisher aligned_depth_publisher;
    FramePublisher ir_publisher;
    FramePublisher rgb_publisher;
    CameraInfoPublisher camera_info_publisher;
    CameraInfoPublisher depth_camera_info_publisher;
    CameraInfoPublisher aligned_depth_info_publisher;
    ImuPublisher accel_publisher;
    ImuPublisher gyro_publisher;

    // Last valid RGB frame, reused while no new color frame is available.
    cv::Mat last_rgb;

#ifdef ROS1
    sensor_msgs::CameraInfo camera_info_msg;
    sensor_msgs::CameraInfo depth_camera_info_msg;
    tf::TransformBroadcaster br;
    ros::ServiceServer enable_ai_service;
    ros::ServiceServer device_info_service;
    ros::ServiceServer hw_reset_service;
    ros::Publisher serial_publisher;
    ros::Publisher diagnostics_publisher;
    ros::Publisher metadata_publisher;
    ros::Publisher parameters_publisher;
    ros::Subscriber set_parameters_subscriber;
    ros::Publisher config_result_publisher;
    ros::Subscriber set_config_subscriber;
#else
    sensor_msgs::msg::CameraInfo camera_info_msg;
    sensor_msgs::msg::CameraInfo depth_camera_info_msg;
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_ai_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr device_info_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr hw_reset_service;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_publisher;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr parameters_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_parameters_subscriber;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_result_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_config_subscriber;
#endif
};

int main(int argc, char **argv) {
#ifdef ROS1
    ros::init(argc, argv, "camera_publisher");
#else
    rclcpp::init(argc, argv);
#endif
    try {
        CameraNode cam_node;
        cam_node.publish_loop();
    } catch (const std::exception &e) {
#ifdef ROS1
        ROS_ERROR("Error: %s", e.what());
#else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error: %s", e.what());
#endif
        return 1;
    }

#ifdef ROS1
    return 0;
#else
    rclcpp::shutdown();
    return 0;
#endif
}
