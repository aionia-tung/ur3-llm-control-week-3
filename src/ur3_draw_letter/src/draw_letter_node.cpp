#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <Eigen/Geometry>


using namespace std::chrono_literals;


class DrawLetter
{
public:

    explicit DrawLetter(
        const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        // ============================================================
        // MoveIt
        // ============================================================

        move_group_ =
            std::make_shared<
                moveit::planning_interface::MoveGroupInterface>(
                    node_,
                    "ur_manipulator");

        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(10);

        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);

        move_group_->setPoseReferenceFrame("world");
        move_group_->setEndEffectorLink("tool0");

        if (!move_group_->startStateMonitor(10.0))
        {
            RCLCPP_WARN(node_->get_logger(), "Could not start current state monitor");
        }


        // ============================================================
        // Publisher
        // ============================================================

        letter_marker_pub_ =
            node_->create_publisher<
                visualization_msgs::msg::Marker>(
                    "/letter_path",
                    10);

        eef_marker_pub_ =
            node_->create_publisher<
                visualization_msgs::msg::Marker>(
                    "/eef_trajectory",
                    10);


        // ============================================================
        // Timer publish Marker liên tục
        //
        // 10 Hz
        // ============================================================

        marker_timer_ =
            node_->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]()
                {
                    publishMarkers();
                });


        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");

        RCLCPP_INFO(
            node_->get_logger(),
            "DrawLetter initialized");

        RCLCPP_INFO(
            node_->get_logger(),
            "Planning frame: %s",
            move_group_->getPlanningFrame().c_str());

        RCLCPP_INFO(
            node_->get_logger(),
            "EEF link: %s",
            move_group_->getEndEffectorLink().c_str());

        RCLCPP_INFO(
            node_->get_logger(),
            "Publish topics:");

        RCLCPP_INFO(
            node_->get_logger(),
            "  /letter_path");

        RCLCPP_INFO(
            node_->get_logger(),
            "  /eef_trajectory");

        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");
    }


    // ================================================================
    // Publish Marker định kỳ
    // ================================================================

    void publishMarkers()
    {
        // ------------------------------------------------------------
        // Letter path
        // ------------------------------------------------------------

        if (!letter_marker_.points.empty())
        {
            letter_marker_.header.stamp =
                node_->now();

            letter_marker_pub_->publish(
                letter_marker_);
        }


        // ------------------------------------------------------------
        // EEF trajectory
        // ------------------------------------------------------------

        if (!eef_marker_.points.empty())
        {
            eef_marker_.header.stamp =
                node_->now();

            eef_marker_pub_->publish(
                eef_marker_);
        }
    }


    // ================================================================
    // Orientation tool hướng xuống
    // ================================================================

    geometry_msgs::msg::Quaternion downwardOrientation()
    {
        return move_group_->getCurrentPose("tool0").pose.orientation;
    }


    // ================================================================
    // Create Pose
    // ================================================================

    geometry_msgs::msg::Pose createPose(
        double x,
        double y,
        double z)
    {
        geometry_msgs::msg::Pose pose;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        pose.orientation =
            downwardOrientation();

        return pose;
    }


    // ================================================================
    // Tạo Marker đường chữ L lý tưởng
    // ================================================================

    void publishLetterPath(
        const std::vector<
            geometry_msgs::msg::Pose>& path)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id =
            move_group_->getPlanningFrame();

        marker.header.stamp =
            node_->now();

        marker.ns =
            "letter_path";

        marker.id = 0;

        marker.type =
            visualization_msgs::msg::Marker::LINE_STRIP;

        marker.action =
            visualization_msgs::msg::Marker::ADD;


        // ============================================================
        // Marker không tự biến mất
        // ============================================================

        marker.lifetime =
            rclcpp::Duration::from_seconds(0);


        // ============================================================
        // Độ dày
        // ============================================================

        marker.scale.x = 0.01;


        // ============================================================
        // Màu đỏ
        // ============================================================

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;


        // ============================================================
        // Orientation marker
        // ============================================================

        marker.pose.orientation.w = 1.0;


        // ============================================================
        // Points
        // ============================================================

        for (const auto& pose : path)
        {
            geometry_msgs::msg::Point point;

            point.x =
                pose.position.x;

            point.y =
                pose.position.y;

            point.z =
                pose.position.z;

            marker.points.push_back(point);
        }


        // ============================================================
        // Lưu Marker lại
        //
        // Không publish một lần duy nhất nữa.
        // Timer sẽ publish liên tục.
        // ============================================================

        letter_marker_ = marker;


        RCLCPP_INFO(
            node_->get_logger(),
            "Letter marker created: %zu points",
            letter_marker_.points.size());

        RCLCPP_INFO(
            node_->get_logger(),
            "Letter frame: %s",
            letter_marker_.header.frame_id.c_str());
    }


    // ================================================================
    // Lấy đường đi EEF từ RobotTrajectory
    // ================================================================

    void publishActualEEFPath(
        const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        auto robot_model =
            move_group_->getRobotModel();


        if (!robot_model)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Robot model is null");

            return;
        }


        // ============================================================
        // RobotState
        // ============================================================

        moveit::core::RobotState robot_state(
            robot_model);

        robot_state.setToDefaultValues();


        // ============================================================
        // tool0
        // ============================================================

        const moveit::core::LinkModel* link_model =
            robot_model->getLinkModel("tool0");


        if (!link_model)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cannot find tool0");

            return;
        }


        const auto& joint_names =
            trajectory.joint_trajectory.joint_names;

        const auto& points =
            trajectory.joint_trajectory.points;


        RCLCPP_INFO(
            node_->get_logger(),
            "Trajectory has %zu points",
            points.size());


        // ============================================================
        // Duyệt trajectory
        // ============================================================

        for (const auto& trajectory_point : points)
        {
            if (trajectory_point.positions.size()
                != joint_names.size())
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Joint position size mismatch");

                continue;
            }


            // --------------------------------------------------------
            // Set joint positions
            // --------------------------------------------------------

            for (size_t i = 0;
                 i < joint_names.size();
                 ++i)
            {
                robot_state.setVariablePosition(
                    joint_names[i],
                    trajectory_point.positions[i]);
            }


            // --------------------------------------------------------
            // Forward kinematics
            // --------------------------------------------------------

            robot_state.update();


            // --------------------------------------------------------
            // tool0 transform
            // --------------------------------------------------------

            const Eigen::Isometry3d&
                transform =
                    robot_state.getGlobalLinkTransform(
                        link_model);


            // --------------------------------------------------------
            // XYZ
            // --------------------------------------------------------

            geometry_msgs::msg::Point point;

            point.x =
                transform.translation().x();

            point.y =
                transform.translation().y();

            point.z =
                transform.translation().z();


            // --------------------------------------------------------
            // Lưu vào EEF path
            // --------------------------------------------------------

            eef_path_.push_back(point);
        }


        // ============================================================
        // Tạo Marker
        // ============================================================

        visualization_msgs::msg::Marker marker;

        marker.header.frame_id =
            move_group_->getPlanningFrame();

        marker.header.stamp =
            node_->now();

        marker.ns =
            "eef_trajectory";

        marker.id = 0;

        marker.type =
            visualization_msgs::msg::Marker::LINE_STRIP;

        marker.action =
            visualization_msgs::msg::Marker::ADD;


        // ============================================================
        // Không tự biến mất
        // ============================================================

        marker.lifetime =
            rclcpp::Duration::from_seconds(0);


        // ============================================================
        // Độ dày
        // ============================================================

        marker.scale.x = 0.005;


        // ============================================================
        // Màu xanh lá
        // ============================================================

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;


        // ============================================================
        // Orientation
        // ============================================================

        marker.pose.orientation.w = 1.0;


        // ============================================================
        // Toàn bộ EEF path
        // ============================================================

        marker.points =
            eef_path_;


        // ============================================================
        // Lưu Marker
        //
        // Timer sẽ publish liên tục.
        // ============================================================

        eef_marker_ = marker;


        RCLCPP_INFO(
            node_->get_logger(),
            "EEF marker created: %zu points",
            eef_marker_.points.size());

        RCLCPP_INFO(
            node_->get_logger(),
            "EEF frame: %s",
            eef_marker_.header.frame_id.c_str());
    }


    // ================================================================
    // Retime trajectory for the low-gain Gazebo position interface.
    // ================================================================

    bool retimeTrajectory(
        moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        auto current_state = move_group_->getCurrentState(5.0);
        auto robot_model = move_group_->getRobotModel();

        if (!current_state || !robot_model)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cannot retime trajectory without current robot state");
            return false;
        }

        robot_trajectory::RobotTrajectory robot_trajectory(
            robot_model,
            "ur_manipulator");
        robot_trajectory.setRobotTrajectoryMsg(
            *current_state,
            trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
        if (!time_parameterization.computeTimeStamps(
                robot_trajectory,
                0.03,
                0.03))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Could not retime trajectory");
            return false;
        }

        robot_trajectory.getRobotTrajectoryMsg(trajectory);
        return true;
    }


    // ================================================================
    // Move tới Pose
    // ================================================================

    bool moveToPose(
        const geometry_msgs::msg::Pose& target_pose)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Move to: x=%.3f y=%.3f z=%.3f",
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z);


        move_group_->setStartStateToCurrentState();

        move_group_->setPoseTarget(
            target_pose);



        moveit::planning_interface::
            MoveGroupInterface::Plan plan;


        auto result =
            move_group_->plan(plan);

        move_group_->clearPoseTargets();

        if (result == moveit::core::MoveItErrorCode::SUCCESS &&
            !retimeTrajectory(plan.trajectory_))
        {
            return false;
        }


        if (result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Planning failed");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "Planning successful");


        auto execute_result =
            move_group_->execute(plan);


        if (execute_result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Execution failed");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "Execution successful");


        return true;
    }


    // ================================================================
    // Cartesian Move
    // ================================================================

    bool cartesianMove(
        const std::vector<
            geometry_msgs::msg::Pose>& waypoints)
    {
        if (waypoints.empty())
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Waypoints are empty");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "Computing Cartesian path: %zu waypoints",
            waypoints.size());

        auto current_state = move_group_->getCurrentState(5.0);
        if (!current_state)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Could not obtain current robot state for Cartesian path");
            return false;
        }
        move_group_->setStartState(*current_state);


        moveit_msgs::msg::RobotTrajectory trajectory;


        // ============================================================
        // Cartesian path
        //
        // eef_step = 5 mm
        // jump_threshold = 0
        // ============================================================

        double fraction =
            move_group_->computeCartesianPath(
                waypoints,
                0.005,
                0.0,
                trajectory,
                true);


        RCLCPP_INFO(
            node_->get_logger(),
            "Cartesian fraction: %.2f%%",
            fraction * 100.0);


        if (fraction < 0.99)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cartesian path incomplete");

            return false;
        }


        // ============================================================
        // Plan
        // ============================================================

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        plan.trajectory_ =
            trajectory;

        if (!retimeTrajectory(plan.trajectory_))
        {
            return false;
        }


        // ============================================================
        // Execute
        // ============================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Executing Cartesian trajectory...");


        auto result =
            move_group_->execute(plan);


        if (result !=
            moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Cartesian execution failed");

            return false;
        }


        RCLCPP_INFO(
            node_->get_logger(),
            "Cartesian execution successful");


        // ============================================================
        // Lấy EEF path
        // ============================================================

        publishActualEEFPath(
            trajectory);


        return true;
    }


    // ================================================================
    // Execute a long Cartesian stroke as short segments.
    // This keeps the IK solution continuous near workspace boundaries.
    // ================================================================

    bool cartesianMoveSegmented(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Pose& target)
    {
        const double dx = target.position.x - start.position.x;
        const double dy = target.position.y - start.position.y;
        const double dz = target.position.z - start.position.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.01)));

        for (int i = 1; i <= steps; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            geometry_msgs::msg::Pose waypoint = start;
            waypoint.position.x = start.position.x + t * dx;
            waypoint.position.y = start.position.y + t * dy;
            waypoint.position.z = start.position.z + t * dz;
            waypoint.orientation = start.orientation;

            if (!cartesianMove({waypoint}))
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Cartesian segment %d/%d failed",
                    i,
                    steps);
                return false;
            }

            std::this_thread::sleep_for(100ms);
        }

        return true;
    }


    // ================================================================
    // Vẽ chữ L
    // ================================================================

    void drawLetterL()
    {
        RCLCPP_INFO(
            node_->get_logger(),
            " ");
        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");
        RCLCPP_INFO(
            node_->get_logger(),
            "START DRAWING LETTER L");
        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");


        // ============================================================
        // Xóa EEF trajectory cũ
        // ============================================================

        eef_path_.clear();

        eef_marker_.points.clear();


        // ============================================================
        // P1
        // ============================================================

        geometry_msgs::msg::Pose P1 =
            move_group_->getCurrentPose("tool0").pose;
        P1.orientation = downwardOrientation();

        const double x0 = P1.position.x;
        const double y0 = P1.position.y;
        const double z0 = P1.position.z;


        // ============================================================
        // P2
        // ============================================================

        geometry_msgs::msg::Pose P2 =
            createPose(x0 + 0.06, y0, z0);
        P2.orientation = P1.orientation;


        // ============================================================
        // P3
        // ============================================================

        geometry_msgs::msg::Pose P3 =
            createPose(x0 + 0.06, y0 - 0.05, z0);
        P3.orientation = P1.orientation;


        // ============================================================
        // P4 - nhấc tool
        // ============================================================

        geometry_msgs::msg::Pose P4 =
            createPose(x0 + 0.06, y0 - 0.05, z0 + 0.02);
        P4.orientation = P1.orientation;


        // ============================================================
        // Ideal letter path
        // ============================================================

        std::vector<
            geometry_msgs::msg::Pose>
            letter_path;

        letter_path.push_back(P1);
        letter_path.push_back(P2);
        letter_path.push_back(P3);


        publishLetterPath(
            letter_path);


        // P1 is captured from the current tool pose.  Starting the
        // Cartesian path directly avoids asking MoveIt for a new IK branch
        // for a pose that is already reached.
        RCLCPP_INFO(
            node_->get_logger(),
            "Starting Cartesian drawing from the current tool pose");


        // ============================================================
        // P1 -> P2
        // ============================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Drawing P1 -> P2");


        std::vector<
            geometry_msgs::msg::Pose>
            path_P1_P2;

        path_P1_P2.push_back(P2);


        if (!cartesianMoveSegmented(P1, P2))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "P1 -> P2 failed");

            return;
        }


        // ============================================================
        // P2 -> P3
        // ============================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Drawing P2 -> P3");


        std::vector<
            geometry_msgs::msg::Pose>
            path_P2_P3;

        path_P2_P3.push_back(P3);


        if (!cartesianMoveSegmented(P2, P3))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "P2 -> P3 failed");

            return;
        }


        // ============================================================
        // P3 -> P4
        // ============================================================

        RCLCPP_INFO(
            node_->get_logger(),
            "Lifting P3 -> P4");


        std::vector<
            geometry_msgs::msg::Pose>
            path_P3_P4;

        path_P3_P4.push_back(P4);


        if (!cartesianMoveSegmented(P3, P4))
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "P3 -> P4 failed");

            return;
        }


        // ============================================================
        // DONE
        // ============================================================

        RCLCPP_INFO(
            node_->get_logger(),
            " ");
        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");
        RCLCPP_INFO(
            node_->get_logger(),
            "LETTER L DRAWING FINISHED");
        RCLCPP_INFO(
            node_->get_logger(),
            "EEF points: %zu",
            eef_path_.size());
        RCLCPP_INFO(
            node_->get_logger(),
            "Markers are continuously published.");
        RCLCPP_INFO(
            node_->get_logger(),
            "RViz can display them at any time.");
        RCLCPP_INFO(
            node_->get_logger(),
            "Press Ctrl+C to exit.");
        RCLCPP_INFO(
            node_->get_logger(),
            "========================================");
    }


private:

    // ================================================================
    // ROS Node
    // ================================================================

    rclcpp::Node::SharedPtr node_;


    // ================================================================
    // MoveIt
    // ================================================================

    std::shared_ptr<
        moveit::planning_interface::
        MoveGroupInterface>
        move_group_;


    // ================================================================
    // Publishers
    // ================================================================

    rclcpp::Publisher<
        visualization_msgs::msg::Marker>::SharedPtr
        letter_marker_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker>::SharedPtr
        eef_marker_pub_;


    // ================================================================
    // Timer
    // ================================================================

    rclcpp::TimerBase::SharedPtr
        marker_timer_;


    // ================================================================
    // Marker được lưu lại
    // ================================================================

    visualization_msgs::msg::Marker
        letter_marker_;

    visualization_msgs::msg::Marker
        eef_marker_;


    // ================================================================
    // EEF trajectory
    // ================================================================

    std::vector<
        geometry_msgs::msg::Point>
        eef_path_;
};


// ====================================================================
// MAIN
// ====================================================================

int main(
    int argc,
    char* argv[])
{
    rclcpp::init(
        argc,
        argv);


    // ================================================================
    // Node
    // ================================================================

    auto node =
        std::make_shared<rclcpp::Node>(
            "draw_letter_node",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true));


    // ================================================================
    // DrawLetter
    // ================================================================

    auto draw_letter =
        std::make_shared<DrawLetter>(
            node);


    // ================================================================
    // Executor
    // ================================================================

    rclcpp::executors::
        SingleThreadedExecutor executor;

    executor.add_node(node);


    // ================================================================
    // Spin thread
    // ================================================================

    std::thread spinner(
        [&executor]()
        {
            executor.spin();
        });


    // ================================================================
    // Chờ MoveIt
    // ================================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Waiting 2 seconds for MoveIt...");

    std::this_thread::sleep_for(
        2s);


    // ================================================================
    // Draw
    // ================================================================

    draw_letter->drawLetterL();


    // ================================================================
    // Giữ node chạy
    // ================================================================

    RCLCPP_INFO(
        node->get_logger(),
        " ");
    RCLCPP_INFO(
        node->get_logger(),
        "Node remains alive.");
    RCLCPP_INFO(
        node->get_logger(),
        "Marker topics are publishing at 10 Hz.");
    RCLCPP_INFO(
        node->get_logger(),
        "Press Ctrl+C to stop.");


    // ================================================================
    // Chờ Ctrl+C
    // ================================================================

    spinner.join();


    // ================================================================
    // Shutdown
    // ================================================================

    rclcpp::shutdown();

    return 0;
}