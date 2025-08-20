#ifndef VISUALIZATION_PUB_H
#define VISUALIZATION_PUB_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <sstream>
#include <vector>

#include "canopen_interfaces/msg/co_data.hpp"
#include "orl_common/common.hpp"
#include "orl_interfaces/msg/control_trajectory.hpp"
#include "orl_interfaces/msg/reacsa_state.hpp"
#include "orl_interfaces/msg/solenoid_valve_state.hpp"
#include "orl_interfaces/msg/thrusters_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class VisualizationPublisher : public rclcpp::Node
{
  public:
	VisualizationPublisher();

  private:
	//  Callback functions
	void state_callback(const orl_interfaces::msg::ReacsaState::SharedPtr msg);
	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void reaction_wheel_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
	void waypoint_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
	void solenoid_valve_callback(const orl_interfaces::msg::SolenoidValveState::SharedPtr msg);
	void pressure_sensor_highline_callback(const canopen_interfaces::msg::COData::SharedPtr msg);
	void pressure_sensor_midline_callback(const canopen_interfaces::msg::COData::SharedPtr msg);
	void pressure_sensor_lowline_callback(const canopen_interfaces::msg::COData::SharedPtr msg);
	void thrusters_callback(const orl_interfaces::msg::ThrustersCommand::SharedPtr msg);
	void wrench_reactionwheel_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
	void planned_trajectory_callback(const orl_interfaces::msg::ControlTrajectory::SharedPtr msg);
	void requested_trajectory_callback(const orl_interfaces::msg::ControlTrajectory::SharedPtr msg);
	void wrench_timer_callback();
	void marker_timer_callback();

	// Member pointers
	rclcpp::Subscription<orl_interfaces::msg::ReacsaState>::SharedPtr state_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reaction_wheel_sub;
	rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr waypoint_sub;
	rclcpp::Subscription<orl_interfaces::msg::SolenoidValveState>::SharedPtr solenoid_valve_state_sub;
	rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr pressure_sensor_highline_sub;
	rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr pressure_sensor_midline_sub;
	rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr pressure_sensor_lowline_sub;
	rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_reactionwheel_sub;
	rclcpp::Subscription<orl_interfaces::msg::ThrustersCommand>::SharedPtr thrusters_sub;
	rclcpp::Subscription<orl_interfaces::msg::ControlTrajectory>::SharedPtr planned_trajectory_sub;
	rclcpp::Subscription<orl_interfaces::msg::ControlTrajectory>::SharedPtr requested_trajectory_sub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planned_marker_trajectory_nodes_pub, marker_text_pub,
	    marker_mesh_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr requested_marker_trajectory_nodes_pub,
	    marker_pressure_bar_pub;
	rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_thrusters_pub, wrench_reactionwheel_pub;
	rclcpp::TimerBase::SharedPtr wrench_timer, marker_timer;

	// Member variables
	visualization_msgs::msg::Marker info_text_marker, flatfloor_mesh_marker, planned_trajectory_nodes_marker,
	    requested_trajectory_nodes_marker, requested_trajectory_line_marker, high_status_bar_marker,
	    high_status_bar_bg_marker, mid_status_bar_marker, mid_status_bar_bg_marker, low_status_bar_marker,
	    low_status_bar_bg_marker;
	visualization_msgs::msg::MarkerArray requested_trajectory_marker_array, pressure_bar_array;
	double estimated_x, estimated_y, estimated_theta, estimated_rw_vel, measured_x, measured_y, measured_theta,
	    measured_rw_vel, waypoint_x, waypoint_y, waypoint_theta;
	double reactionwheel;
	bool isSolenoidValveClosed;
	std::vector<double> thrusters = std::vector<double>(8, 0.0);
	geometry_msgs::msg::WrenchStamped thrusters_wrench, reactionwheel_wrench;

	float curr_pressure_high;  // Current pressure value
	float curr_pressure_mid;
	float curr_pressure_low;

	rclcpp::Time lowline_pressure_timer;
	rclcpp::Time midline_pressure_timer;
	rclcpp::Time highline_pressure_timer;

	// Config constants
	int max_pressure_low;
	int max_pressure_mid;
	int max_pressure_high;
	std::vector<double> critical_pressure_low;
	std::vector<int64_t> critical_pressure_mid;
	std::vector<int64_t> critical_pressure_high;
	double no_pressure_data_max_time;
};

#endif  // VISUALIZATION_PUB_H
