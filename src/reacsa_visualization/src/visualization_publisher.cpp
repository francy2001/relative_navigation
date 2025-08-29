#include "visualization_publisher.hpp"

VisualizationPublisher::VisualizationPublisher() : Node("info_text_publisher")
{
	// Retrieve config constants
	this->declare_parameter("max_pressure_low", 10);
	this->max_pressure_low = this->get_parameter("max_pressure_low").as_int();
	this->declare_parameter("max_pressure_mid", 25);
	this->max_pressure_mid = this->get_parameter("max_pressure_mid").as_int();
	this->declare_parameter("max_pressure_high", 300);
	this->max_pressure_high = this->get_parameter("max_pressure_high").as_int();
	this->declare_parameter("critical_pressure_low", std::vector<double>({6.7, 8}));
	this->critical_pressure_low = this->get_parameter("critical_pressure_low").as_double_array();
	this->declare_parameter("critical_pressure_mid", std::vector<int64_t>({7, 10}));
	this->critical_pressure_mid = this->get_parameter("critical_pressure_mid").as_integer_array();
	this->declare_parameter("critical_pressure_high", std::vector<int64_t>({50, 250}));
	this->critical_pressure_high = this->get_parameter("critical_pressure_high").as_integer_array();
	this->declare_parameter("no_pressure_data_max_time", 5.0);
	this->no_pressure_data_max_time = this->get_parameter("no_pressure_data_max_time").as_double();
	// Reacsa state subscriber
	this->state_sub = this->create_subscription<orl_interfaces::msg::ReacsaState>(
	    TOPIC_ROBOT_STATE, QOS_BEST_EFFORT_NO_DEPTH, std::bind(&VisualizationPublisher::state_callback, this, _1));
	// Reacsa pose subscriber
	this->pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
	    TOPIC_POSE, QOS_BEST_EFFORT_NO_DEPTH, std::bind(&VisualizationPublisher::pose_callback, this, _1));
	// Reaction_wheel state subscriber
	this->reaction_wheel_sub = this->create_subscription<sensor_msgs::msg::JointState>(
	    TOPIC_JOINT_STATES,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::reaction_wheel_callback, this, _1));
	// Waypoint marker subscriber
	this->waypoint_sub = this->create_subscription<visualization_msgs::msg::Marker>(
	    "/visual/" + TOPIC_MARKER_WAYPOINT_ORIENTATION,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::waypoint_callback, this, _1));
	// Valve state subscriber
	this->solenoid_valve_state_sub = this->create_subscription<orl_interfaces::msg::SolenoidValveState>(
	    TOPIC_SOLENOID_VALVE_STATE,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::solenoid_valve_callback, this, std::placeholders::_1));
	// Highline pressure sensor subscriber
	this->pressure_sensor_highline_sub = this->create_subscription<canopen_interfaces::msg::COData>(
	    TOPIC_PRESSURE_SENSOR_HIGHLINE,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::pressure_sensor_highline_callback, this, std::placeholders::_1));
	// Midline pressure sensor subscriber
	this->pressure_sensor_midline_sub = this->create_subscription<canopen_interfaces::msg::COData>(
	    TOPIC_PRESSURE_SENSOR_MIDLINE,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::pressure_sensor_midline_callback, this, std::placeholders::_1));
	this->pressure_sensor_lowline_sub = this->create_subscription<canopen_interfaces::msg::COData>(
	    TOPIC_PRESSURE_SENSOR_LOWLINE,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::pressure_sensor_lowline_callback, this, std::placeholders::_1));
	// Thrusters subscriber
	this->thrusters_sub = this->create_subscription<orl_interfaces::msg::ThrustersCommand>(
	    TOPIC_THRUSTERS_STATE,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::thrusters_callback, this, _1));
	// Reaction wheel subscriber
	this->wrench_reactionwheel_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
	    TOPIC_WRENCH_REACTION_WHEEL,
	    QOS_RELIABLE_NO_DEPTH,
	    std::bind(&VisualizationPublisher::wrench_reactionwheel_callback, this, _1));
	// Trajectory subscriber
	this->planned_trajectory_sub = this->create_subscription<orl_interfaces::msg::ControlTrajectory>(
	    TOPIC_PLANNED_TRAJECTORY,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::planned_trajectory_callback, this, _1));
	this->requested_trajectory_sub = this->create_subscription<orl_interfaces::msg::ControlTrajectory>(
	    TOPIC_REQUESTED_TRAJECTORY,
	    QOS_BEST_EFFORT_NO_DEPTH,
	    std::bind(&VisualizationPublisher::requested_trajectory_callback, this, _1));

	// Wrench publishers
	this->wrench_thrusters_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
	    "/visual/" + TOPIC_WRENCH_THRUSTERS, QOS_RELIABLE_NO_DEPTH);  // Overwrite namespace
	this->wrench_reactionwheel_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
	    "/visual/" + TOPIC_WRENCH_REACTION_WHEEL, QOS_RELIABLE_NO_DEPTH);  // Overwrite namespace
	this->wrench_timer =
	    this->create_wall_timer(100ms, std::bind(&VisualizationPublisher::wrench_timer_callback, this));

	// Marker publishers
	this->marker_text_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visual/" + TOPIC_MARKER_TELEMETRY,
	                                                                                QOS_RELIABLE_NO_DEPTH);
	this->marker_mesh_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visual/" + TOPIC_MARKER_FLATFLOOR,
	                                                                                QOS_RELIABLE_PERSISTENT);
	this->planned_marker_trajectory_nodes_pub = this->create_publisher<visualization_msgs::msg::Marker>(
	    "/visual/" + TOPIC_MARKER_PLANNED_TRAJECTORY, QOS_RELIABLE_NO_DEPTH);
	this->requested_marker_trajectory_nodes_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
	    "/visual/" + TOPIC_MARKER_REQUESTED_TRAJECTORY, QOS_RELIABLE_NO_DEPTH);
	this->marker_pressure_bar_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
	    "/visual/" + TOPIC_MARKER_PRESSURE_BAR, QOS_RELIABLE_NO_DEPTH);
	this->marker_timer =
	    this->create_wall_timer(300ms, std::bind(&VisualizationPublisher::marker_timer_callback, this));

	// Init info text marker
	info_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
	info_text_marker.scale.z = 0.2;
	info_text_marker.color.r = 0;
	info_text_marker.color.g = 0;
	info_text_marker.color.b = 0;
	info_text_marker.color.a = 1.0;
	info_text_marker.pose.position.x = 4;
	info_text_marker.pose.position.y = 1;
	info_text_marker.pose.position.z = 0.2;
	info_text_marker.header.frame_id = "world";
	info_text_marker.action = visualization_msgs::msg::Marker::ADD;

	// Create the status bar marker
	high_status_bar_marker.header.frame_id = "world";
	high_status_bar_marker.id = 0;
	high_status_bar_marker.type = visualization_msgs::msg::Marker::CUBE;
	high_status_bar_marker.pose.position.y = info_text_marker.pose.position.y - 1.5;
	high_status_bar_marker.pose.position.z = info_text_marker.pose.position.z;
	high_status_bar_marker.pose.orientation.w = 1.0;
	high_status_bar_marker.scale.y = 0.2;
	high_status_bar_marker.scale.z = 0.2;
	high_status_bar_marker.color.b = 0;
	high_status_bar_marker.color.a = 1.0;
	high_status_bar_marker.action = visualization_msgs::msg::Marker::ADD;

	high_status_bar_bg_marker.header.frame_id = "world";
	high_status_bar_bg_marker.id = 5;
	high_status_bar_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
	high_status_bar_bg_marker.pose.position.x = info_text_marker.pose.position.x + 0.5;
	high_status_bar_bg_marker.pose.position.y = info_text_marker.pose.position.y - 1.5;
	high_status_bar_bg_marker.pose.position.z = info_text_marker.pose.position.z;
	high_status_bar_bg_marker.pose.orientation.w = 1.0;
	high_status_bar_bg_marker.scale.x = 4.0;
	high_status_bar_bg_marker.scale.y = 0.25;
	high_status_bar_bg_marker.scale.z = 0.25;
	high_status_bar_bg_marker.color.a = 0.5;
	high_status_bar_bg_marker.action = visualization_msgs::msg::Marker::ADD;

	mid_status_bar_marker.header.frame_id = "world";
	mid_status_bar_marker.id = 10;
	mid_status_bar_marker.type = visualization_msgs::msg::Marker::CUBE;
	mid_status_bar_marker.pose.position.y = info_text_marker.pose.position.y - 2.3;
	mid_status_bar_marker.pose.position.z = info_text_marker.pose.position.z;
	mid_status_bar_marker.pose.orientation.w = 1.0;
	mid_status_bar_marker.scale.y = 0.2;
	mid_status_bar_marker.scale.z = 0.2;
	mid_status_bar_marker.color.g = 0;
	mid_status_bar_marker.color.a = 1.0;
	mid_status_bar_marker.action = visualization_msgs::msg::Marker::ADD;

	mid_status_bar_bg_marker.header.frame_id = "world";
	mid_status_bar_bg_marker.id = 15;
	mid_status_bar_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
	mid_status_bar_bg_marker.pose.position.x = info_text_marker.pose.position.x + 0.5;
	mid_status_bar_bg_marker.pose.position.y = info_text_marker.pose.position.y - 2.3;
	mid_status_bar_bg_marker.pose.position.z = info_text_marker.pose.position.z;
	mid_status_bar_bg_marker.pose.orientation.w = 1.0;
	mid_status_bar_bg_marker.scale.x = 4.0;
	mid_status_bar_bg_marker.scale.y = 0.25;
	mid_status_bar_bg_marker.scale.z = 0.25;
	mid_status_bar_bg_marker.color.a = 0.5;
	mid_status_bar_bg_marker.action = visualization_msgs::msg::Marker::ADD;

	low_status_bar_marker.header.frame_id = "world";
	low_status_bar_marker.id = 20;
	low_status_bar_marker.type = visualization_msgs::msg::Marker::CUBE;
	low_status_bar_marker.pose.position.y = info_text_marker.pose.position.y - 3.1;
	low_status_bar_marker.pose.position.z = info_text_marker.pose.position.z;
	low_status_bar_marker.pose.orientation.w = 1.0;
	low_status_bar_marker.scale.y = 0.2;
	low_status_bar_marker.scale.z = 0.2;
	low_status_bar_marker.color.g = 0;
	low_status_bar_marker.color.a = 1.0;
	low_status_bar_marker.action = visualization_msgs::msg::Marker::ADD;

	low_status_bar_bg_marker.header.frame_id = "world";
	low_status_bar_bg_marker.id = 25;
	low_status_bar_bg_marker.type = visualization_msgs::msg::Marker::CUBE;
	low_status_bar_bg_marker.pose.position.x = info_text_marker.pose.position.x + 0.5;
	low_status_bar_bg_marker.pose.position.y = info_text_marker.pose.position.y - 3.1;
	low_status_bar_bg_marker.pose.position.z = info_text_marker.pose.position.z;
	low_status_bar_bg_marker.pose.orientation.w = 1.0;
	low_status_bar_bg_marker.scale.x = 4.0;
	low_status_bar_bg_marker.scale.y = 0.25;
	low_status_bar_bg_marker.scale.z = 0.25;
	low_status_bar_bg_marker.color.a = 0.5;
	low_status_bar_bg_marker.action = visualization_msgs::msg::Marker::ADD;

	// Flatfloor mesh marker
	// Providing absolute path:
	// https://answers.ros.org/question/288501/ros2-equivalent-of-rospackagegetpath/
	// https://answers.ros.org/question/282745/rviz-doesnt-load-dae-mesh-cannot-locate-it/
	flatfloor_mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	flatfloor_mesh_marker.mesh_resource = "file://" + ament_index_cpp::get_package_share_directory("orbit_simulation")
	                                      + "/models/ORBIT/materials/textures/heightmap_513.dae";
	flatfloor_mesh_marker.lifetime = rclcpp::Duration(0, 0);  // lifetime=0 marker is displayed forever
	flatfloor_mesh_marker.scale.x = 1;
	flatfloor_mesh_marker.scale.y = 1;
	flatfloor_mesh_marker.scale.z = 1;
	flatfloor_mesh_marker.pose.position.x = 1.999205;
	flatfloor_mesh_marker.pose.position.y = 0.015795;
	flatfloor_mesh_marker.mesh_use_embedded_materials = true;
	flatfloor_mesh_marker.header.frame_id = "world";
	flatfloor_mesh_marker.action = visualization_msgs::msg::Marker::ADD;

	// Only publish the mesh marker once since it will not change over time
	marker_mesh_pub->publish(flatfloor_mesh_marker);

	// Init trajectory markers
	planned_trajectory_nodes_marker.type = visualization_msgs::msg::Marker::POINTS;
	planned_trajectory_nodes_marker.header.stamp =
	    rclcpp::Time(0);                     // T=0 marker is displayed regardless of current time
	planned_trajectory_nodes_marker.id = 0;  // Marker ID needs to be unique for it to be displayed in an array
	planned_trajectory_nodes_marker.scale.x = 0.04;
	planned_trajectory_nodes_marker.scale.y = 0.04;
	planned_trajectory_nodes_marker.color.a = 1.0;
	planned_trajectory_nodes_marker.pose.orientation.w = 1.0;
	planned_trajectory_nodes_marker.header.frame_id = "world";

	requested_trajectory_nodes_marker.type = visualization_msgs::msg::Marker::POINTS;
	requested_trajectory_nodes_marker.header.stamp =
	    rclcpp::Time(0);                       // T=0 marker is displayed regardless of current time
	requested_trajectory_nodes_marker.id = 1;  // Marker ID needs to be unique for it to be displayed in an array
	requested_trajectory_nodes_marker.scale.x = 0.04;
	requested_trajectory_nodes_marker.scale.y = 0.04;
	requested_trajectory_nodes_marker.color.a = 1.0;
	requested_trajectory_nodes_marker.color.r = 0.0;
	requested_trajectory_nodes_marker.color.g = 1.0;
	requested_trajectory_nodes_marker.color.b = 0.0;
	requested_trajectory_nodes_marker.pose.orientation.w = 1.0;
	requested_trajectory_nodes_marker.header.frame_id = "world";

	requested_trajectory_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
	requested_trajectory_line_marker.header.stamp =
	    rclcpp::Time(0);                      // T=0 marker is displayed regardless of current time
	requested_trajectory_line_marker.id = 2;  // Marker ID needs to be unique for it to be displayed in an array
	requested_trajectory_line_marker.scale.x = 0.05;
	requested_trajectory_line_marker.color.a = 0.5;
	requested_trajectory_line_marker.color.r = 0.0;
	requested_trajectory_line_marker.color.g = 0.5;
	requested_trajectory_line_marker.color.b = 0.0;
	requested_trajectory_line_marker.pose.orientation.w = 1.0;
	requested_trajectory_line_marker.header.frame_id = "world";
}

void VisualizationPublisher::state_callback(const orl_interfaces::msg::ReacsaState::SharedPtr msg)
{
	this->estimated_x = msg->pose.position.x;
	this->estimated_y = msg->pose.position.y;
	this->estimated_theta = RAD_2_DEG * quat_2_euler_z(msg->pose.orientation);
	this->estimated_rw_vel = RADPS_2_RPM * msg->rw_vel;
}

void VisualizationPublisher::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	this->measured_x = msg->pose.position.x;
	this->measured_y = msg->pose.position.y;
	this->measured_theta = RAD_2_DEG * quat_2_euler_z(msg->pose.orientation);
}

void VisualizationPublisher::reaction_wheel_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
	this->measured_rw_vel = msg->velocity[0];
}

void VisualizationPublisher::waypoint_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
	this->waypoint_x = msg->pose.position.x;
	this->waypoint_y = msg->pose.position.y;
	this->waypoint_theta = RAD_2_DEG * quat_2_euler_z(msg->pose.orientation);
}

void VisualizationPublisher::thrusters_callback(const orl_interfaces::msg::ThrustersCommand::SharedPtr msg)
{
	std::transform(msg->fire.begin(), msg->fire.end(), this->thrusters.begin(), [](auto& c) {
		return double(c) * FORCE_THRUSTER;
	});
}

void VisualizationPublisher::wrench_reactionwheel_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
	this->reactionwheel = msg->wrench.torque.z;
}

void VisualizationPublisher::solenoid_valve_callback(const orl_interfaces::msg::SolenoidValveState::SharedPtr msg)
{
	this->isSolenoidValveClosed = msg->is_closed;
}

void VisualizationPublisher::pressure_sensor_highline_callback(const canopen_interfaces::msg::COData::SharedPtr msg)
{
	this->highline_pressure_timer = this->now();
	this->curr_pressure_high = msg->data;
}

void VisualizationPublisher::pressure_sensor_midline_callback(const canopen_interfaces::msg::COData::SharedPtr msg)
{
	this->midline_pressure_timer = this->now();
	// The value 10000 comes from the decimal digits sdo of the sensor
	this->curr_pressure_mid = msg->data / 10000.0;
}

void VisualizationPublisher::pressure_sensor_lowline_callback(const canopen_interfaces::msg::COData::SharedPtr msg)
{
	this->lowline_pressure_timer = this->now();
	// The value 1000 comes from the decimal digits sdo of the sensor
	this->curr_pressure_low = msg->data / 1000.0;
}

void VisualizationPublisher::planned_trajectory_callback(const orl_interfaces::msg::ControlTrajectory::SharedPtr msg)
{
	if (!msg->times.empty())
	{
		this->planned_trajectory_nodes_marker.points.clear();
		this->planned_trajectory_nodes_marker.action = visualization_msgs::msg::Marker::ADD;

		for (unsigned int i = 0; i < msg->times.size(); i++)
		{
			geometry_msgs::msg::Point points;
			points.x = msg->states[i * msg->state_dim];
			points.y = msg->states[i * msg->state_dim + 1];
			points.z = 0.1;
			this->planned_trajectory_nodes_marker.points.push_back(points);
		}
	}
	else  // Empty trajectory message indicates ending or cancelling of a trajectory, therefore delete markers
	{
		this->planned_trajectory_nodes_marker.action = visualization_msgs::msg::Marker::DELETE;
	}

	this->planned_marker_trajectory_nodes_pub->publish(this->planned_trajectory_nodes_marker);
}

void VisualizationPublisher::requested_trajectory_callback(const orl_interfaces::msg::ControlTrajectory::SharedPtr msg)
{
	this->requested_trajectory_marker_array.markers.clear();

	if (!msg->times.empty())
	{
		this->requested_trajectory_nodes_marker.points.clear();
		this->requested_trajectory_nodes_marker.action = visualization_msgs::msg::Marker::ADD;
		this->requested_trajectory_line_marker.points.clear();
		this->requested_trajectory_line_marker.action = visualization_msgs::msg::Marker::ADD;

		for (unsigned int i = 0; i < msg->times.size(); i++)
		{
			geometry_msgs::msg::Point points;
			points.x = msg->states[i * msg->state_dim];
			points.y = msg->states[i * msg->state_dim + 1];
			points.z = 0.1;
			this->requested_trajectory_nodes_marker.points.push_back(points);
			this->requested_trajectory_line_marker.points.push_back(points);
		}
	}
	else  // Empty trajectory message indicates ending or cancelling of a trajectory, therefore delete markers
	{
		this->requested_trajectory_nodes_marker.action = visualization_msgs::msg::Marker::DELETE;
		this->requested_trajectory_line_marker.action = visualization_msgs::msg::Marker::DELETE;
	}

	this->requested_trajectory_marker_array.markers.push_back(this->requested_trajectory_nodes_marker);
	this->requested_trajectory_marker_array.markers.push_back(this->requested_trajectory_line_marker);

	this->requested_marker_trajectory_nodes_pub->publish(this->requested_trajectory_marker_array);
}

void VisualizationPublisher::wrench_timer_callback()
{
	// If thrust is being applied, publish value
	// INFO: No timestamp is published to avoid crashing RVIZ
	double full_force = 0.0;
	for (auto it = this->thrusters.begin(); it != this->thrusters.end(); ++it)
	{
		int index = std::distance(this->thrusters.begin(), it);
		full_force += *it;
		if (*it != 0)
		{
			thrusters_wrench.header.frame_id = "thruster" + std::to_string(index);
			thrusters_wrench.wrench.force.y = *it;
			wrench_thrusters_pub->publish(thrusters_wrench);
		}
	}

	// If no thrust is being applied, simply publish zeros on acrobat (random)
	if (full_force < 0.001)
	{
		thrusters_wrench.header.frame_id = "ACROBAT";
		thrusters_wrench.wrench.force.y = 0;
		wrench_thrusters_pub->publish(thrusters_wrench);
	}

	// Publish reaction wheel torque at less high frequency
	reactionwheel_wrench.header.frame_id = "reaction_wheel";
	reactionwheel_wrench.wrench.force.z = this->reactionwheel;
	wrench_reactionwheel_pub->publish(reactionwheel_wrench);
}

void VisualizationPublisher::marker_timer_callback()
{
	// Update text marker
	std::stringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "Reacsa estimated state:\n"
	       << "x: " << this->estimated_x << " m,\ny: " << this->estimated_y << " m,\nt: " << this->estimated_theta
	       << " deg,\nw: " << this->estimated_rw_vel << " rpm\n";
	stream << "\nReacsa measured state:\n"
	       << "x: " << this->measured_x << " m,\ny: " << this->measured_y << " m,\nt: " << this->measured_theta
	       << " deg,\nw: " << this->measured_rw_vel * RADPS_2_RPM << " rpm\n";
	stream << "\nWaypoint state:\n"
	       << "x: " << this->waypoint_x << " m,\ny: " << this->waypoint_y << " m,\nt: " << this->waypoint_theta
	       << " deg\n";
	stream << "\nSolenoid valve state:\n" << (this->isSolenoidValveClosed ? "Closed" : "Open\n");
	stream << "\nHighline pressure: " << this->curr_pressure_high << "bar\n \n \n";
	stream << "\nMidline pressure: " << this->curr_pressure_mid << "bar\n \n \n";
	stream << "\nLowline pressure: " << this->curr_pressure_low << "bar\n";
	info_text_marker.text = stream.str();

	// Check whether the pressure readings are received correctly
	if (this->now().seconds() - this->highline_pressure_timer.seconds() > no_pressure_data_max_time
	    || this->highline_pressure_timer.seconds() == 0)
	{
		// Use a timer to alternate the color every 500ms
		static auto last_highline_toggle_time = std::chrono::steady_clock::now();
		if (std::chrono::steady_clock::now() - last_highline_toggle_time >= std::chrono::milliseconds(500))
		{
			// Toggle the color
			high_status_bar_bg_marker.color.r = (high_status_bar_bg_marker.color.r == 1) ? 0.6 : 1;
			high_status_bar_bg_marker.color.g = (high_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			high_status_bar_bg_marker.color.b = (high_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			last_highline_toggle_time = std::chrono::steady_clock::now();
		}
	}
	else
	{
		high_status_bar_bg_marker.color.r = 0.6;
		high_status_bar_bg_marker.color.g = 0.6;
		high_status_bar_bg_marker.color.b = 0.6;
	}

	if (this->now().seconds() - this->midline_pressure_timer.seconds() > no_pressure_data_max_time
	    || this->midline_pressure_timer.seconds() == 0)
	{
		// Use a timer to alternate the color every 500ms
		static auto last_midline_toggle_time = std::chrono::steady_clock::now();
		if (std::chrono::steady_clock::now() - last_midline_toggle_time >= std::chrono::milliseconds(500))
		{
			// Toggle the color
			mid_status_bar_bg_marker.color.r = (mid_status_bar_bg_marker.color.r == 1) ? 0.6 : 1;
			mid_status_bar_bg_marker.color.g = (mid_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			mid_status_bar_bg_marker.color.b = (mid_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			last_midline_toggle_time = std::chrono::steady_clock::now();
		}
	}
	else
	{
		mid_status_bar_bg_marker.color.r = 0.6;
		mid_status_bar_bg_marker.color.g = 0.6;
		mid_status_bar_bg_marker.color.b = 0.6;
	}

	if (this->now().seconds() - this->lowline_pressure_timer.seconds() > no_pressure_data_max_time
	    || this->lowline_pressure_timer.seconds() == 0)
	{
		// Use a timer to alternate the color every 500ms
		static auto last_lowline_toggle_time = std::chrono::steady_clock::now();
		if (std::chrono::steady_clock::now() - last_lowline_toggle_time >= std::chrono::milliseconds(500))
		{
			// Toggle the color
			low_status_bar_bg_marker.color.r = (low_status_bar_bg_marker.color.r == 1) ? 0.6 : 1;
			low_status_bar_bg_marker.color.g = (low_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			low_status_bar_bg_marker.color.b = (low_status_bar_bg_marker.color.r == 1) ? 0 : 0.6;
			last_lowline_toggle_time = std::chrono::steady_clock::now();
		}
	}
	else
	{
		low_status_bar_bg_marker.color.r = 0.6;
		low_status_bar_bg_marker.color.g = 0.6;
		low_status_bar_bg_marker.color.b = 0.6;
	}

	// Update the scale of the status bar based on the value of the pressure variable
	this->high_status_bar_marker.scale.x =
	    this->high_status_bar_bg_marker.scale.x * this->curr_pressure_high / this->max_pressure_high;
	this->mid_status_bar_marker.scale.x =
	    this->mid_status_bar_bg_marker.scale.x * this->curr_pressure_mid / this->max_pressure_mid;
	this->low_status_bar_marker.scale.x =
	    this->low_status_bar_bg_marker.scale.x * this->curr_pressure_low / this->max_pressure_low;

	// As the default positioning in rviz is centered, the status bar must be aligned to the left to overlap the
	// background bar.
	this->high_status_bar_marker.pose.position.x =
	    this->high_status_bar_bg_marker.pose.position.x
	    - (this->high_status_bar_bg_marker.scale.x - this->high_status_bar_marker.scale.x) / 2;
	this->mid_status_bar_marker.pose.position.x =
	    this->mid_status_bar_bg_marker.pose.position.x
	    - (this->mid_status_bar_bg_marker.scale.x - this->mid_status_bar_marker.scale.x) / 2;
	this->low_status_bar_marker.pose.position.x =
	    this->low_status_bar_bg_marker.pose.position.x
	    - (this->low_status_bar_bg_marker.scale.x - this->low_status_bar_marker.scale.x) / 2;

	// Change the color of the status bar when pressure is out of the set limits
	bool high_status = this->curr_pressure_high > critical_pressure_high.at(0)
	                   && this->curr_pressure_high < critical_pressure_high.at(1);
	this->high_status_bar_marker.color.r = high_status ? 0 : 1;
	this->high_status_bar_marker.color.g = high_status ? 1 : 0;

	bool mid_status =
	    this->curr_pressure_mid > critical_pressure_mid.at(0) && this->curr_pressure_mid < critical_pressure_mid.at(1);
	this->mid_status_bar_marker.color.r = mid_status ? 0 : 1;
	this->mid_status_bar_marker.color.g = mid_status ? 1 : 0;

	bool low_status =
	    this->curr_pressure_low > critical_pressure_low.at(0) && this->curr_pressure_low < critical_pressure_low.at(1);
	this->low_status_bar_marker.color.r = low_status ? 0 : 1;
	this->low_status_bar_marker.color.g = low_status ? 1 : 0;

	// Empty the pressure bar marker array
	this->pressure_bar_array.markers.clear();
	// Populate the pressure bar marker array
	this->pressure_bar_array.markers.push_back(this->high_status_bar_marker);
	this->pressure_bar_array.markers.push_back(this->high_status_bar_bg_marker);
	this->pressure_bar_array.markers.push_back(this->mid_status_bar_marker);
	this->pressure_bar_array.markers.push_back(this->mid_status_bar_bg_marker);
	this->pressure_bar_array.markers.push_back(this->low_status_bar_marker);
	this->pressure_bar_array.markers.push_back(this->low_status_bar_bg_marker);

	// Publish markers
	this->marker_text_pub->publish(info_text_marker);
	this->marker_pressure_bar_pub->publish(this->pressure_bar_array);
}

int main(int argc, char const* argv[])
{
	if (!rclcpp::ok())
	{
		rclcpp::init(argc, argv);
	}

	rclcpp::spin(std::make_shared<VisualizationPublisher>());
	rclcpp::shutdown();
	return 0;
}
