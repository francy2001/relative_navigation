import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)


def generate_launch_description():
    # Declare launch arguments
    declare_run_in_sim_cmd = DeclareLaunchArgument(
        name="run_in_sim",
        default_value="False",
        description="Use the provided Gazebo simulation if true. Default is false.",
    )
    run_in_sim = LaunchConfiguration("run_in_sim")

    declare_run_gazebo_gui_cmd = DeclareLaunchArgument(
        name="sim_gui",
        default_value="False",
        description='Run gazebo with gui (not headless) if true. Ignored if "run_in_sim" is false.',
    )
    run_gazebo_gui = LaunchConfiguration("sim_gui")

    declare_use_even_floor_cmd = DeclareLaunchArgument(
        name="use_even_floor",
        default_value="False",
        description='Run gazebo with even floor if true. Ignored if "run_in_sim" is false.',
    )
    use_even_floor = LaunchConfiguration("use_even_floor")

    declare_run_with_control_cmd = DeclareLaunchArgument(
        name="run_with_control",
        default_value="True",
        description="Use the provided Gazebo simulation if true. Default is true.",
    )
    run_with_control = LaunchConfiguration("run_with_control")

    declare_controller_cmd = DeclareLaunchArgument(
        name="controller",
        default_value="tvlqr",
        description="Which controller to use, if run_with_control true. The value has to be the name of the launch file in reacsa_control. Default is 'tvlqr'.",
    )
    controller = LaunchConfiguration("controller")

    declare_observer_cmd = DeclareLaunchArgument(
        name="observer",
        default_value="extended_kalman_filter",
        description="Which observer to use. The value has to be the name of the launch file in reacsa_control. Default is 'extended_kalman_filter'.",
    )
    observer = LaunchConfiguration("observer")

    declare_record_rosbag_cmd = DeclareLaunchArgument(
        name="record_rosbag",
        default_value="False",
        description="Record all topics to rosbag if true. Default is false.",
    )
    record_rosbag = LaunchConfiguration("record_rosbag")

    declare_create_report_cmd = DeclareLaunchArgument(
        name="create_report",
        default_value="True",
        description="Create a pdf report of all executed trajectories if true. Default is true.",
    )
    create_report = LaunchConfiguration("create_report")

    declare_save_csv_cmd = DeclareLaunchArgument(
        name="save_csv",
        default_value="False",
        description="Save report data to csv files if true. Saves data independently of 'create_report' value. Default is false.",
    )
    save_csv = LaunchConfiguration("save_csv")

    declare_save_npz_cmd = DeclareLaunchArgument(
        name="save_npz",
        default_value="False",
        description="Save report data to a npz file if true. Saves data independently of 'create_report' value. Default is false.",
    )
    save_npz = LaunchConfiguration("save_npz")

    declare_use_joystick_cmd = DeclareLaunchArgument(
        name="use_joystick",
        default_value="False",
        description="Launch the joy linux node to capture joystick inputs if true. To be used when a gamepad is available. Default is false.",
    )
    use_joystick = LaunchConfiguration("use_joystick")

    declare_relative_navigation_cmd = DeclareLaunchArgument(
        name="relative_navigation",
        default_value="True",
        description="Launch the relative navigation node to capture lidar datas. Default is True.",
    )
    relative_navigation = LaunchConfiguration("relative_navigation")

    # Specify paths to reacsa packages
    pkg_reacsa_control = get_package_share_directory("reacsa_control")
    pkg_reacsa_description = get_package_share_directory("reacsa_description")
    pkg_reacsa_input = get_package_share_directory("reacsa_input")
    pkg_reacsa_observer = get_package_share_directory("reacsa_observer")
    pkg_orbit_simulation = get_package_share_directory("orbit_simulation")
    pkg_reacsa_visualization = get_package_share_directory("reacsa_visualization")
    pkg_reacsa_report = get_package_share_directory("reacsa_report")
    pkg_reacsa_relative_navigation = get_package_share_directory("reacsa_relative_navigation")

    # Generate nodes from package launch files
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_reacsa_control, "launch", PythonExpression(expression=["'", controller, "'", " + '.launch.py'"])]
            )
        ),
        condition=IfCondition(run_with_control),
        launch_arguments={"use_sim_time": run_in_sim}.items(),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_reacsa_description, "launch", "robot_description.launch.py"])]
        ),
        launch_arguments=[("use_sim_time", LaunchConfiguration("use_sim_time"))],
    )

    input_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_reacsa_input, "launch", "reacsa.launch.py")),
        launch_arguments={"use_sim_time": run_in_sim, "use_joystick": use_joystick}.items(),
    )

    observer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_reacsa_observer, "launch", PythonExpression(expression=["'", observer, "'", " + '.launch.py'"])]
            )
        ),
        launch_arguments={"use_sim_time": run_in_sim}.items(),
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_orbit_simulation, "launch", "reacsa.launch.py")),
        condition=IfCondition(run_in_sim),
        launch_arguments={
            "sim_gui": run_gazebo_gui,
            "use_even_floor": use_even_floor,
        }.items(),
    )

    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_reacsa_visualization, "launch", "reacsa.launch.py")),
        launch_arguments={"use_sim_time": run_in_sim}.items(),
    )

    report_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_reacsa_report, "launch", "reacsa.launch.py")),
        launch_arguments={
            "use_sim_time": run_in_sim,
            "use_even_floor": use_even_floor,
            "create_report": create_report,
            "save_csv": save_csv,
            "save_npz": save_npz,
        }.items(),
    )

    relative_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_reacsa_relative_navigation, "launch", "relative_navigation.launch.py")),
        condition=IfCondition(relative_navigation),
        launch_arguments={"use_sim_time": run_in_sim}.items(),
    )

    # Generate node to record topics to rosbag
    record_rosbag_sim_time_exec = ExecuteProcess(
        condition=IfCondition(AndSubstitution(record_rosbag, run_in_sim)),
        cmd=["ros2", "bag", "record", "-a", "-b 1000000000", "--include-hidden-topics", "--use-sim-time"],
        output="screen",
    )
    record_rosbag_exec = ExecuteProcess(
        condition=IfCondition(AndSubstitution(record_rosbag, NotSubstitution(run_in_sim))),
        cmd=["ros2", "bag", "record", "-a", "-b 1000000000", "--include-hidden-topics"],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_run_in_sim_cmd,
            declare_run_gazebo_gui_cmd,
            declare_use_even_floor_cmd,
            declare_run_with_control_cmd,
            declare_controller_cmd,
            declare_observer_cmd,
            declare_record_rosbag_cmd,
            declare_create_report_cmd,
            declare_save_csv_cmd,
            declare_save_npz_cmd,
            declare_use_joystick_cmd,
            declare_relative_navigation_cmd,
            control_launch,
            description_launch,
            input_launch,
            observer_launch,
            simulation_launch,
            visualization_launch,
            report_launch,
            relative_navigation_launch,
            record_rosbag_sim_time_exec,
            record_rosbag_exec,
        ]
    )
