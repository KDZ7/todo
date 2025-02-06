from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from relaymd.relay import Relay


def setEnvironment(key, value):
    os.environ[key] = str(value)
def addEnvironment(key, value):
    os.environ[key] = os.environ.get(key, "") + os.pathsep + str(value)

def bindContext(context):
    if "pkg_robot_description" in Relay.dict and Relay.dict["pkg_robot_description"] is not None:
        Relay.dict["robot_description"] = Command(["xacro ", PathJoinSubstitution([FindPackageShare(Relay.dict["pkg_robot_description"]), Relay.dict["robot_description"]])])
    else:
        Relay.dict["robot_description"] = Command(["xacro ", Relay.dict["robot_description"]])
        
    if "gazebo" in Relay.dict and "pkg_resource_path" in Relay.dict["gazebo"] and Relay.dict["gazebo"]["pkg_resource_path"] is not None:
        Relay.dict["gazebo"]["resource_path"] = PathJoinSubstitution([FindPackageShare(Relay.dict["gazebo"]["pkg_resource_path"]), Relay.dict["gazebo"]["resource_path"]])
        Relay.dict["gazebo"]["plugin_path"] = PathJoinSubstitution([FindPackageShare(Relay.dict["gazebo"]["pkg_resource_path"]), Relay.dict["gazebo"]["plugin_path"]])
        Relay.dict["gazebo"]["library_path"] = PathJoinSubstitution([FindPackageShare(Relay.dict["gazebo"]["pkg_resource_path"]), Relay.dict["gazebo"]["library_path"]])

def exec(context):
    tasks = []
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=Relay.dict["namespace"],
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Relay.dict["robot_description"]},
            {"use_sim_time": Relay.dict["use_sim_time"]}
        ]
    )
    tasks.append(robot_state_publisher)

    if Relay.dict["display_on"] != "gazebo":
        joint_state_publisher_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            namespace=Relay.dict["namespace"],
            name="joint_state_publisher_gui",
            output="screen"
        )
        tasks.append(joint_state_publisher_gui)

    if "rviz" in Relay.dict and Relay.dict["display_on"] == "rviz" or Relay.dict["display_on"] == "both":
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            namespace=Relay.dict["namespace"],
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("relaymd"), "config", Relay.dict["rviz"]["config"]])]
        )
        tasks.append(rviz)

    if "gazebo" in Relay.dict and Relay.dict["display_on"] == "gazebo" or Relay.dict["display_on"] == "both":
        if Relay.dict["gazebo"]["mode"] == "server" or Relay.dict["gazebo"]["mode"] == "both":
            gazebo_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
                launch_arguments={
                    "gz_args": [
                        " -s -v ", 
                        " -r " if Relay.dict["gazebo"]["autorun"] else "",
                        PathJoinSubstitution([Relay.dict["gazebo"]["resource_path"], Relay.dict["gazebo"]["world"]])
                    ],
                    "on_exit_shutdown": "True",
                }.items()
            )
            tasks.append(gazebo_server)

        if Relay.dict["gazebo"]["mode"] == "client" or Relay.dict["gazebo"]["mode"] == "both":
            spawn_robot = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=Relay.dict["namespace"],
                name=Relay.dict["gazebo"]["robot_name"],
                arguments = [
                    "-name", Relay.dict["gazebo"]["robot_name"],
                    "-string", Relay.dict["robot_description"],
                    "-x", str(Relay.dict["gazebo"]["robot_position"][0]),
                    "-y", str(Relay.dict["gazebo"]["robot_position"][1]),
                    "-z", str(Relay.dict["gazebo"]["robot_position"][2]),
                    "-R", str(Relay.dict["gazebo"]["robot_position"][3]),
                    "-P", str(Relay.dict["gazebo"]["robot_position"][4]),
                    "-Y", str(Relay.dict["gazebo"]["robot_position"][5]),
                ],
                output="screen"
            )
            tasks.append(spawn_robot)
            
            gazebo_client = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
                launch_arguments={
                    "gz_args": "-g",
                    "on_exit_shutdown": "True",
                }.items()
            )
            tasks.append(gazebo_client)

    return tasks

def generate_launch_description():
    cfgrelay = LaunchConfiguration("relay")
    _cfgrelay = DeclareLaunchArgument(
        "relay",
        default_value=PathJoinSubstitution([FindPackageShare("relaymd"), "config", "relay.yaml"]),
        description="""
        ros_domain_id: [ROS domain id] (default: 0)
        namespace: [namespace] (default: Null)
        pkg_robot_description: [package name of robot description file] (default: Null)
        robot_description: [path to robot description file]
        display_on: [rviz/gazebo/both]
        use_sim_time: [use simulation time] (default: False)
        rviz:
            config: [rviz config file] (default: rviz.yaml)
        gazebo:
            mode: [client/server/both]
            pkg_resource_path: [package name of gazebo models] (default: Null)
            resource_path: [path to gazebo models]
            plugin_path: [path to gazebo plugins]
            library_path: [path to gazebo libraries] 
            world: [model file of world] (default: empty.world.sdf)
            robot_name: [name of robot]
            robot_position: [initial position of robot: [x, y, z, roll, pitch, yaw]]
            autorun: [run the simulation automatically] (default: False)
        """
    )
    actions = []
    actions.append(_cfgrelay) 
    actions.append(Relay.read(cfgrelay))
    actions.append(Relay.action(bindContext))
    actions.append(Relay.action(lambda _: setEnvironment("ROS_DOMAIN_ID", Relay.dict["ros_domain_id"])))
    actions.append(Relay.action(lambda _: addEnvironment("GZ_SIM_RESOURCE_PATH", Relay.dict["gazebo"]["resource_path"])))
    actions.append(Relay.action(lambda _: addEnvironment("GZ_SIM_SYSTEM_PLUGIN_PATH", Relay.dict["gazebo"]["plugin_path"])))
    actions.append(Relay.action(lambda _: addEnvironment("LD_LIBRARY_PATH", Relay.dict["gazebo"]["library_path"])))
    actions.append(ExecuteProcess(cmd=["bash", "-c", "printenv | grep -E 'ROS_DOMAIN_ID|GZ_SIM_RESOURCE_PATH|GZ_SIM_SYSTEM_PLUGIN_PATH|LD_LIBRARY_PATH|LD_LIBRARY_PATH|MESA_D3D12_DEFAULT_ADAPTER_NAME'"], output="screen"))
    actions.append(Relay.action(exec))
    return LaunchDescription([_cfgrelay, *actions])
    
