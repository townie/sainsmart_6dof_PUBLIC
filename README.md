# Chefy!


----URDF stuff:

Generate URDF files
rosrun xacro xacro.py -o chefy_arm.urdf chefy_arm.xacro

Check the URDF file
check_urdf chefy_arm.urdf

Displays the URDF file
roslaunch urdf_tutorial display.launch model:='$(find chefy_arm_description)/urdf/chefy_arm.urdf'

Start the moveit assistant
rosrun moveit_setup_assistant moveit_setup_assistant


------ Launch Start
Start gazebo simulation environment:
roslaunch chefy_arm_gazebo chefy_arm_bringup.launch

Start moveit Demo:
roslaunch chefy_arm_moveit_config demo.launch

Start the grasp builder
roslaunch chefy_arm_gazebo grasp_generator_server.launch



------ Topics 
View Camera Image:
rosrun image_view image_view image:=/camera/image_raw

View topic list:
rostopic list