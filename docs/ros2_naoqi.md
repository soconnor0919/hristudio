🤖 NAO6 — ROS 2 Humble Topics (via naoqi_driver2)
🏃 Motion & Odometry
Topic	Message Type	Description
/cmd_vel	geometry_msgs/msg/Twist	Command linear and angular base velocities (walking).
/odom	nav_msgs/msg/Odometry	Estimated robot position and velocity.
/move_base_simple/goal	geometry_msgs/msg/PoseStamped	Send goal poses for autonomous navigation.
🔩 Joints & Robot State
Topic	Message Type	Description
/joint_states	sensor_msgs/msg/JointState	Standard ROS joint angles, velocities, efforts.
/joint_angles	naoqi_bridge_msgs/msg/JointAnglesWithSpeed	NAO-specific joint control interface.
/info	naoqi_bridge_msgs/msg/RobotInfo	General robot info (model, battery, language, etc.).
🎥 Cameras
Topic	Message Type	Description
/camera/front/image_raw	sensor_msgs/msg/Image	Front (head) camera image stream.
/camera/front/camera_info	sensor_msgs/msg/CameraInfo	Intrinsics for front camera.
/camera/bottom/image_raw	sensor_msgs/msg/Image	Bottom (mouth) camera image stream.
/camera/bottom/camera_info	sensor_msgs/msg/CameraInfo	Intrinsics for bottom camera.
🦶 Sensors
Topic	Message Type	Description
/imu/torso	sensor_msgs/msg/Imu	Torso inertial measurement data.
/bumper	naoqi_bridge_msgs/msg/Bumper	Foot bumper contact sensors.
/hand_touch	naoqi_bridge_msgs/msg/HandTouch	Hand tactile sensors.
/head_touch	naoqi_bridge_msgs/msg/HeadTouch	Head tactile sensors.
/sonar/left	sensor_msgs/msg/Range	Left ultrasonic range sensor.
/sonar/right	sensor_msgs/msg/Range	Right ultrasonic range sensor.
🔊 Audio & Speech
Topic	Message Type	Description
/audio	audio_common_msgs/msg/AudioData	Raw audio input from NAO’s microphones.
/speech	std_msgs/msg/String	Send text-to-speech commands.
🧠 System & Diagnostics
Topic	Message Type	Description
/diagnostics	diagnostic_msgs/msg/DiagnosticArray	Hardware and driver status.
/robot_description	std_msgs/msg/String	URDF description of the robot.
/tf	tf2_msgs/msg/TFMessage	Coordinate transforms between frames.
/parameter_events	rcl_interfaces/msg/ParameterEvent	Parameter change notifications.
/rosout	rcl_interfaces/msg/Log	Logging output.
✅ ROS 2 bridge status: Active
✅ Robot model detected: NAO V6 (NAOqi 2.8.7.4)
✅ Driver: naoqi_driver2
✅ System confirmed working: motion, speech, camera, IMU, touch, sonar
