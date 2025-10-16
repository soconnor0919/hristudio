/**
 * NAO6 ROS2 Transform Functions
 *
 * This module provides transform functions for converting HRIStudio action parameters
 * to ROS2 message formats for the NAO6 robot via naoqi_driver2.
 */

/**
 * Transform velocity parameters to geometry_msgs/msg/Twist
 */
export function transformToTwist(
  params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    linear: {
      x: params.linear ?? 0.0,
      y: 0.0,
      z: 0.0,
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: params.angular ?? 0.0,
    },
  };
}

/**
 * Transform text to std_msgs/msg/String
 */
export function transformToStringMessage(
  params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    data: params.text ?? "",
  };
}

/**
 * Transform joint parameters to naoqi_bridge_msgs/msg/JointAnglesWithSpeed
 */
export function transformToJointAngles(
  params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    joint_names: [params.joint_name as string],
    joint_angles: [params.angle as number],
    speed: params.speed ?? 0.2,
  };
}

/**
 * Transform head movement parameters to naoqi_bridge_msgs/msg/JointAnglesWithSpeed
 */
export function transformToHeadMovement(
  params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    joint_names: ["HeadYaw", "HeadPitch"],
    joint_angles: [params.yaw as number, params.pitch as number],
    speed: params.speed ?? 0.3,
  };
}

/**
 * Get camera image - returns subscription request
 */
export function getCameraImage(
  params: Record<string, unknown>,
): Record<string, unknown> {
  const camera = params.camera as string;
  const topic =
    camera === "front" ? "/camera/front/image_raw" : "/camera/bottom/image_raw";

  return {
    subscribe: true,
    topic,
    messageType: "sensor_msgs/msg/Image",
    once: true,
  };
}

/**
 * Get joint states - returns subscription request
 */
export function getJointStates(
  _params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    subscribe: true,
    topic: "/joint_states",
    messageType: "sensor_msgs/msg/JointState",
    once: true,
  };
}

/**
 * Get IMU data - returns subscription request
 */
export function getImuData(
  _params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    subscribe: true,
    topic: "/imu/torso",
    messageType: "sensor_msgs/msg/Imu",
    once: true,
  };
}

/**
 * Get bumper status - returns subscription request
 */
export function getBumperStatus(
  _params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    subscribe: true,
    topic: "/bumper",
    messageType: "naoqi_bridge_msgs/msg/Bumper",
    once: true,
  };
}

/**
 * Get touch sensors - returns subscription request
 */
export function getTouchSensors(
  params: Record<string, unknown>,
): Record<string, unknown> {
  const sensorType = params.sensor_type as string;
  const topic = sensorType === "hand" ? "/hand_touch" : "/head_touch";
  const messageType =
    sensorType === "hand"
      ? "naoqi_bridge_msgs/msg/HandTouch"
      : "naoqi_bridge_msgs/msg/HeadTouch";

  return {
    subscribe: true,
    topic,
    messageType,
    once: true,
  };
}

/**
 * Get sonar range - returns subscription request
 */
export function getSonarRange(
  params: Record<string, unknown>,
): Record<string, unknown> {
  const sensor = params.sensor as string;
  let topic: string;

  if (sensor === "left") {
    topic = "/sonar/left";
  } else if (sensor === "right") {
    topic = "/sonar/right";
  } else {
    // For "both", we'll default to left and let the wizard interface handle multiple calls
    topic = "/sonar/left";
  }

  return {
    subscribe: true,
    topic,
    messageType: "sensor_msgs/msg/Range",
    once: true,
  };
}

/**
 * Get robot info - returns subscription request
 */
export function getRobotInfo(
  _params: Record<string, unknown>,
): Record<string, unknown> {
  return {
    subscribe: true,
    topic: "/info",
    messageType: "naoqi_bridge_msgs/msg/RobotInfo",
    once: true,
  };
}

/**
 * NAO6-specific joint limits for safety
 */
export const NAO6_JOINT_LIMITS = {
  HeadYaw: { min: -2.0857, max: 2.0857 },
  HeadPitch: { min: -0.672, max: 0.5149 },
  LShoulderPitch: { min: -2.0857, max: 2.0857 },
  LShoulderRoll: { min: -0.3142, max: 1.3265 },
  LElbowYaw: { min: -2.0857, max: 2.0857 },
  LElbowRoll: { min: -1.5446, max: -0.0349 },
  LWristYaw: { min: -1.8238, max: 1.8238 },
  RShoulderPitch: { min: -2.0857, max: 2.0857 },
  RShoulderRoll: { min: -1.3265, max: 0.3142 },
  RElbowYaw: { min: -2.0857, max: 2.0857 },
  RElbowRoll: { min: 0.0349, max: 1.5446 },
  RWristYaw: { min: -1.8238, max: 1.8238 },
  LHipYawPitch: { min: -1.1453, max: 0.7408 },
  LHipRoll: { min: -0.3793, max: 0.79 },
  LHipPitch: { min: -1.7732, max: 0.484 },
  LKneePitch: { min: -0.0923, max: 2.1121 },
  LAnklePitch: { min: -1.1894, max: 0.9228 },
  LAnkleRoll: { min: -0.3976, max: 0.769 },
  RHipRoll: { min: -0.79, max: 0.3793 },
  RHipPitch: { min: -1.7732, max: 0.484 },
  RKneePitch: { min: -0.0923, max: 2.1121 },
  RAnklePitch: { min: -1.1894, max: 0.9228 },
  RAnkleRoll: { min: -0.769, max: 0.3976 },
};

/**
 * Validate joint angle against NAO6 limits
 */
export function validateJointAngle(jointName: string, angle: number): boolean {
  const limits = NAO6_JOINT_LIMITS[jointName as keyof typeof NAO6_JOINT_LIMITS];
  if (!limits) {
    console.warn(`Unknown joint: ${jointName}`);
    return false;
  }

  return angle >= limits.min && angle <= limits.max;
}

/**
 * Clamp joint angle to NAO6 limits
 */
export function clampJointAngle(jointName: string, angle: number): number {
  const limits = NAO6_JOINT_LIMITS[jointName as keyof typeof NAO6_JOINT_LIMITS];
  if (!limits) {
    console.warn(`Unknown joint: ${jointName}, returning 0`);
    return 0;
  }

  return Math.max(limits.min, Math.min(limits.max, angle));
}

/**
 * NAO6 velocity limits for safety
 */
export const NAO6_VELOCITY_LIMITS = {
  linear: { min: -0.55, max: 0.55 },
  angular: { min: -2.0, max: 2.0 },
};

/**
 * Validate velocity against NAO6 limits
 */
export function validateVelocity(linear: number, angular: number): boolean {
  return (
    linear >= NAO6_VELOCITY_LIMITS.linear.min &&
    linear <= NAO6_VELOCITY_LIMITS.linear.max &&
    angular >= NAO6_VELOCITY_LIMITS.angular.min &&
    angular <= NAO6_VELOCITY_LIMITS.angular.max
  );
}

/**
 * Clamp velocity to NAO6 limits
 */
export function clampVelocity(
  linear: number,
  angular: number,
): { linear: number; angular: number } {
  return {
    linear: Math.max(
      NAO6_VELOCITY_LIMITS.linear.min,
      Math.min(NAO6_VELOCITY_LIMITS.linear.max, linear),
    ),
    angular: Math.max(
      NAO6_VELOCITY_LIMITS.angular.min,
      Math.min(NAO6_VELOCITY_LIMITS.angular.max, angular),
    ),
  };
}

/**
 * Convert degrees to radians (helper for UI)
 */
export function degreesToRadians(degrees: number): number {
  return degrees * (Math.PI / 180);
}

/**
 * Convert radians to degrees (helper for UI)
 */
export function radiansToDegrees(radians: number): number {
  return radians * (180 / Math.PI);
}

/**
 * Transform function registry for dynamic lookup
 */
export const NAO6_TRANSFORM_FUNCTIONS = {
  transformToTwist,
  transformToStringMessage,
  transformToJointAngles,
  transformToHeadMovement,
  getCameraImage,
  getJointStates,
  getImuData,
  getBumperStatus,
  getTouchSensors,
  getSonarRange,
  getRobotInfo,
} as const;

/**
 * Get transform function by name
 */
export function getTransformFunction(
  name: string,
): ((params: Record<string, unknown>) => Record<string, unknown>) | null {
  return (
    NAO6_TRANSFORM_FUNCTIONS[name as keyof typeof NAO6_TRANSFORM_FUNCTIONS] ||
    null
  );
}
