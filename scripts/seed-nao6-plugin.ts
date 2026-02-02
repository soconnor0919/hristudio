#!/usr/bin/env bun

/**
 * Seed NAO6 Plugin into HRIStudio Database
 *
 * This script adds the NAO6 ROS2 integration plugin to the HRIStudio database,
 * including the plugin repository and plugin definition with all available actions.
 */

import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import { env } from "~/env";
import {
  plugins,
  pluginRepositories,
  robots,
  users,
  type InsertPlugin,
  type InsertPluginRepository,
  type InsertRobot,
} from "~/server/db/schema";
import { eq } from "drizzle-orm";

const connectionString = env.DATABASE_URL;
const client = postgres(connectionString);
const db = drizzle(client);

async function seedNAO6Plugin() {
  console.log("🤖 Seeding NAO6 plugin into HRIStudio database...");

  try {
    // 0. Get system user for created_by fields
    console.log("👤 Getting system user...");

    const systemUser = await db
      .select()
      .from(users)
      .where(eq(users.email, "sean@soconnor.dev"))
      .limit(1);

    if (systemUser.length === 0) {
      throw new Error(
        "System user not found. Please run 'bun db:seed' first to create initial users.",
      );
    }

    const userId = systemUser[0]!.id;
    console.log(`✅ Found system user: ${userId}`);

    // 1. Create or update NAO6 robot entry
    console.log("📋 Creating NAO6 robot entry...");

    const existingRobot = await db
      .select()
      .from(robots)
      .where(eq(robots.name, "NAO6"))
      .limit(1);

    let robotId: string;

    if (existingRobot.length > 0) {
      robotId = existingRobot[0]!.id;
      console.log(`✅ Found existing NAO6 robot: ${robotId}`);
    } else {
      const newRobots = await db
        .insert(robots)
        .values({
          name: "NAO6",
          manufacturer: "SoftBank Robotics",
          model: "NAO V6.0",
          description:
            "Humanoid robot designed for education, research, and human-robot interaction studies. Features bipedal walking, speech synthesis, cameras, and comprehensive sensor suite.",
          capabilities: [
            "bipedal_walking",
            "speech_synthesis",
            "head_movement",
            "arm_gestures",
            "touch_sensors",
            "visual_sensors",
            "audio_sensors",
            "posture_control",
            "balance_control",
          ],
          communicationProtocol: "ros2",
        } satisfies InsertRobot)
        .returning();

      robotId = newRobots[0]!.id;
      console.log(`✅ Created NAO6 robot: ${robotId}`);
    }

    // 2. Create or update plugin repository
    console.log("📦 Creating NAO6 plugin repository...");

    const existingRepo = await db
      .select()
      .from(pluginRepositories)
      .where(eq(pluginRepositories.name, "NAO6 ROS2 Integration Repository"))
      .limit(1);

    let repoId: string;

    if (existingRepo.length > 0) {
      repoId = existingRepo[0]!.id;
      console.log(`✅ Found existing repository: ${repoId}`);
    } else {
      const newRepos = await db
        .insert(pluginRepositories)
        .values({
          name: "NAO6 ROS2 Integration Repository",
          url: "https://github.com/hristudio/nao6-ros2-plugins",
          description:
            "Official NAO6 robot plugins for ROS2-based Human-Robot Interaction experiments",
          trustLevel: "official",
          isEnabled: true,
          isOfficial: true,
          createdBy: userId,
          lastSyncAt: new Date(),
          metadata: {
            author: {
              name: "HRIStudio Team",
              email: "support@hristudio.com",
            },
            license: "MIT",
            ros2: {
              distro: "humble",
              packages: [
                "naoqi_driver2",
                "naoqi_bridge_msgs",
                "rosbridge_suite",
              ],
            },
            supportedRobots: ["NAO6"],
            categories: [
              "movement",
              "speech",
              "sensors",
              "interaction",
              "vision",
            ],
          },
        } satisfies InsertPluginRepository)
        .returning();

      repoId = newRepos[0]!.id;
      console.log(`✅ Created repository: ${repoId}`);
    }

    // 3. Create or update NAO6 plugin
    console.log("🔌 Creating NAO6 enhanced plugin...");

    const existingPlugin = await db
      .select()
      .from(plugins)
      .where(eq(plugins.name, "NAO6 Robot (Enhanced ROS2 Integration)"))
      .limit(1);

    const actionDefinitions = [
      {
        id: "nao_speak",
        name: "Speak Text",
        description:
          "Make the NAO robot speak the specified text using text-to-speech synthesis",
        category: "speech",
        icon: "volume2",
        parametersSchema: {
          type: "object",
          properties: {
            text: {
              type: "string",
              title: "Text to Speak",
              description: "The text that the robot should speak aloud",
              minLength: 1,
              maxLength: 500,
            },
            volume: {
              type: "number",
              title: "Volume",
              description: "Speech volume level (0.1 = quiet, 1.0 = loud)",
              default: 0.7,
              minimum: 0.1,
              maximum: 1.0,
              step: 0.1,
            },
            speed: {
              type: "number",
              title: "Speech Speed",
              description: "Speech rate multiplier (0.5 = slow, 2.0 = fast)",
              default: 1.0,
              minimum: 0.5,
              maximum: 2.0,
              step: 0.1,
            },
          },
          required: ["text"],
        },
        implementation: {
          type: "ros2_topic",
          topic: "/speech",
          messageType: "std_msgs/String",
        },
      },
      {
        id: "nao_move",
        name: "Move Robot",
        description:
          "Move the NAO robot with specified linear and angular velocities",
        category: "movement",
        icon: "move",
        parametersSchema: {
          type: "object",
          properties: {
            direction: {
              type: "string",
              title: "Movement Direction",
              description: "Predefined movement direction",
              enum: [
                "forward",
                "backward",
                "left",
                "right",
                "turn_left",
                "turn_right",
              ],
              default: "forward",
            },
            distance: {
              type: "number",
              title: "Distance (meters)",
              description: "Distance to move in meters",
              default: 0.1,
              minimum: 0.01,
              maximum: 2.0,
              step: 0.01,
            },
            speed: {
              type: "number",
              title: "Movement Speed",
              description: "Speed factor (0.1 = very slow, 1.0 = normal speed)",
              default: 0.5,
              minimum: 0.1,
              maximum: 1.0,
              step: 0.1,
            },
          },
          required: ["direction"],
        },
        implementation: {
          type: "ros2_topic",
          topic: "/cmd_vel",
          messageType: "geometry_msgs/Twist",
        },
      },
      {
        id: "nao_pose",
        name: "Set Posture",
        description: "Set the NAO robot to a specific posture or pose",
        category: "movement",
        icon: "user",
        parametersSchema: {
          type: "object",
          properties: {
            posture: {
              type: "string",
              title: "Posture",
              description: "Target posture for the robot",
              enum: ["Stand", "Sit", "SitRelax", "StandInit", "Crouch"],
              default: "Stand",
            },
            speed: {
              type: "number",
              title: "Movement Speed",
              description:
                "Speed of posture transition (0.1 = slow, 1.0 = fast)",
              default: 0.5,
              minimum: 0.1,
              maximum: 1.0,
              step: 0.1,
            },
          },
          required: ["posture"],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/robot_posture/go_to_posture",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
      {
        id: "nao_head_movement",
        name: "Move Head",
        description:
          "Control NAO robot head movement for gaze direction and attention",
        category: "movement",
        icon: "eye",
        parametersSchema: {
          type: "object",
          properties: {
            headYaw: {
              type: "number",
              title: "Head Yaw (degrees)",
              description:
                "Left/right head rotation (-90° = right, +90° = left)",
              default: 0.0,
              minimum: -90.0,
              maximum: 90.0,
              step: 1.0,
            },
            headPitch: {
              type: "number",
              title: "Head Pitch (degrees)",
              description: "Up/down head rotation (-25° = down, +25° = up)",
              default: 0.0,
              minimum: -25.0,
              maximum: 25.0,
              step: 1.0,
            },
            speed: {
              type: "number",
              title: "Movement Speed",
              description: "Speed of head movement (0.1 = slow, 1.0 = fast)",
              default: 0.3,
              minimum: 0.1,
              maximum: 1.0,
              step: 0.1,
            },
          },
          required: [],
        },
        implementation: {
          type: "ros2_topic",
          topic: "/joint_angles",
          messageType: "naoqi_bridge_msgs/JointAnglesWithSpeed",
        },
      },
      {
        id: "nao_gesture",
        name: "Perform Gesture",
        description:
          "Make NAO robot perform predefined gestures and animations",
        category: "interaction",
        icon: "hand",
        parametersSchema: {
          type: "object",
          properties: {
            gesture: {
              type: "string",
              title: "Gesture Type",
              description: "Select a predefined gesture or animation",
              enum: [
                "wave",
                "point_left",
                "point_right",
                "applause",
                "thumbs_up",
              ],
              default: "wave",
            },
            intensity: {
              type: "number",
              title: "Gesture Intensity",
              description:
                "Intensity of the gesture movement (0.5 = subtle, 1.0 = full)",
              default: 0.8,
              minimum: 0.3,
              maximum: 1.0,
              step: 0.1,
            },
          },
          required: ["gesture"],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/animation_player/run_animation",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
      {
        id: "nao_sensor_monitor",
        name: "Monitor Sensors",
        description: "Monitor NAO robot sensors for interaction detection",
        category: "sensors",
        icon: "activity",
        parametersSchema: {
          type: "object",
          properties: {
            sensorType: {
              type: "string",
              title: "Sensor Type",
              description: "Which sensors to monitor",
              enum: ["touch", "bumper", "sonar", "all"],
              default: "touch",
            },
            duration: {
              type: "number",
              title: "Duration (seconds)",
              description: "How long to monitor sensors",
              default: 10,
              minimum: 1,
              maximum: 300,
            },
          },
          required: ["sensorType"],
        },
        implementation: {
          type: "ros2_subscription",
          topics: [
            "/naoqi_driver/bumper",
            "/naoqi_driver/hand_touch",
            "/naoqi_driver/head_touch",
          ],
        },
      },
      {
        id: "nao_emergency_stop",
        name: "Emergency Stop",
        description: "Immediately stop all robot movement for safety",
        category: "safety",
        icon: "stop-circle",
        parametersSchema: {
          type: "object",
          properties: {
            stopType: {
              type: "string",
              title: "Stop Type",
              description: "Type of emergency stop",
              enum: ["movement", "all"],
              default: "all",
            },
          },
          required: [],
        },
        implementation: {
          type: "ros2_topic",
          topic: "/cmd_vel",
          messageType: "geometry_msgs/Twist",
        },
      },
      {
        id: "nao_wake_rest",
        name: "Wake Up / Rest Robot",
        description: "Wake up the robot or put it to rest position",
        category: "system",
        icon: "power",
        parametersSchema: {
          type: "object",
          properties: {
            action: {
              type: "string",
              title: "Action",
              description: "Wake up robot or put to rest",
              enum: ["wake", "rest"],
              default: "wake",
            },
          },
          required: ["action"],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/motion/wake_up",
          serviceType: "std_srvs/srv/Empty",
        },
      },
      {
        id: "nao_status_check",
        name: "Check Robot Status",
        description: "Get current robot status including battery and health",
        category: "system",
        icon: "info",
        parametersSchema: {
          type: "object",
          properties: {
            statusType: {
              type: "string",
              title: "Status Type",
              description: "What status information to retrieve",
              enum: ["basic", "battery", "sensors", "all"],
              default: "basic",
            },
          },
          required: ["statusType"],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/get_robot_config",
          serviceType: "naoqi_bridge_msgs/srv/GetRobotInfo",
        },
      },
      {
        id: "nao_nod",
        name: "Nod Head",
        description: "Make the robot nod its head (Yes)",
        category: "interaction",
        icon: "check-circle",
        parametersSchema: {
          type: "object",
          properties: {},
          required: [],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/animation_player/run_animation",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
      {
        id: "nao_shake_head",
        name: "Shake Head",
        description: "Make the robot shake its head (No)",
        category: "interaction",
        icon: "x-circle",
        parametersSchema: {
          type: "object",
          properties: {},
          required: [],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/animation_player/run_animation",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
      {
        id: "nao_bow",
        name: "Bow",
        description: "Make the robot bow",
        category: "interaction",
        icon: "user-check",
        parametersSchema: {
          type: "object",
          properties: {},
          required: [],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/animation_player/run_animation",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
      {
        id: "nao_open_hand",
        name: "Present (Open Hand)",
        description: "Make the robot gesture with an open hand",
        category: "interaction",
        icon: "hand",
        parametersSchema: {
          type: "object",
          properties: {
            hand: {
              type: "string",
              enum: ["left", "right", "both"],
              default: "right",
            },
          },
          required: ["hand"],
        },
        implementation: {
          type: "ros2_service",
          service: "/naoqi_driver/animation_player/run_animation",
          serviceType: "naoqi_bridge_msgs/srv/SetString",
        },
      },
    ];

    const pluginData: InsertPlugin = {
      robotId: robotId,
      name: "NAO6 Robot (Enhanced ROS2 Integration)",
      version: "2.0.0",
      description:
        "Comprehensive NAO6 robot integration for HRIStudio experiments via ROS2. Provides full robot control including movement, speech synthesis, posture control, sensor monitoring, and safety features.",
      author: "HRIStudio RoboLab Team",
      repositoryUrl: "https://github.com/hristudio/nao6-ros2-plugins",
      trustLevel: "official",
      status: "active",
      configurationSchema: {
        type: "object",
        properties: {
          robotIp: {
            type: "string",
            default: "nao.local",
            title: "Robot IP Address",
            description: "IP address or hostname of the NAO6 robot",
          },
          robotPassword: {
            type: "string",
            default: "robolab",
            title: "Robot Password",
            description: "Password for robot authentication",
            format: "password",
          },
          websocketUrl: {
            type: "string",
            default: "ws://localhost:9090",
            title: "WebSocket URL",
            description: "ROS bridge WebSocket URL for robot communication",
          },
          maxLinearVelocity: {
            type: "number",
            default: 0.2,
            minimum: 0.01,
            maximum: 0.5,
            title: "Max Linear Velocity (m/s)",
            description: "Maximum allowed linear movement speed for safety",
          },
          speechVolume: {
            type: "number",
            default: 0.7,
            minimum: 0.1,
            maximum: 1.0,
            title: "Speech Volume",
            description: "Default volume for speech synthesis",
          },
          enableSafetyMonitoring: {
            type: "boolean",
            default: true,
            title: "Enable Safety Monitoring",
            description:
              "Enable automatic safety monitoring and emergency stops",
          },
        },
        required: ["robotIp", "websocketUrl"],
      },
      actionDefinitions: actionDefinitions,
      metadata: {
        robotModel: "NAO V6.0",
        manufacturer: "SoftBank Robotics",
        naoqiVersion: "2.8.7.4",
        ros2Distro: "humble",
        launchPackage: "nao_launch",
        capabilities: [
          "bipedal_walking",
          "speech_synthesis",
          "head_movement",
          "arm_gestures",
          "touch_sensors",
          "visual_sensors",
          "posture_control",
        ],
        tags: [
          "nao6",
          "ros2",
          "speech",
          "movement",
          "sensors",
          "hri",
          "production",
        ],
      },
    };

    if (existingPlugin.length > 0) {
      await db
        .update(plugins)
        .set({
          ...pluginData,
          updatedAt: new Date(),
        })
        .where(eq(plugins.id, existingPlugin[0]!.id));

      console.log(`✅ Updated existing NAO6 plugin: ${existingPlugin[0]!.id}`);
    } else {
      const newPlugins = await db
        .insert(plugins)
        .values(pluginData)
        .returning();

      console.log(`✅ Created NAO6 plugin: ${newPlugins[0]!.id}`);
    }

    console.log("\n🎉 NAO6 plugin seeding completed successfully!");
    console.log("\nNext steps:");
    console.log("1. Install the plugin in a study via the HRIStudio interface");
    console.log("2. Configure the robot IP and WebSocket URL");
    console.log(
      "3. Launch ROS integration: ros2 launch nao_launch nao6_production.launch.py",
    );
    console.log("4. Test robot actions in the experiment designer");

    console.log("\n📊 Plugin Summary:");
    console.log(`   Robot: NAO6 (${robotId})`);
    console.log(`   Repository: NAO6 ROS2 Integration (${repoId})`);
    console.log(`   Actions: ${actionDefinitions.length} available`);
    console.log(
      "   Categories: speech, movement, interaction, sensors, safety, system",
    );
  } catch (error) {
    console.error("❌ Error seeding NAO6 plugin:", error);
    throw error;
  } finally {
    await client.end();
  }
}

// Run the seeding script
seedNAO6Plugin()
  .then(() => {
    console.log("✅ Database seeding completed");
    process.exit(0);
  })
  .catch((error) => {
    console.error("❌ Database seeding failed:", error);
    process.exit(1);
  });
