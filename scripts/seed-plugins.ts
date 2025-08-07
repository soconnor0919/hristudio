import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

const connectionString =
  process.env.DATABASE_URL ??
  "postgresql://postgres:password@localhost:5140/hristudio";
const client = postgres(connectionString);
const db = drizzle(client, { schema });

async function seedRobots() {
  console.log("🤖 Seeding robots...");

  // Check if robots already exist
  const existingRobots = await db.select().from(schema.robots);
  if (existingRobots.length > 0) {
    console.log(
      `⚠️  ${existingRobots.length} robots already exist, skipping robot seeding`,
    );
    return;
  }

  const robots = [
    {
      id: "31234567-89ab-cdef-0123-456789abcde1",
      name: "TurtleBot3 Burger",
      manufacturer: "ROBOTIS",
      model: "TurtleBot3 Burger",
      description:
        "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
      capabilities: [
        "differential_drive",
        "lidar",
        "imu",
        "odometry",
        "autonomous_navigation",
      ],
      communicationProtocol: "ros2" as const,
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-01T00:00:00"),
    },
    {
      id: "31234567-89ab-cdef-0123-456789abcde2",
      name: "NAO Humanoid Robot",
      manufacturer: "SoftBank Robotics",
      model: "NAO v6",
      description:
        "Autonomous, programmable humanoid robot designed for education, research, and human-robot interaction studies",
      capabilities: [
        "bipedal_walking",
        "speech_synthesis",
        "speech_recognition",
        "computer_vision",
        "gestures",
        "led_control",
        "touch_sensors",
      ],
      communicationProtocol: "custom" as const,
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-01T00:00:00"),
    },
    {
      id: "31234567-89ab-cdef-0123-456789abcde3",
      name: "TurtleBot3 Waffle Pi",
      manufacturer: "ROBOTIS",
      model: "TurtleBot3 Waffle Pi",
      description:
        "Extended TurtleBot3 platform with additional sensors and computing power for advanced research applications",
      capabilities: [
        "differential_drive",
        "lidar",
        "imu",
        "odometry",
        "camera",
        "manipulation",
        "autonomous_navigation",
      ],
      communicationProtocol: "ros2" as const,
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-01T00:00:00"),
    },
  ];

  await db.insert(schema.robots).values(robots);
  console.log(`✅ Created ${robots.length} robots`);
}

async function seedPluginRepositories() {
  console.log("📦 Seeding plugin repositories...");

  // Check if repositories already exist
  const existingRepos = await db.select().from(schema.pluginRepositories);
  if (existingRepos.length > 0) {
    console.log(
      `⚠️  ${existingRepos.length} plugin repositories already exist, skipping`,
    );
    return;
  }

  // Get the first user to use as creator
  const users = await db.select().from(schema.users);
  const adminUser =
    users.find((u) => u.email?.includes("sean@soconnor.dev")) ?? users[0];

  if (!adminUser) {
    console.log("⚠️  No users found. Please run basic seeding first.");
    return;
  }

  const repositories = [
    {
      id: "41234567-89ab-cdef-0123-456789abcde1",
      name: "HRIStudio Official Robot Plugins",
      url: "https://repo.hristudio.com",
      description:
        "Official collection of robot plugins maintained by the HRIStudio team",
      trustLevel: "official" as const,
      isEnabled: true,
      isOfficial: true,
      lastSyncAt: new Date("2024-01-10T12:00:00"),
      syncStatus: "completed" as const,
      syncError: null,
      metadata: {
        apiVersion: "1.0",
        pluginApiVersion: "1.0",
        categories: [
          "mobile-robots",
          "humanoid-robots",
          "manipulators",
          "drones",
        ],
        compatibility: {
          hristudio: { min: "0.1.0", recommended: "0.1.0" },
          ros2: { distributions: ["humble", "iron"], recommended: "iron" },
        },
      },
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-10T12:00:00"),
      createdBy: adminUser.id,
    },
  ];

  await db.insert(schema.pluginRepositories).values(repositories);
  console.log(`✅ Created ${repositories.length} plugin repositories`);
}

async function seedPlugins() {
  console.log("🔌 Seeding robot plugins...");

  // Check if plugins already exist
  const existingPlugins = await db.select().from(schema.plugins);
  if (existingPlugins.length > 0) {
    console.log(
      `⚠️  ${existingPlugins.length} plugins already exist, skipping plugin seeding`,
    );
    return;
  }

  const plugins = [
    {
      id: "51234567-89ab-cdef-0123-456789abcde1",
      robotId: "31234567-89ab-cdef-0123-456789abcde1",
      name: "TurtleBot3 Burger",
      version: "2.0.0",
      description:
        "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
      author: "ROBOTIS",
      repositoryUrl: "https://repo.hristudio.com",
      trustLevel: "official" as const,
      status: "active" as const,
      configurationSchema: {
        type: "object",
        properties: {
          namespace: { type: "string", default: "turtlebot3" },
          topics: {
            type: "object",
            properties: {
              cmd_vel: { type: "string", default: "/cmd_vel" },
              odom: { type: "string", default: "/odom" },
              scan: { type: "string", default: "/scan" },
            },
          },
        },
      },
      actionDefinitions: [
        {
          id: "move_velocity",
          name: "Set Velocity",
          description: "Control the robot's linear and angular velocity",
          category: "movement",
          icon: "navigation",
          timeout: 30000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              linear: {
                type: "number",
                minimum: -0.22,
                maximum: 0.22,
                default: 0,
                description: "Forward/backward velocity in m/s",
              },
              angular: {
                type: "number",
                minimum: -2.84,
                maximum: 2.84,
                default: 0,
                description: "Rotational velocity in rad/s",
              },
            },
            required: ["linear", "angular"],
          },
          ros2: {
            messageType: "geometry_msgs/msg/Twist",
            topic: "/cmd_vel",
            payloadMapping: {
              type: "transform",
              transformFn: "transformToTwist",
            },
          },
        },
        {
          id: "move_to_pose",
          name: "Navigate to Position",
          description:
            "Navigate to a specific position on the map using autonomous navigation",
          category: "movement",
          icon: "target",
          timeout: 120000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              x: {
                type: "number",
                default: 0,
                description: "X coordinate in meters",
              },
              y: {
                type: "number",
                default: 0,
                description: "Y coordinate in meters",
              },
              theta: {
                type: "number",
                default: 0,
                description: "Final orientation in radians",
              },
            },
            required: ["x", "y", "theta"],
          },
          ros2: {
            messageType: "geometry_msgs/msg/PoseStamped",
            action: "/navigate_to_pose",
            payloadMapping: {
              type: "transform",
              transformFn: "transformToPoseStamped",
            },
          },
        },
        {
          id: "stop_robot",
          name: "Stop Robot",
          description: "Immediately stop all robot movement",
          category: "movement",
          icon: "square",
          timeout: 5000,
          retryable: false,
          parameterSchema: {
            type: "object",
            properties: {},
            required: [],
          },
          ros2: {
            messageType: "geometry_msgs/msg/Twist",
            topic: "/cmd_vel",
            payloadMapping: {
              type: "static",
              payload: {
                linear: { x: 0.0, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 },
              },
            },
          },
        },
      ],
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-10T12:00:00"),
    },
    {
      id: "51234567-89ab-cdef-0123-456789abcde2",
      robotId: "31234567-89ab-cdef-0123-456789abcde2",
      name: "NAO Humanoid Robot",
      version: "1.0.0",
      description:
        "Autonomous, programmable humanoid robot designed for education, research, and human-robot interaction studies",
      author: "SoftBank Robotics",
      repositoryUrl: "https://repo.hristudio.com",
      trustLevel: "verified" as const,
      status: "active" as const,
      configurationSchema: {
        type: "object",
        properties: {
          ip: { type: "string", default: "nao.local" },
          port: { type: "number", default: 9559 },
          modules: {
            type: "array",
            default: [
              "ALMotion",
              "ALTextToSpeech",
              "ALAnimationPlayer",
              "ALLeds",
            ],
          },
        },
      },
      actionDefinitions: [
        {
          id: "say_text",
          name: "Say Text",
          description: "Make the robot speak using text-to-speech",
          category: "interaction",
          icon: "volume-2",
          timeout: 15000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              text: {
                type: "string",
                default: "Hello, I am NAO!",
                description: "Text to speak",
              },
              volume: {
                type: "number",
                minimum: 0.1,
                maximum: 1.0,
                default: 0.7,
                description: "Speech volume (0.1 to 1.0)",
              },
            },
            required: ["text"],
          },
          naoqi: {
            module: "ALTextToSpeech",
            method: "say",
            parameters: ["text"],
          },
        },
        {
          id: "play_animation",
          name: "Play Animation",
          description: "Play a predefined animation or gesture",
          category: "interaction",
          icon: "zap",
          timeout: 20000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              animation: {
                type: "string",
                enum: ["Hello", "Goodbye", "Excited", "Thinking"],
                default: "Hello",
                description: "Animation to play",
              },
            },
            required: ["animation"],
          },
          naoqi: {
            module: "ALAnimationPlayer",
            method: "run",
            parameters: ["animations/Stand/Gestures/{animation}"],
          },
        },
        {
          id: "walk_to_position",
          name: "Walk to Position",
          description:
            "Walk to a specific position relative to current location",
          category: "movement",
          icon: "footprints",
          timeout: 30000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              x: {
                type: "number",
                minimum: -2.0,
                maximum: 2.0,
                default: 0.5,
                description: "Forward distance in meters",
              },
              y: {
                type: "number",
                minimum: -1.0,
                maximum: 1.0,
                default: 0.0,
                description: "Sideways distance in meters (left is positive)",
              },
              theta: {
                type: "number",
                minimum: -3.14159,
                maximum: 3.14159,
                default: 0.0,
                description: "Turn angle in radians",
              },
            },
            required: ["x", "y", "theta"],
          },
          naoqi: {
            module: "ALMotion",
            method: "walkTo",
            parameters: ["x", "y", "theta"],
          },
        },
      ],
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-10T12:00:00"),
    },
    {
      id: "51234567-89ab-cdef-0123-456789abcde3",
      robotId: "31234567-89ab-cdef-0123-456789abcde3",
      name: "TurtleBot3 Waffle Pi",
      version: "2.0.0",
      description:
        "Extended TurtleBot3 platform with additional sensors and computing power for advanced research applications",
      author: "ROBOTIS",
      repositoryUrl: "https://repo.hristudio.com",
      trustLevel: "official" as const,
      status: "active" as const,
      configurationSchema: {
        type: "object",
        properties: {
          namespace: { type: "string", default: "turtlebot3" },
          topics: {
            type: "object",
            properties: {
              cmd_vel: { type: "string", default: "/cmd_vel" },
              odom: { type: "string", default: "/odom" },
              scan: { type: "string", default: "/scan" },
              camera: { type: "string", default: "/camera/image_raw" },
            },
          },
        },
      },
      actionDefinitions: [
        {
          id: "move_velocity",
          name: "Set Velocity",
          description: "Control the robot's linear and angular velocity",
          category: "movement",
          icon: "navigation",
          timeout: 30000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              linear: {
                type: "number",
                minimum: -0.26,
                maximum: 0.26,
                default: 0,
                description: "Forward/backward velocity in m/s",
              },
              angular: {
                type: "number",
                minimum: -1.82,
                maximum: 1.82,
                default: 0,
                description: "Rotational velocity in rad/s",
              },
            },
            required: ["linear", "angular"],
          },
          ros2: {
            messageType: "geometry_msgs/msg/Twist",
            topic: "/cmd_vel",
            payloadMapping: {
              type: "transform",
              transformFn: "transformToTwist",
            },
          },
        },
        {
          id: "capture_image",
          name: "Capture Image",
          description: "Capture an image from the robot's camera",
          category: "sensors",
          icon: "camera",
          timeout: 10000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              filename: {
                type: "string",
                default: "image_{timestamp}.jpg",
                description: "Filename for the captured image",
              },
              quality: {
                type: "integer",
                minimum: 1,
                maximum: 100,
                default: 85,
                description: "JPEG quality (1-100)",
              },
            },
            required: ["filename"],
          },
          ros2: {
            messageType: "sensor_msgs/msg/Image",
            topic: "/camera/image_raw",
            payloadMapping: {
              type: "transform",
              transformFn: "captureAndSaveImage",
            },
          },
        },
        {
          id: "scan_environment",
          name: "Scan Environment",
          description:
            "Perform a 360-degree scan of the environment using LIDAR",
          category: "sensors",
          icon: "radar",
          timeout: 15000,
          retryable: true,
          parameterSchema: {
            type: "object",
            properties: {
              duration: {
                type: "number",
                minimum: 1.0,
                maximum: 10.0,
                default: 3.0,
                description: "Scan duration in seconds",
              },
              save_data: {
                type: "boolean",
                default: true,
                description: "Save scan data to file",
              },
            },
            required: ["duration"],
          },
          ros2: {
            messageType: "sensor_msgs/msg/LaserScan",
            topic: "/scan",
            payloadMapping: {
              type: "transform",
              transformFn: "collectLaserScan",
            },
          },
        },
      ],
      createdAt: new Date("2024-01-01T00:00:00"),
      updatedAt: new Date("2024-01-10T12:00:00"),
    },
  ];

  await db.insert(schema.plugins).values(plugins);
  console.log(`✅ Created ${plugins.length} robot plugins`);
}

async function seedStudyPlugins() {
  console.log("🔌 Seeding study plugin installations...");

  // Check if study plugins already exist
  const existingStudyPlugins = await db.select().from(schema.studyPlugins);
  if (existingStudyPlugins.length > 0) {
    console.log(
      `⚠️  ${existingStudyPlugins.length} study plugin installations already exist, skipping`,
    );
    return;
  }

  // Get study IDs from the existing studies
  const studies = await db.select().from(schema.studies);

  if (studies.length === 0) {
    console.log("⚠️  No studies found. Please run basic seeding first.");
    return;
  }

  const studyPlugins = [
    {
      id: "61234567-89ab-cdef-0123-456789abcde1",
      studyId: studies[0]!.id, // First study (navigation)
      pluginId: "51234567-89ab-cdef-0123-456789abcde1", // TurtleBot3 Burger
      configuration: {
        namespace: "navigation_study",
        topics: {
          cmd_vel: "/navigation_study/cmd_vel",
          odom: "/navigation_study/odom",
          scan: "/navigation_study/scan",
        },
        max_speed: 0.15,
        safety_distance: 0.3,
      },
      installedAt: new Date("2024-01-05T10:00:00"),
      installedBy: studies[0]!.createdBy,
    },
    {
      id: "61234567-89ab-cdef-0123-456789abcde2",
      studyId: studies[1]?.id ?? studies[0]!.id, // Second study (social robots) or fallback
      pluginId: "51234567-89ab-cdef-0123-456789abcde2", // NAO Humanoid
      configuration: {
        ip: "192.168.1.100",
        port: 9559,
        modules: [
          "ALMotion",
          "ALTextToSpeech",
          "ALAnimationPlayer",
          "ALLeds",
          "ALSpeechRecognition",
        ],
        language: "English",
        speech_speed: 100,
        volume: 0.8,
      },
      installedAt: new Date("2024-01-05T11:00:00"),
      installedBy: studies[1]?.createdBy ?? studies[0]!.createdBy,
    },
    {
      id: "61234567-89ab-cdef-0123-456789abcde3",
      studyId: studies[0]!.id, // First study also gets Waffle for advanced tasks
      pluginId: "51234567-89ab-cdef-0123-456789abcde3", // TurtleBot3 Waffle
      configuration: {
        namespace: "advanced_navigation",
        topics: {
          cmd_vel: "/advanced_navigation/cmd_vel",
          odom: "/advanced_navigation/odom",
          scan: "/advanced_navigation/scan",
          camera: "/advanced_navigation/camera/image_raw",
        },
        max_speed: 0.2,
        camera_enabled: true,
        lidar_enabled: true,
      },
      installedAt: new Date("2024-01-05T12:00:00"),
      installedBy: studies[0]!.createdBy,
    },
  ];

  await db.insert(schema.studyPlugins).values(studyPlugins);
  console.log(`✅ Created ${studyPlugins.length} study plugin installations`);
}

async function main() {
  try {
    console.log("🔌 HRIStudio Plugin System Seeding Started");
    console.log("📍 Database:", connectionString.replace(/:[^:]*@/, ":***@"));

    await seedRobots();
    await seedPluginRepositories();
    await seedPlugins();
    await seedStudyPlugins();

    console.log("✅ Plugin system seeding completed successfully!");
    console.log("\n📋 Plugin System Summary:");
    console.log("   🤖 Robots: 3 (TurtleBot3 Burger, NAO, TurtleBot3 Waffle)");
    console.log("   📦 Plugin Repositories: 1 (official HRIStudio repo)");
    console.log("   🔌 Robot Plugins: 3 (with complete action definitions)");
    console.log("   📱 Study Plugin Installations: 3 (active configurations)");
    console.log("\n🎯 Plugin Actions Available:");
    console.log(
      "   📍 TurtleBot3 Burger: 3 actions (movement, navigation, stop)",
    );
    console.log("   🤖 NAO Humanoid: 3 actions (speech, animations, walking)");
    console.log("   📊 TurtleBot3 Waffle: 3 actions (movement, camera, LIDAR)");
    console.log("\n🧪 Test Plugin Integration:");
    console.log("   1. Navigate to any experiment designer");
    console.log("   2. Check 'Robot' category in block library");
    console.log("   3. Plugin actions should appear alongside core blocks");
    console.log("   4. Actions are configured per study installation");
    console.log("\n🚀 Ready to test robot plugin integration!");
  } catch (error) {
    console.error("❌ Plugin seeding failed:", error);
    process.exit(1);
  } finally {
    await client.end();
  }
}

if (require.main === module) {
  void main();
}
