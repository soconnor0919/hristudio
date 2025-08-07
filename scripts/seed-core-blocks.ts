import { drizzle } from "drizzle-orm/postgres-js";
import { eq } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

const connectionString =
  process.env.DATABASE_URL ??
  "postgresql://postgres:password@localhost:5140/hristudio";
const client = postgres(connectionString);
const db = drizzle(client, { schema });

async function seedCoreRepository() {
  console.log("🏗️  Seeding core system repository...");

  // Check if core repository already exists
  const existingCoreRepo = await db
    .select()
    .from(schema.pluginRepositories)
    .where(eq(schema.pluginRepositories.url, "https://core.hristudio.com"));

  if (existingCoreRepo.length > 0) {
    console.log("⚠️  Core repository already exists, skipping");
    return;
  }

  // Get the first user to use as creator
  const users = await db.select().from(schema.users);
  const adminUser =
    users.find((u) => u.email?.includes("sarah.chen")) ?? users[0];

  if (!adminUser) {
    console.log("⚠️  No users found. Please run basic seeding first.");
    return;
  }

  const coreRepository = {
    id: "00000000-0000-0000-0000-000000000001",
    name: "HRIStudio Core System Blocks",
    url: "https://core.hristudio.com",
    description:
      "Essential system blocks for experiment design including events, control flow, wizard actions, and logic operations",
    trustLevel: "official" as const,
    isEnabled: true,
    isOfficial: true,
    lastSyncAt: new Date(),
    syncStatus: "completed" as const,
    syncError: null,
    metadata: {
      apiVersion: "1.0",
      pluginApiVersion: "1.0",
      categories: ["core", "wizard", "control", "logic", "events"],
      compatibility: {
        hristudio: { min: "0.1.0", recommended: "0.1.0" },
      },
      isCore: true,
    },
    createdAt: new Date("2024-01-01T00:00:00"),
    updatedAt: new Date(),
    createdBy: adminUser.id,
  };

  await db.insert(schema.pluginRepositories).values([coreRepository]);
  console.log("✅ Created core system repository");
}

async function seedCorePlugin() {
  console.log("🧱 Seeding core system plugin...");

  // Check if core plugin already exists
  const existingCorePlugin = await db
    .select()
    .from(schema.plugins)
    .where(eq(schema.plugins.name, "HRIStudio Core System"));

  if (existingCorePlugin.length > 0) {
    console.log("⚠️  Core plugin already exists, skipping");
    return;
  }

  const corePlugin = {
    id: "00000000-0000-0000-0000-000000000001",
    robotId: null, // Core plugin doesn't need a specific robot
    name: "HRIStudio Core System",
    version: "1.0.0",
    description:
      "Essential system blocks for experiment design including events, control flow, wizard actions, and logic operations",
    author: "HRIStudio Team",
    repositoryUrl: "https://core.hristudio.com",
    trustLevel: "official" as const,
    status: "active" as const,
    configurationSchema: {
      type: "object",
      properties: {
        enableAdvancedBlocks: {
          type: "boolean",
          default: true,
          description: "Enable advanced control flow blocks",
        },
        wizardInterface: {
          type: "string",
          enum: ["basic", "advanced"],
          default: "basic",
          description: "Wizard interface complexity level",
        },
      },
    },
    actionDefinitions: [
      // Event Blocks
      {
        id: "when_trial_starts",
        name: "when trial starts",
        description: "Triggered when the trial begins",
        category: "logic",
        icon: "Play",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {},
          required: [],
        },
        blockType: "hat",
        color: "#22c55e",
        nestable: false,
      },
      {
        id: "when_participant_speaks",
        name: "when participant speaks",
        description: "Triggered when participant says something",
        category: "logic",
        icon: "Mic",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {
            keywords: {
              type: "array",
              items: { type: "string" },
              default: [],
              description: "Optional keywords to listen for",
            },
          },
          required: [],
        },
        blockType: "hat",
        color: "#22c55e",
        nestable: false,
      },

      // Wizard Actions
      {
        id: "wizard_say",
        name: "say",
        description: "Wizard speaks to participant",
        category: "interaction",
        icon: "Users",
        timeout: 30000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            message: {
              type: "string",
              default: "",
              description: "What should the wizard say?",
            },
          },
          required: ["message"],
        },
        blockType: "action",
        color: "#a855f7",
        nestable: false,
      },
      {
        id: "wizard_gesture",
        name: "gesture",
        description: "Wizard performs a gesture",
        category: "interaction",
        icon: "Users",
        timeout: 10000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            type: {
              type: "string",
              enum: ["wave", "point", "nod", "thumbs_up", "clap"],
              default: "wave",
              description: "Type of gesture to perform",
            },
          },
          required: ["type"],
        },
        blockType: "action",
        color: "#a855f7",
        nestable: false,
      },
      {
        id: "wizard_note",
        name: "take note",
        description: "Wizard records an observation",
        category: "sensors",
        icon: "FileText",
        timeout: 15000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            category: {
              type: "string",
              enum: [
                "behavior",
                "performance",
                "engagement",
                "technical",
                "other",
              ],
              default: "behavior",
              description: "Category of observation",
            },
            note: {
              type: "string",
              default: "",
              description: "Observation details",
            },
          },
          required: ["note"],
        },
        blockType: "action",
        color: "#f59e0b",
        nestable: false,
      },

      // Control Flow
      {
        id: "wait",
        name: "wait",
        description: "Pause execution for specified time",
        category: "logic",
        icon: "Clock",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {
            seconds: {
              type: "number",
              minimum: 0.1,
              maximum: 300,
              default: 1,
              description: "Time to wait in seconds",
            },
          },
          required: ["seconds"],
        },
        blockType: "action",
        color: "#f97316",
        nestable: false,
      },
      {
        id: "repeat",
        name: "repeat",
        description: "Execute contained blocks multiple times",
        category: "logic",
        icon: "GitBranch",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {
            times: {
              type: "number",
              minimum: 1,
              maximum: 50,
              default: 3,
              description: "Number of times to repeat",
            },
          },
          required: ["times"],
        },
        blockType: "control",
        color: "#f97316",
        nestable: true,
      },
      {
        id: "if_condition",
        name: "if",
        description: "Conditional execution based on conditions",
        category: "logic",
        icon: "GitBranch",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {
            condition: {
              type: "string",
              enum: [
                "participant_speaks",
                "time_elapsed",
                "wizard_signal",
                "custom_condition",
              ],
              default: "participant_speaks",
              description: "Condition to evaluate",
            },
            value: {
              type: "string",
              default: "",
              description: "Value to compare against (if applicable)",
            },
          },
          required: ["condition"],
        },
        blockType: "control",
        color: "#f97316",
        nestable: true,
      },
      {
        id: "parallel",
        name: "do together",
        description: "Execute multiple blocks simultaneously",
        category: "logic",
        icon: "Layers",
        timeout: 0,
        retryable: false,
        parameterSchema: {
          type: "object",
          properties: {},
          required: [],
        },
        blockType: "control",
        color: "#f97316",
        nestable: true,
      },

      // Data Collection
      {
        id: "start_recording",
        name: "start recording",
        description: "Begin recording specified data streams",
        category: "sensors",
        icon: "Circle",
        timeout: 5000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            streams: {
              type: "array",
              items: {
                type: "string",
                enum: [
                  "video",
                  "audio",
                  "screen",
                  "robot_data",
                  "wizard_actions",
                ],
              },
              default: ["video", "audio"],
              description: "Data streams to record",
            },
            quality: {
              type: "string",
              enum: ["low", "medium", "high"],
              default: "medium",
              description: "Recording quality",
            },
          },
          required: ["streams"],
        },
        blockType: "action",
        color: "#dc2626",
        nestable: false,
      },
      {
        id: "stop_recording",
        name: "stop recording",
        description: "Stop recording and save data",
        category: "sensors",
        icon: "Square",
        timeout: 10000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            save_location: {
              type: "string",
              default: "default",
              description: "Where to save the recording",
            },
          },
          required: [],
        },
        blockType: "action",
        color: "#dc2626",
        nestable: false,
      },
      {
        id: "mark_event",
        name: "mark event",
        description: "Add a timestamped marker to the data",
        category: "sensors",
        icon: "MapPin",
        timeout: 1000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            event_name: {
              type: "string",
              default: "",
              description: "Name of the event to mark",
            },
            description: {
              type: "string",
              default: "",
              description: "Optional event description",
            },
          },
          required: ["event_name"],
        },
        blockType: "action",
        color: "#f59e0b",
        nestable: false,
      },

      // Study Flow Control
      {
        id: "show_instructions",
        name: "show instructions",
        description: "Display instructions to the participant",
        category: "interaction",
        icon: "FileText",
        timeout: 60000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            title: {
              type: "string",
              default: "Instructions",
              description: "Instruction title",
            },
            content: {
              type: "string",
              default: "",
              description: "Instruction content (supports markdown)",
            },
            require_acknowledgment: {
              type: "boolean",
              default: true,
              description: "Require participant to acknowledge reading",
            },
          },
          required: ["content"],
        },
        blockType: "action",
        color: "#3b82f6",
        nestable: false,
      },
      {
        id: "collect_response",
        name: "collect response",
        description: "Collect a response from the participant",
        category: "sensors",
        icon: "MessageCircle",
        timeout: 120000,
        retryable: true,
        parameterSchema: {
          type: "object",
          properties: {
            question: {
              type: "string",
              default: "",
              description: "Question to ask the participant",
            },
            response_type: {
              type: "string",
              enum: ["text", "scale", "choice", "voice"],
              default: "text",
              description: "Type of response to collect",
            },
            options: {
              type: "array",
              items: { type: "string" },
              default: [],
              description: "Options for choice responses",
            },
          },
          required: ["question"],
        },
        blockType: "action",
        color: "#8b5cf6",
        nestable: false,
      },
    ],
    createdAt: new Date("2024-01-01T00:00:00"),
    updatedAt: new Date(),
  };

  await db.insert(schema.plugins).values([corePlugin]);
  console.log("✅ Created core system plugin");
}

async function seedCoreStudyPlugins() {
  console.log("🔗 Installing core plugin in all studies...");

  // Get all studies
  const studies = await db.select().from(schema.studies);

  if (studies.length === 0) {
    console.log("⚠️  No studies found. Please run basic seeding first.");
    return;
  }

  // Check if core plugin installations already exist
  const existingInstallation = await db
    .select()
    .from(schema.studyPlugins)
    .where(
      eq(schema.studyPlugins.pluginId, "00000000-0000-0000-0000-000000000001"),
    );

  if (existingInstallation.length > 0) {
    console.log("⚠️  Core plugin already installed in studies, skipping");
    return;
  }

  const coreInstallations = studies.map((study, index) => ({
    id: `00000000-0000-0000-0000-00000000000${index + 2}`, // Start from 2 to avoid conflicts
    studyId: study.id,
    pluginId: "00000000-0000-0000-0000-000000000001",
    configuration: {
      enableAdvancedBlocks: true,
      wizardInterface: "advanced",
      recordingDefaults: {
        video: true,
        audio: true,
        quality: "high",
      },
    },
    installedAt: new Date("2024-01-01T00:00:00"),
    installedBy: study.createdBy,
  }));

  await db.insert(schema.studyPlugins).values(coreInstallations);
  console.log(`✅ Installed core plugin in ${studies.length} studies`);
}

async function main() {
  try {
    console.log("🏗️  HRIStudio Core System Seeding Started");
    console.log("📍 Database:", connectionString.replace(/:[^:]*@/, ":***@"));

    await seedCoreRepository();
    await seedCorePlugin();
    await seedCoreStudyPlugins();

    console.log("✅ Core system seeding completed successfully!");
    console.log("\n📋 Core System Summary:");
    console.log("   🏗️  Core Repository: 1 (HRIStudio Core System)");
    console.log("   🧱 Core Plugin: 1 (with 15 essential blocks)");
    console.log("   🔗 Study Installations: Installed in all studies");
    console.log("\n🧱 Core Blocks Available:");
    console.log("   🎯 Events: when trial starts, when participant speaks");
    console.log("   🧙 Wizard: say, gesture, take note");
    console.log("   ⏳ Control: wait, repeat, if condition, do together");
    console.log("   📊 Data: start/stop recording, mark event");
    console.log("   📋 Study: show instructions, collect response");
    console.log("\n🎨 Block Designer Integration:");
    console.log("   • All core blocks now come from the plugin system");
    console.log("   • Consistent with robot plugin architecture");
    console.log("   • Easy to extend and version core functionality");
    console.log("   • Unified block management across all categories");
    console.log("\n🚀 Ready to test unified block system!");
  } catch (error) {
    console.error("❌ Core system seeding failed:", error);
    process.exit(1);
  } finally {
    await client.end();
  }
}

if (require.main === module) {
  void main();
}
