import bcrypt from "bcryptjs";
import { drizzle } from "drizzle-orm/postgres-js";
import { sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";
import { createHash } from "crypto";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

// --- NAO6 Plugin Definitions ---
import * as fs from "fs";
import * as path from "path";

// Function to load plugin definition (Remote -> Local Fallback)
async function loadNaoPluginDef() {
  const REMOTE_URL = "https://repo.hristudio.com/plugins/nao6-ros2.json";
  const LOCAL_PATH = path.join(__dirname, "../robot-plugins/plugins/nao6-ros2.json");

  try {
    console.log(`🌐 Attempting to fetch plugin definition from ${REMOTE_URL}...`);
    const response = await fetch(REMOTE_URL, { signal: AbortSignal.timeout(3000) }); // 3s timeout
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    console.log("✅ Successfully fetched plugin definition from remote.");
    return data;
  } catch (err) {
    console.warn(`⚠️ Remote fetch failed (${err instanceof Error ? err.message : String(err)}). Falling back to local file.`);
    const rawPlugin = fs.readFileSync(LOCAL_PATH, "utf-8");
    return JSON.parse(rawPlugin);
  }
}

// Global variable to hold the loaded definition
let NAO_PLUGIN_DEF: any;
let CORE_PLUGIN_DEF: any;
let WOZ_PLUGIN_DEF: any;

function loadSystemPlugin(filename: string) {
  const LOCAL_PATH = path.join(__dirname, `../src/plugins/definitions/${filename}`);
  try {
    const raw = fs.readFileSync(LOCAL_PATH, "utf-8");
    return JSON.parse(raw);
  } catch (err) {
    console.error(`❌ Failed to load system plugin ${filename}:`, err);
    process.exit(1);
  }
}

async function main() {
  console.log("🌱 Starting realistic seed script...");



  try {
    NAO_PLUGIN_DEF = await loadNaoPluginDef();
    CORE_PLUGIN_DEF = loadSystemPlugin("hristudio-core.json");
    WOZ_PLUGIN_DEF = loadSystemPlugin("hristudio-woz.json");

    // Ensure legacy 'actions' property maps to 'actionDefinitions' if needed, though schema supports both if we map it
    if (NAO_PLUGIN_DEF.actions && !NAO_PLUGIN_DEF.actionDefinitions) {
      NAO_PLUGIN_DEF.actionDefinitions = NAO_PLUGIN_DEF.actions;
    }

    // 1. Clean existing data (Full Wipe)
    console.log("🧹 Cleaning existing data...");
    await db.delete(schema.mediaCaptures).where(sql`1=1`);
    await db.delete(schema.trialEvents).where(sql`1=1`);
    await db.delete(schema.trials).where(sql`1=1`);
    await db.delete(schema.actions).where(sql`1=1`);
    await db.delete(schema.steps).where(sql`1=1`);
    await db.delete(schema.experiments).where(sql`1=1`);
    await db.delete(schema.participants).where(sql`1=1`);
    await db.delete(schema.studyPlugins).where(sql`1=1`);
    await db.delete(schema.studyMembers).where(sql`1=1`);
    await db.delete(schema.studies).where(sql`1=1`);
    await db.delete(schema.studies).where(sql`1=1`);
    await db.delete(schema.plugins).where(sql`1=1`);
    await db.delete(schema.pluginRepositories).where(sql`1=1`);
    await db.delete(schema.userSystemRoles).where(sql`1=1`);
    await db.delete(schema.users).where(sql`1=1`);
    await db.delete(schema.robots).where(sql`1=1`);

    // 2. Create Users
    console.log("👥 Creating users...");
    const hashedPassword = await bcrypt.hash("password123", 12);

    const gravatarUrl = (email: string) => `https://www.gravatar.com/avatar/${createHash("md5").update(email.toLowerCase().trim()).digest("hex")}?d=identicon`;

    const [adminUser] = await db.insert(schema.users).values({
      name: "Sean O'Connor",
      email: "sean@soconnor.dev",
      password: hashedPassword,
      emailVerified: new Date(),
      image: gravatarUrl("sean@soconnor.dev"),
    }).returning();

    const [researcherUser] = await db.insert(schema.users).values({
      name: "Dr. Felipe Perrone",
      email: "felipe.perrone@bucknell.edu",
      password: hashedPassword,
      emailVerified: new Date(),
      image: "https://api.dicebear.com/7.x/avataaars/svg?seed=Felipe",
    }).returning();

    if (!adminUser) throw new Error("Failed to create admin user");

    await db.insert(schema.userSystemRoles).values({ userId: adminUser.id, role: "administrator" });

    // 3. Create Robots & Plugins
    console.log("🤖 Creating robots and plugins...");
    const [naoRobot] = await db.insert(schema.robots).values({
      name: "NAO6",
      manufacturer: "SoftBank Robotics",
      model: "NAO V6",
      description: "Humanoid robot for social interaction studies.",
      capabilities: ["speech", "vision", "bipedal_walking", "gestures"],
      communicationProtocol: "ros2",
    }).returning();

    const [naoRepo] = await db.insert(schema.pluginRepositories).values({
      name: "HRIStudio Official Plugins",
      url: "https://github.com/hristudio/plugins",
      description: "Official verified plugins",
      trustLevel: "official",
      isEnabled: true,
      isOfficial: true,
      createdBy: adminUser.id,
    }).returning();

    const [naoPlugin] = await db.insert(schema.plugins).values({
      robotId: naoRobot!.id,
      name: NAO_PLUGIN_DEF.name,
      version: NAO_PLUGIN_DEF.version,
      description: NAO_PLUGIN_DEF.description,
      author: "HRIStudio Team",
      repositoryUrl: "https://github.com/hristudio/plugins/tree/main/nao6",
      trustLevel: "verified",
      actionDefinitions: NAO_PLUGIN_DEF.actionDefinitions,
      metadata: NAO_PLUGIN_DEF,
      status: "active",
      createdAt: new Date(),
    }).returning();

    // 4. Create Study & Experiment - Comparative WoZ Study
    console.log("📚 Creating 'Comparative WoZ Study'...");
    const [study] = await db.insert(schema.studies).values({
      name: "Comparative WoZ Study",
      description: "Comparison of HRIStudio vs Choregraphe for The Interactive Storyteller scenario.",
      institution: "Bucknell University",
      irbProtocol: "2024-HRI-COMP",
      status: "active",
      createdBy: adminUser.id,
    }).returning();

    await db.insert(schema.studyMembers).values([
      { studyId: study!.id, userId: adminUser.id, role: "owner" },
      { studyId: study!.id, userId: researcherUser!.id, role: "researcher" }
    ]);

    // Insert System Plugins
    const [corePlugin] = await db.insert(schema.plugins).values({
      name: CORE_PLUGIN_DEF.name,
      version: CORE_PLUGIN_DEF.version,
      description: CORE_PLUGIN_DEF.description,
      author: CORE_PLUGIN_DEF.author,
      trustLevel: "official",
      actionDefinitions: CORE_PLUGIN_DEF.actionDefinitions,
      robotId: null, // System Plugin
      metadata: { ...CORE_PLUGIN_DEF, id: CORE_PLUGIN_DEF.id },
      status: "active"
    }).returning();

    const [wozPlugin] = await db.insert(schema.plugins).values({
      name: WOZ_PLUGIN_DEF.name,
      version: WOZ_PLUGIN_DEF.version,
      description: WOZ_PLUGIN_DEF.description,
      author: WOZ_PLUGIN_DEF.author,
      trustLevel: "official",
      actionDefinitions: WOZ_PLUGIN_DEF.actionDefinitions,
      robotId: null, // System Plugin
      metadata: { ...WOZ_PLUGIN_DEF, id: WOZ_PLUGIN_DEF.id },
      status: "active"
    }).returning();

    await db.insert(schema.studyPlugins).values([
      {
        studyId: study!.id,
        pluginId: naoPlugin!.id,
        configuration: { robotIp: "10.0.0.42" },
        installedBy: adminUser.id
      },
      {
        studyId: study!.id,
        pluginId: corePlugin!.id,
        configuration: {},
        installedBy: adminUser.id
      },
      {
        studyId: study!.id,
        pluginId: wozPlugin!.id,
        configuration: {},
        installedBy: adminUser.id
      }
    ]);

    const [experiment] = await db.insert(schema.experiments).values({
      studyId: study!.id,
      name: "The Interactive Storyteller",
      description: "A storytelling scenario where the robot tells a story and asks questions to the participant.",
      version: 1,
      status: "ready",
      robotId: naoRobot!.id,
      createdBy: adminUser.id,
      // visualDesign will be auto-generated by designer from DB steps
    }).returning();

    // 5. Create Steps & Actions (The Interactive Storyteller Protocol)
    console.log("🎬 Creating experiment steps (Interactive Storyteller)...");

    // --- Step 1: The Hook ---
    const [step1] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "The Hook",
      description: "Initial greeting and story introduction",
      type: "robot",
      orderIndex: 0,
      required: true,
      durationEstimate: 25
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step1!.id,
        name: "Introduce Story",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Hello. I have a story to tell you about a space traveler. Are you ready?" },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step1!.id,
        name: "Welcome Gesture",
        type: "nao6-ros2.move_arm",
        orderIndex: 1,
        // Open hand/welcome position
        parameters: {
          arm: "right",
          shoulder_pitch: 1.0,
          shoulder_roll: -0.2,
          elbow_yaw: 0.5,
          elbow_roll: -0.4,
          speed: 0.4
        },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      }
    ]);

    // --- Step 2: The Narrative ---
    const [step2] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "The Narrative",
      description: "Robot tells the space traveler story with gaze behavior",
      type: "robot",
      orderIndex: 1,
      required: true,
      durationEstimate: 45
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step2!.id,
        name: "Tell Story",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "The traveler flew to Mars. He found a red rock that glowed in the dark. He put it in his pocket." },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step2!.id,
        name: "Look Away (Thinking)",
        type: "nao6-ros2.turn_head",
        orderIndex: 1,
        parameters: { yaw: 1.5, pitch: 0.0, speed: 0.3 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      },
      {
        stepId: step2!.id,
        name: "Look Back at Participant",
        type: "nao6-ros2.turn_head",
        orderIndex: 2,
        parameters: { yaw: 0.0, pitch: -0.1, speed: 0.4 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      }
    ]);

    // --- Step 3: Comprehension Check (Wizard Decision Point) ---
    // Note: Wizard will choose to proceed to Step 4a (Correct) or 4b (Incorrect)
    // --- Step 3: Comprehension Check (Wizard Decision Point) ---
    // Note: Wizard will choose to proceed to Step 4a (Correct) or 4b (Incorrect)
    // --- Step 4a: Correct Response Branch ---
    const [step4a] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Branch A: Correct Response",
      description: "Response when participant says 'Red'",
      type: "robot",
      orderIndex: 3,
      required: false,
      durationEstimate: 20
    }).returning();

    // --- Step 4b: Incorrect Response Branch ---
    const [step4b] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Branch B: Incorrect Response",
      description: "Response when participant gives wrong answer",
      type: "robot",
      orderIndex: 4,
      required: false,
      durationEstimate: 20
    }).returning();

    // --- Step 3: Comprehension Check (Wizard Decision Point) ---
    // Note: Wizard will choose to proceed to Step 4a (Correct) or 4b (Incorrect)
    const [step3] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Comprehension Check",
      description: "Ask participant about rock color and wait for wizard input",
      type: "conditional",
      orderIndex: 2,
      required: true,
      durationEstimate: 30,
      conditions: {
        variable: "last_wizard_response",
        options: [
          { label: "Correct Response (Red)", value: "Correct", nextStepId: step4a!.id, variant: "default" },
          { label: "Incorrect Response", value: "Incorrect", nextStepId: step4b!.id, variant: "destructive" }
        ]
      }
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step3!.id,
        name: "Ask Question",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "What color was the rock the traveler found?" },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step3!.id,
        name: "Wait for Choice",
        type: "wizard_wait_for_response",
        orderIndex: 1,
        // Define the options that will be presented to the Wizard
        parameters: {
          prompt_text: "Did participant answer 'Red' correctly?",
          options: ["Correct", "Incorrect"]
        },
        sourceKind: "core",
        pluginId: "hristudio-woz", // Explicit link
        category: "wizard"
      },
      {
        stepId: step3!.id,
        name: "Branch Decision",
        type: "branch",
        orderIndex: 2,
        parameters: {},
        sourceKind: "core",
        pluginId: "hristudio-core", // Explicit link
        category: "control"
      }
    ]);

    await db.insert(schema.actions).values([
      {
        stepId: step4a!.id,
        name: "Confirm Correct Answer",
        type: "nao6-ros2.say_with_emotion",
        orderIndex: 0,
        parameters: { text: "Yes! It was a glowing red rock.", emotion: "happy", speed: 1.0 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step4a!.id,
        name: "Nod Head",
        type: "nao6-ros2.turn_head",
        orderIndex: 1,
        parameters: { yaw: 0.0, pitch: -0.3, speed: 0.5 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      },
      {
        stepId: step4a!.id,
        name: "Return to Neutral",
        type: "nao6-ros2.turn_head",
        orderIndex: 2,
        parameters: { yaw: 0.0, pitch: 0.0, speed: 0.4 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      }
    ]);



    await db.insert(schema.actions).values([
      {
        stepId: step4b!.id,
        name: "Correct Participant",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Actually, it was red." },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step4b!.id,
        name: "Shake Head (Left)",
        type: "nao6-ros2.turn_head",
        orderIndex: 1,
        parameters: { yaw: -0.5, pitch: 0.0, speed: 0.5 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      },
      {
        stepId: step4b!.id,
        name: "Shake Head (Right)",
        type: "nao6-ros2.turn_head",
        orderIndex: 2,
        parameters: { yaw: 0.5, pitch: 0.0, speed: 0.5 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      },
      {
        stepId: step4b!.id,
        name: "Return to Center",
        type: "nao6-ros2.turn_head",
        orderIndex: 3,
        parameters: { yaw: 0.0, pitch: 0.0, speed: 0.4 },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      }
    ]);

    // --- Step 5: Conclusion ---
    const [step5] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Conclusion",
      description: "End the story and thank participant",
      type: "robot",
      orderIndex: 5,
      required: true,
      durationEstimate: 25
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step5!.id,
        name: "End Story",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "The End. Thank you for listening." },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "interaction",
        retryable: true
      },
      {
        stepId: step5!.id,
        name: "Bow Gesture",
        type: "nao6-ros2.move_arm",
        orderIndex: 1,
        parameters: {
          arm: "right",
          shoulder_pitch: 1.8,
          shoulder_roll: 0.1,
          elbow_yaw: 0.0,
          elbow_roll: -0.3,
          speed: 0.3
        },
        pluginId: NAO_PLUGIN_DEF.robotId || "nao6-ros2",
        pluginVersion: "2.1.0",
        category: "movement",
        retryable: true
      }
    ]);

    // 6. Participants (N=20 for study)
    console.log("👤 Creating 20 participants for N=20 study...");
    const participants = [];
    for (let i = 1; i <= 20; i++) {
      participants.push({
        studyId: study!.id,
        participantCode: `P${100 + i}`,
        name: `Participant ${100 + i}`,
        consentGiven: true,
        consentGivenAt: new Date(),
        notes: i % 2 === 0 ? "Condition: HRIStudio" : "Condition: Choregraphe"
      });
    }
    const insertedParticipants = await db.insert(schema.participants).values(participants).returning();

    console.log("\n✅ Database seeded successfully!");
    console.log(`Summary:`);
    console.log(`- 1 Admin User (sean@soconnor.dev)`);
    console.log(`- Study: 'Comparative WoZ Study'`);
    console.log(`- Experiment: 'The Interactive Storyteller' (6 steps created)`);
    console.log(`  - Step 1: The Hook (greeting + welcome gesture)`);
    console.log(`  - Step 2: The Narrative (story + gaze sequence)`);
    console.log(`  - Step 3: Comprehension Check (question + wizard wait)`);
    console.log(`  - Step 4a: Branch A - Correct Response (affirmation + nod)`);
    console.log(`  - Step 4b: Branch B - Incorrect Response (correction + head shake)`);
    console.log(`  - Step 5: Conclusion (ending + bow)`);
    console.log(`- ${insertedParticipants.length} Participants`);

  } catch (error) {
    console.error("❌ Seeding failed:", error);
    process.exit(1);
  } finally {
    await connection.end();
  }
}

main();
