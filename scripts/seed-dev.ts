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

async function main() {
  console.log("🌱 Starting realistic seed script...");



  try {
    NAO_PLUGIN_DEF = await loadNaoPluginDef();

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

    await db.insert(schema.studyPlugins).values({
      studyId: study!.id,
      pluginId: naoPlugin!.id,
      configuration: { robotIp: "10.0.0.42" },
      installedBy: adminUser.id
    });

    const [experiment] = await db.insert(schema.experiments).values({
      studyId: study!.id,
      name: "The Interactive Storyteller",
      description: "A storytelling scenario where the robot tells a story and asks questions to the participant.",
      version: 1,
      status: "ready",
      robotId: naoRobot!.id,
      createdBy: adminUser.id,
    }).returning();

    // 5. Create Steps & Actions (The Interactive Storyteller Protocol)
    console.log("🎬 Creating experiment steps (Interactive Storyteller)...");

    // --- Step 1: The Hook ---
    const [step1] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "The Hook",
      description: "Initial greeting and engagement",
      type: "robot",
      orderIndex: 0,
      required: true,
      durationEstimate: 30
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step1!.id,
        name: "Greet Participant",
        type: "nao6-ros2.say_with_emotion",
        orderIndex: 0,
        parameters: { text: "Hello there! I have a wonderful story to share with you today.", emotion: "happy", speed: 1.0 },
        pluginId: naoPlugin!.id,
        category: "interaction",
        retryable: true
      },
      {
        stepId: step1!.id,
        name: "Wave Greeting",
        type: "nao6-ros2.move_arm",
        orderIndex: 1,
        // Raising right arm to wave position
        parameters: {
          arm: "right",
          shoulder_pitch: -1.0,
          shoulder_roll: -0.3,
          elbow_yaw: 1.5,
          elbow_roll: 0.5,
          speed: 0.5
        },
        pluginId: naoPlugin!.id,
        category: "movement",
        retryable: true
      }
    ]);

    // --- Step 2: The Narrative (Part 1) ---
    const [step2] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "The Narrative - Part 1",
      description: "Robot tells the first part of the story",
      type: "robot",
      orderIndex: 1,
      required: true,
      durationEstimate: 60
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step2!.id,
        name: "Tell Story Part 1",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Once upon a time, in a land far away, there lived a curious robot named Alpha." },
        pluginId: naoPlugin!.id,
        category: "interaction"
      },
      {
        stepId: step2!.id,
        name: "Look at Audience",
        type: "nao6-ros2.move_head",
        orderIndex: 1,
        parameters: { yaw: 0.0, pitch: -0.2, speed: 0.5 },
        pluginId: naoPlugin!.id,
        category: "movement"
      }
    ]);

    // --- Step 3: Comprehension Check (Wizard Decision) ---
    // Note: In a real visual designer, this would be a Branch/Conditional.
    // Here we model it as a Wizard Step where they explicitly choose the next robot action.
    const [step3] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Comprehension Check",
      description: "Wizard verifies participant understanding",
      type: "wizard",
      orderIndex: 2,
      required: true,
      durationEstimate: 45
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step3!.id,
        name: "Ask Question",
        type: "nao6-ros2.say_with_emotion",
        orderIndex: 0,
        parameters: { text: "Did you understand the story so far?", emotion: "happy", speed: 1.0 },
        pluginId: naoPlugin!.id,
        category: "interaction"
      },
      {
        stepId: step3!.id,
        name: "Wait for Wizard Input",
        type: "wizard_wait_for_response",
        orderIndex: 1,
        parameters: {
          prompt_text: "Did participant answer 'Alpha'?",
          response_type: "verbal",
          timeout: 60
        },
        sourceKind: "core",
        category: "wizard"
      }
    ]);

    // --- Step 4: Feedback (Positive/Negative branches implied) ---
    // For linear seed, we just add the Positive feedback step
    const [step4] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Positive Feedback",
      description: "Correct answer response",
      type: "robot",
      orderIndex: 3,
      required: true,
      durationEstimate: 15
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step4!.id,
        name: "Express Agreement",
        type: "nao6-ros2.say_with_emotion",
        orderIndex: 0,
        parameters: { text: "Yes, exactly!", emotion: "happy", speed: 1.0 },
        pluginId: naoPlugin!.id,
        category: "interaction"
      },
      {
        stepId: step4!.id,
        name: "Say Correct",
        type: "nao6-ros2.say_text",
        orderIndex: 1,
        parameters: { text: "That is correct! Well done." },
        pluginId: naoPlugin!.id,
        category: "interaction"
      }
    ]);

    // --- Step 5: Conclusion ---
    const [step5] = await db.insert(schema.steps).values({
      experimentId: experiment!.id,
      name: "Conclusion",
      description: "Wrap up the story",
      type: "robot",
      orderIndex: 4,
      required: true,
      durationEstimate: 30
    }).returning();

    await db.insert(schema.actions).values([
      {
        stepId: step5!.id,
        name: "Finish Story",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Alpha explored the world and learned many things. The end." },
        pluginId: naoPlugin!.id,
        category: "interaction"
      },
      {
        stepId: step5!.id,
        name: "Say Goodbye",
        type: "nao6-ros2.say_with_emotion",
        orderIndex: 1,
        parameters: { text: "Goodbye everyone!", emotion: "happy", speed: 1.0 },
        pluginId: naoPlugin!.id,
        category: "interaction"
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
    console.log(`- Experiment: 'The Interactive Storyteller' (${5} steps created)`);
    console.log(`- ${insertedParticipants.length} Participants`);

  } catch (error) {
    console.error("❌ Seeding failed:", error);
    process.exit(1);
  } finally {
    await connection.end();
  }
}

main();
