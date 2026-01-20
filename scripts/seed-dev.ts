import bcrypt from "bcryptjs";
import { drizzle } from "drizzle-orm/postgres-js";
import { eq, sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

// --- NAO6 Plugin Definitions (Inlined for reliability) ---
const NAO_PLUGIN_DEF = {
  name: "NAO6 Robot (Enhanced ROS2 Integration)",
  version: "2.0.0",
  description: "Comprehensive NAO6 robot integration for HRIStudio experiments via ROS2.",
  actions: [
    {
      id: "nao_speak",
      name: "Speak Text",
      category: "speech",
      parametersSchema: {
        type: "object",
        properties: {
          text: { type: "string" },
          volume: { type: "number", default: 0.7 }
        },
        required: ["text"]
      }
    },
    {
      id: "nao_gesture",
      name: "Perform Gesture",
      category: "interaction",
      parametersSchema: {
        type: "object",
        properties: {
          gesture: { type: "string", enum: ["wave", "bow", "point"] },
          speed: { type: "number", default: 0.8 }
        }
      }
    },
    {
      id: "nao_look_at",
      name: "Look At",
      category: "movement",
      parametersSchema: {
        type: "object",
        properties: {
          target: { type: "string", enum: ["participant", "screen", "away"] },
          duration: { type: "number", default: 2.0 }
        }
      }
    }
  ]
};

async function main() {
  console.log("🌱 Starting realistic seed script...");

  try {
    // 1. Clean existing data
    console.log("🧹 Cleaning existing data...");
    // Delete in reverse dependency order
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

    const [adminUser] = await db.insert(schema.users).values({
      name: "Sean O'Connor",
      email: "sean@soconnor.dev",
      password: hashedPassword,
      emailVerified: new Date(),
      image: "https://api.dicebear.com/7.x/avataaars/svg?seed=Sean", // Consistent avatar
    }).returning();

    const [researcherUser] = await db.insert(schema.users).values({
      name: "Dr. Felipe Perrone",
      email: "felipe.perrone@bucknell.edu",
      password: hashedPassword,
      emailVerified: new Date(),
      image: "https://api.dicebear.com/7.x/avataaars/svg?seed=Felipe",
    }).returning();

    if (!adminUser) throw new Error("Failed to create admin user");

    await db.insert(schema.userSystemRoles).values({
      userId: adminUser.id,
      role: "administrator",
    });

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
      status: "active",
      createdBy: adminUser.id,
    }).returning();

    const [naoPlugin] = await db.insert(schema.plugins).values({
      robotId: naoRobot!.id,
      repositoryId: naoRepo!.id,
      name: NAO_PLUGIN_DEF.name,
      version: NAO_PLUGIN_DEF.version,
      description: NAO_PLUGIN_DEF.description,
      author: "HRIStudio Team",
      trustLevel: "official",
      status: "active",
      repositoryUrl: naoRepo!.url,
      actionDefinitions: NAO_PLUGIN_DEF.actions,
      configurationSchema: {
        type: "object",
        properties: {
          robotIp: { type: "string", default: "192.168.1.100" }
        }
      },
      metadata: { category: "robot_control" }
    }).returning();

    // 4. Create Study & Experiment
    console.log("📚 Creating study and experiment...");
    const [study] = await db.insert(schema.studies).values({
      name: "Social Robot Attention Study",
      description: "Investigating the effect of robot gaze on participant attention retention.",
      institution: "Bucknell University",
      irbProtocol: "2024-HRI-055",
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
      name: "Attention Gaze Protocol A",
      description: "Condition A: Robot maintains eye contact.",
      version: 1,
      status: "ready", // Correct enum value
      createdBy: adminUser.id,
    }).returning();

    // 5. Participants
    console.log("👤 Creating participants...");
    const participants = [];
    for (let i = 1; i <= 5; i++) {
      participants.push({
        studyId: study!.id,
        participantCode: `P${100 + i}`,
        name: `Participant ${100 + i}`,
        consentGiven: true,
        consentGivenAt: new Date(),
        notes: i % 2 === 0 ? "Condition A" : "Condition B"
      });
    }
    const insertedParticipants = await db.insert(schema.participants).values(participants).returning();

    // 6. Trials & Realistic Logs
    console.log("🧪 Generating trials with dense logs...");

    for (const p of insertedParticipants) {
      const isCompleted = Math.random() > 0.2; // 80% completed
      const trialStart = new Date(Date.now() - Math.random() * 7 * 24 * 60 * 60 * 1000);
      const durationSecs = 300 + Math.floor(Math.random() * 300); // 5-10 mins
      const trialEnd = new Date(trialStart.getTime() + durationSecs * 1000);

      const [trial] = await db.insert(schema.trials).values({
        studyId: study!.id, // Ensure referencing study if schema allows, otherwise via experiment
        experimentId: experiment!.id,
        participantId: p.id,
        wizardId: adminUser.id,
        sessionNumber: 1,
        status: isCompleted ? "completed" : "in_progress",
        startedAt: trialStart,
        completedAt: isCompleted ? trialEnd : null,
        duration: isCompleted ? durationSecs : null,
        createdBy: adminUser.id,
      }).returning();

      // Generate dense events
      let currentTime = trialStart.getTime();
      const events = [];

      // Event: Trial Start
      events.push({
        trialId: trial!.id,
        eventType: "system",
        timestamp: new Date(currentTime),
        data: { message: "Trial started", system_check: "nominal" },
        createdBy: adminUser.id
      });
      currentTime += 2000;

      // Event: Wizard Introduction (Wizard Action)
      events.push({
        trialId: trial!.id,
        eventType: "wizard_action",
        timestamp: new Date(currentTime),
        data: { action: "read_script", section: "intro" },
        createdBy: adminUser.id
      });
      currentTime += 5000;

      // Loop for interaction events
      const interactionCount = 15;
      for (let k = 0; k < interactionCount; k++) {
        // Gap
        currentTime += 2000 + Math.random() * 5000;

        // 1. Robot Action (Speak/Gesture)
        const actionType = Math.random() > 0.6 ? "nao_gesture" : "nao_speak";
        events.push({
          trialId: trial!.id,
          eventType: "robot_action",
          timestamp: new Date(currentTime),
          data: actionType === "nao_gesture"
            ? { plugin: "nao6", action: "gesture", type: "wave", speed: 0.8 }
            : { plugin: "nao6", action: "speak", text: "Please look at the screen now.", volume: 0.7 },
          createdBy: adminUser.id
        });

        // Execution log
        currentTime += 100;
        events.push({
          trialId: trial!.id,
          eventType: "system",
          timestamp: new Date(currentTime),
          data: { source: "ros2_bridge", topic: "/nao/cmd", status: "sent" },
          createdBy: adminUser.id
        });

        // 2. Participant Reaction (Simulated Logs/Wizard Note)
        if (Math.random() > 0.7) {
          currentTime += 3000;
          events.push({
            trialId: trial!.id,
            eventType: "wizard_note",
            timestamp: new Date(currentTime),
            data: { note: "Participant looked away briefly.", tag: "distraction" },
            createdBy: adminUser.id
          });
        }
      }

      // End
      if (isCompleted) {
        events.push({
          trialId: trial!.id,
          eventType: "system",
          timestamp: trialEnd,
          data: { message: "Trial completed successfully" },
          createdBy: adminUser.id
        });

        // Fake Media Capture
        await db.insert(schema.mediaCaptures).values({
          trialId: trial!.id,
          mediaType: "video", // Changed from "video/webm" to general "video"
          storagePath: `trials/${trial!.id}/recording.webm`,
          fileSize: 15480000 + Math.floor(Math.random() * 5000000), // ~15-20MB
          duration: durationSecs,
          startTimestamp: trialStart,
          endTimestamp: trialEnd,
        });
      }

      await db.insert(schema.trialEvents).values(events);
    }

    console.log("\n✅ Database seeded successfully!");
    console.log(`Summary:`);
    console.log(`- 1 Admin User (sean@soconnor.dev)`);
    console.log(`- 1 Study (Social Robot Attention)`);
    console.log(`- ${insertedParticipants.length} Participants`);
    console.log(`- ${insertedParticipants.length} Trials created (mixed status)`);
    console.log(`- ~20 Events per trial`);

  } catch (error) {
    console.error("❌ Seeding failed:", error);
    process.exit(1);
  } finally {
    await connection.end();
  }
}

main();
