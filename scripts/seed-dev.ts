import bcrypt from "bcryptjs";
import { drizzle } from "drizzle-orm/postgres-js";
import { eq, sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";
import { readFile } from "node:fs/promises";
import path from "node:path";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

// Repository sync helper
async function syncRepository(
  repoId: string,
  repoUrl: string,
): Promise<number> {
  try {
    console.log(`🔄 Syncing repository: ${repoUrl}`);

    // Resolve source: use local public repo for core, remote URL otherwise
    const isCore = repoUrl.includes("core.hristudio.com");
    const devUrl = repoUrl;

    // Fetch repository metadata (local filesystem for core)
    const repoMetadata = isCore
      ? (JSON.parse(
          await readFile(
            path.join(
              process.cwd(),
              "public",
              "hristudio-core",
              "repository.json",
            ),
            "utf8",
          ),
        ) as {
          description?: string;
          author?: { name?: string };
          urls?: { git?: string };
          trust?: string;
        })
      : await (async () => {
          const repoResponse = await fetch(`${devUrl}/repository.json`);
          if (!repoResponse.ok) {
            throw new Error(
              `Failed to fetch repository metadata: ${repoResponse.status}`,
            );
          }
          return (await repoResponse.json()) as {
            description?: string;
            author?: { name?: string };
            urls?: { git?: string };
            trust?: string;
          };
        })();

    // For core repository, create a single plugin with all block groups
    if (isCore) {
      const indexData = JSON.parse(
        await readFile(
          path.join(
            process.cwd(),
            "public",
            "hristudio-core",
            "plugins",
            "index.json",
          ),
          "utf8",
        ),
      ) as {
        plugins?: Array<{ blockCount?: number }>;
      };

      // Create core system plugin
      await db.insert(schema.plugins).values({
        robotId: null,
        name: "HRIStudio Core System",
        version: "1.0.0",
        description: repoMetadata.description ?? "",
        author: repoMetadata.author?.name ?? "Unknown",
        repositoryUrl: repoMetadata.urls?.git ?? "",
        trustLevel:
          (repoMetadata.trust as "official" | "verified" | "community") ??
          "community",
        status: "active",
        actionDefinitions: [],
        metadata: {
          platform: "Core",
          category: "system",
          repositoryId: repoId,
          blockGroups: indexData.plugins ?? [],
          totalBlocks:
            indexData.plugins?.reduce(
              (sum: number, p: { blockCount?: number }) =>
                sum + (p.blockCount ?? 0),
              0,
            ) ?? 0,
        },
      });

      console.log(
        `✅ Synced core system with ${indexData.plugins?.length ?? 0} block groups`,
      );
      return 1;
    }

    // For robot repositories, sync individual plugins
    const pluginIndexResponse = await fetch(`${devUrl}/plugins/index.json`);
    if (!pluginIndexResponse.ok) {
      throw new Error(
        `Failed to fetch plugin index: ${pluginIndexResponse.status}`,
      );
    }
    const pluginFiles = (await pluginIndexResponse.json()) as string[];

    let syncedCount = 0;
    for (const pluginFile of pluginFiles) {
      try {
        const pluginResponse = await fetch(`${devUrl}/plugins/${pluginFile}`);
        if (!pluginResponse.ok) {
          console.warn(
            `Failed to fetch ${pluginFile}: ${pluginResponse.status}`,
          );
          continue;
        }
        const pluginData = (await pluginResponse.json()) as {
          name?: string;
          version?: string;
          description?: string;
          manufacturer?: { name?: string };
          documentation?: { mainUrl?: string };
          trustLevel?: string;
          actions?: unknown[];
          platform?: string;
          category?: string;
          specs?: unknown;
          ros2Config?: unknown;
        };

        await db.insert(schema.plugins).values({
          robotId: null, // Will be matched later if needed
          name: pluginData.name ?? pluginFile.replace(".json", ""),
          version: pluginData.version ?? "1.0.0",
          description: pluginData.description ?? "",
          author:
            pluginData.manufacturer?.name ??
            repoMetadata.author?.name ??
            "Unknown",
          repositoryUrl:
            pluginData.documentation?.mainUrl ?? repoMetadata.urls?.git ?? "",
          trustLevel:
            (pluginData.trustLevel as "official" | "verified" | "community") ??
            (repoMetadata.trust as "official" | "verified" | "community") ??
            "community",
          status: "active",
          actionDefinitions: pluginData.actions ?? [],
          metadata: {
            platform: pluginData.platform,
            category: pluginData.category,
            repositoryId: repoId,
            specs: pluginData.specs,
            ros2Config: pluginData.ros2Config,
          },
        });

        console.log(`✅ Synced plugin: ${pluginData.name}`);
        syncedCount++;
      } catch (error) {
        console.warn(`Failed to process ${pluginFile}:`, error);
      }
    }

    return syncedCount;
  } catch (error) {
    console.error(`Failed to sync repository ${repoUrl}:`, error);
    return 0;
  }
}

async function main() {
  console.log("🌱 Starting simplified seed script...");

  try {
    // Clean existing data (in reverse order of dependencies)
    console.log("🧹 Cleaning existing data...");
    await db.delete(schema.studyPlugins).where(sql`1=1`);
    await db.delete(schema.plugins).where(sql`1=1`);
    await db.delete(schema.pluginRepositories).where(sql`1=1`);
    await db.delete(schema.trialEvents).where(sql`1=1`);
    await db.delete(schema.trials).where(sql`1=1`);
    await db.delete(schema.steps).where(sql`1=1`);
    await db.delete(schema.experiments).where(sql`1=1`);
    await db.delete(schema.participants).where(sql`1=1`);
    await db.delete(schema.studyMembers).where(sql`1=1`);
    await db.delete(schema.userSystemRoles).where(sql`1=1`);
    await db.delete(schema.studies).where(sql`1=1`);
    await db.delete(schema.users).where(sql`1=1`);
    await db.delete(schema.robots).where(sql`1=1`);

    // Create robots
    console.log("🤖 Creating robots...");
    const robots = [
      {
        name: "TurtleBot3 Burger",
        manufacturer: "ROBOTIS",
        model: "TurtleBot3 Burger",
        description:
          "A compact, affordable, programmable, ROS2-based mobile robot for education and research",
        capabilities: ["differential_drive", "lidar", "imu", "odometry"],
        communicationProtocol: "ros2" as const,
      },
      {
        name: "NAO Humanoid Robot",
        manufacturer: "SoftBank Robotics",
        model: "NAO V6",
        description:
          "Humanoid robot designed for education, research, and social interaction",
        capabilities: ["speech", "vision", "walking", "gestures"],
        communicationProtocol: "rest" as const,
      },
    ];

    const insertedRobots = await db
      .insert(schema.robots)
      .values(robots)
      .returning();
    console.log(`✅ Created ${insertedRobots.length} robots`);

    // Create users (Bucknell University team)
    console.log("👥 Creating users...");
    const hashedPassword = await bcrypt.hash("password123", 12);

    const users = [
      {
        name: "Sean O'Connor",
        email: "sean@soconnor.dev",
        password: hashedPassword,
        emailVerified: new Date(),
        image: null,
      },
      {
        name: "L. Felipe Perrone",
        email: "felipe.perrone@bucknell.edu",
        password: hashedPassword,
        emailVerified: new Date(),
        image: null,
      },
    ];

    const insertedUsers = await db
      .insert(schema.users)
      .values(users)
      .returning();
    console.log(`✅ Created ${insertedUsers.length} users`);

    // Assign system roles
    console.log("🎭 Assigning system roles...");
    const seanUser = insertedUsers.find((u) => u.email === "sean@soconnor.dev");

    if (!seanUser) {
      throw new Error("Sean user not found after creation");
    }

    await db.insert(schema.userSystemRoles).values({
      userId: seanUser.id,
      role: "administrator",
    });

    console.log(`✅ Assigned administrator role to Sean`);

    // Create plugin repositories
    console.log("📦 Creating plugin repositories...");
    const repositories = [
      {
        name: "HRIStudio Core System Blocks",
        url: "https://core.hristudio.com",
        description:
          "Essential system blocks for experiment design including events, control flow, wizard actions, and logic operations",
        trustLevel: "official" as const,
        isEnabled: true,
        isOfficial: true,
        syncStatus: "pending" as const,
        createdBy: seanUser.id,
      },
      {
        name: "HRIStudio Official Robot Plugins",
        url: "https://repo.hristudio.com",
        description:
          "Official collection of robot plugins maintained by the HRIStudio team",
        trustLevel: "official" as const,
        isEnabled: true,
        isOfficial: true,
        syncStatus: "pending" as const,
        createdBy: seanUser.id,
      },
    ];

    const insertedRepos = await db
      .insert(schema.pluginRepositories)
      .values(repositories)
      .returning();
    console.log(`✅ Created ${insertedRepos.length} plugin repositories`);

    // Sync repositories to populate plugins
    console.log("🔄 Syncing plugin repositories...");
    let totalPlugins = 0;

    for (const repo of insertedRepos) {
      const syncedCount = await syncRepository(repo.id, repo.url);
      totalPlugins += syncedCount;

      // Update sync status
      await db
        .update(schema.pluginRepositories)
        .set({
          syncStatus: syncedCount > 0 ? "completed" : "failed",
          lastSyncAt: new Date(),
        })
        .where(eq(schema.pluginRepositories.id, repo.id));
    }

    // Create studies
    console.log("📚 Creating studies...");
    const studies = [
      {
        name: "NAO Classroom Interaction",
        description:
          "Evaluating student engagement with NAO-led prompts during lab sessions",
        institution: "Bucknell University",
        irbProtocol: "BU-IRB-2025-NAO-01",
        status: "active" as const,
        createdBy: seanUser.id,
      },
      {
        name: "Wizard-of-Oz Dialogue Study",
        description:
          "WoZ-controlled NAO to assess timing and tone in instructional feedback",
        institution: "Bucknell University",
        irbProtocol: "BU-IRB-2025-WOZ-02",
        status: "draft" as const,
        createdBy: seanUser.id,
      },
    ];

    const insertedStudies = await db
      .insert(schema.studies)
      .values(studies)
      .returning();
    console.log(`✅ Created ${insertedStudies.length} studies`);

    // Create study memberships
    console.log("👥 Creating study memberships...");
    const studyMemberships = [];

    // Sean as owner of all studies
    for (const study of insertedStudies) {
      studyMemberships.push({
        studyId: study.id,
        userId: seanUser.id,
        role: "owner" as const,
      });
    }

    // Add other users as researchers/wizards
    const otherUsers = insertedUsers.filter((u) => u.id !== seanUser.id);
    if (otherUsers.length > 0 && insertedStudies[0]) {
      studyMemberships.push({
        studyId: insertedStudies[0].id,
        userId: otherUsers[0]!.id,
        role: "researcher" as const,
      });

      if (otherUsers.length > 1 && insertedStudies[1]) {
        studyMemberships.push({
          studyId: insertedStudies[1].id,
          userId: otherUsers[1]!.id,
          role: "wizard" as const,
        });
      }
    }

    await db.insert(schema.studyMembers).values(studyMemberships);
    console.log(`✅ Created ${studyMemberships.length} study memberships`);

    // Install core plugin in all studies
    console.log("🔌 Installing core plugin in all studies...");
    const corePlugin = await db
      .select()
      .from(schema.plugins)
      .where(eq(schema.plugins.name, "HRIStudio Core System"))
      .limit(1);

    if (corePlugin.length > 0) {
      const coreInstallations = insertedStudies.map((study) => ({
        studyId: study.id,
        pluginId: corePlugin[0]!.id,
        configuration: {},
        installedBy: seanUser.id,
      }));

      await db.insert(schema.studyPlugins).values(coreInstallations);
      console.log(
        `✅ Installed core plugin in ${insertedStudies.length} studies`,
      );
    }

    // Install NAO plugin for first study if available
    console.log("🤝 Installing NAO plugin (if available)...");
    const naoPlugin = await db
      .select()
      .from(schema.plugins)
      .where(eq(schema.plugins.name, "NAO Humanoid Robot"))
      .limit(1);
    if (naoPlugin.length > 0 && insertedStudies[0]) {
      await db.insert(schema.studyPlugins).values({
        studyId: insertedStudies[0].id,
        pluginId: naoPlugin[0]!.id,
        configuration: { voice: "nao-tts", locale: "en-US" },
        installedBy: seanUser.id,
      });
      console.log("✅ Installed NAO plugin in first study");
    } else {
      console.log(
        "ℹ️ NAO plugin not found in repository sync; continuing without it",
      );
    }

    // Create some participants
    console.log("👤 Creating participants...");
    const participants = [];

    for (let i = 0; i < insertedStudies.length; i++) {
      const study = insertedStudies[i];
      if (study) {
        participants.push(
          {
            studyId: study.id,
            participantCode: `P${String(i * 2 + 1).padStart(3, "0")}`,
            name: `Participant ${i * 2 + 1}`,
            email: `participant${i * 2 + 1}@example.com`,
            demographics: { age: 25 + i, gender: "prefer not to say" },
            consentGiven: true,
            consentGivenAt: new Date(),
          },
          {
            studyId: study.id,
            participantCode: `P${String(i * 2 + 2).padStart(3, "0")}`,
            name: `Participant ${i * 2 + 2}`,
            email: `participant${i * 2 + 2}@example.com`,
            demographics: { age: 30 + i, gender: "prefer not to say" },
            consentGiven: true,
            consentGivenAt: new Date(),
          },
        );
      }
    }

    const insertedParticipants = await db
      .insert(schema.participants)
      .values(participants)
      .returning();
    console.log(`✅ Created ${insertedParticipants.length} participants`);

    // Create experiments (include one NAO-based)
    console.log("🧪 Creating experiments...");
    const experiments = [
      {
        studyId: insertedStudies[0]!.id,
        name: "Basic Interaction Protocol 1",
        description: "Wizard prompts + NAO speaks demo script",
        version: 1,
        status: "ready" as const,
        estimatedDuration: 25,
        createdBy: seanUser.id,
      },
      {
        studyId: insertedStudies[1]!.id,
        name: "Dialogue Timing Pilot",
        description: "Compare response timing variants under WoZ control",
        version: 1,
        status: "draft" as const,
        estimatedDuration: 35,
        createdBy: seanUser.id,
      },
    ];

    const insertedExperiments = await db
      .insert(schema.experiments)
      .values(
        experiments.map((e) => ({
          ...e,
          visualDesign: {
            // minimal starter design; steps optionally overwritten below for DB tables
            steps: [],
            version: 1,
            lastSaved: new Date().toISOString(),
          },
        })),
      )
      .returning();
    console.log(`✅ Created ${insertedExperiments.length} experiments`);

    // Seed a richer, multi-step design for the first experiment (wizard + robot)
    if (insertedExperiments[0]) {
      const exp = insertedExperiments[0];

      // Step 1: Wizard demo + robot speaks
      const step1 = await db
        .insert(schema.steps)
        .values({
          experimentId: exp.id,
          name: "Step 1 • Introduction & Object Demo",
          description: "Wizard greets participant and demonstrates an object",
          type: "wizard",
          orderIndex: 0,
          required: true,
          conditions: {},
        })
        .returning();
      const step1Id = step1[0]!.id;

      // Action 1.1: Wizard shows object
      await db.insert(schema.actions).values({
        stepId: step1Id,
        name: "show object",
        description: "Wizard presents or demonstrates an object",
        type: "wizard_show_object",
        orderIndex: 0,
        parameters: { object: "Cube" },
        sourceKind: "core",
        category: "wizard",
        transport: "internal",
        retryable: false,
      });

      // Resolve NAO plugin id/version for namespaced action type
      const naoDbPlugin1 = await db
        .select({ id: schema.plugins.id, version: schema.plugins.version })
        .from(schema.plugins)
        .where(eq(schema.plugins.name, "NAO Humanoid Robot"))
        .limit(1);
      const naoPluginRow1 = naoDbPlugin1[0];

      // Action 1.2: Robot/NAO says text (or wizard says fallback)
      await db.insert(schema.actions).values({
        stepId: step1Id,
        name: naoPluginRow1 ? "NAO Say Text" : "Wizard Say",
        description: naoPluginRow1
          ? "Make the robot speak using text-to-speech"
          : "Wizard speaks to participant",
        type: naoPluginRow1 ? `${naoPluginRow1.id}.say_text` : "wizard_say",
        orderIndex: 1,
        parameters: naoPluginRow1
          ? { text: "Hello, I am NAO. Let's begin!", speed: 110, volume: 0.75 }
          : { message: "Hello! Let's begin the session.", tone: "friendly" },
        sourceKind: naoPluginRow1 ? "plugin" : "core",
        pluginId: naoPluginRow1 ? naoPluginRow1.id : null,
        pluginVersion: naoPluginRow1 ? naoPluginRow1.version : null,
        category: naoPluginRow1 ? "robot" : "wizard",
        transport: naoPluginRow1 ? "rest" : "internal",
        retryable: false,
      });

      // Step 2: Wait for response (wizard)
      const step2 = await db
        .insert(schema.steps)
        .values({
          experimentId: exp.id,
          name: "Step 2 • Participant Response",
          description: "Wizard waits for the participant's response",
          type: "wizard",
          orderIndex: 1,
          required: true,
          conditions: {},
        })
        .returning();
      const step2Id = step2[0]!.id;

      await db.insert(schema.actions).values({
        stepId: step2Id,
        name: "wait for response",
        description: "Wizard waits for participant to respond",
        type: "wizard_wait_for_response",
        orderIndex: 0,
        parameters: {
          response_type: "verbal",
          timeout: 20,
          prompt_text: "What did you notice about the object?",
        },
        sourceKind: "core",
        category: "wizard",
        transport: "internal",
        retryable: false,
      });

      // Step 3: Robot LED feedback (or record note fallback)
      const step3 = await db
        .insert(schema.steps)
        .values({
          experimentId: exp.id,
          name: "Step 3 • Robot Feedback",
          description: "Provide feedback using robot LED color or record note",
          type: "robot",
          orderIndex: 2,
          required: false,
          conditions: {},
        })
        .returning();
      const step3Id = step3[0]!.id;

      const naoDbPlugin2 = await db
        .select({ id: schema.plugins.id, version: schema.plugins.version })
        .from(schema.plugins)
        .where(eq(schema.plugins.name, "NAO Humanoid Robot"))
        .limit(1);
      const naoPluginRow2 = naoDbPlugin2[0];

      if (naoPluginRow2) {
        await db.insert(schema.actions).values({
          stepId: step3Id,
          name: "Set LED Color",
          description: "Change NAO's eye LEDs to reflect state",
          type: `${naoPluginRow2.id}.set_led_color`,
          orderIndex: 0,
          parameters: { color: "blue", intensity: 0.6 },
          sourceKind: "plugin",
          pluginId: naoPluginRow2.id,
          pluginVersion: naoPluginRow2.version,
          category: "robot",
          transport: "rest",
          retryable: false,
        });
      } else {
        await db.insert(schema.actions).values({
          stepId: step3Id,
          name: "record note",
          description: "Wizard records an observation",
          type: "wizard_record_note",
          orderIndex: 0,
          parameters: {
            note_type: "observation",
            prompt: "No robot available",
          },
          sourceKind: "core",
          category: "wizard",
          transport: "internal",
          retryable: false,
        });
      }
    }

    // Seed a richer design for the second experiment (timers + conditional/parallel)
    if (insertedExperiments[1]) {
      const exp2 = insertedExperiments[1];

      // Step A: Baseline prompt
      const stepA = await db
        .insert(schema.steps)
        .values({
          experimentId: exp2.id,
          name: "Step A • Baseline Prompt",
          description: "Wizard provides a baseline instruction",
          type: "wizard",
          orderIndex: 0,
          required: true,
          conditions: {},
        })
        .returning();
      const stepAId = stepA[0]!.id;

      await db.insert(schema.actions).values({
        stepId: stepAId,
        name: "say",
        description: "Wizard speaks to participant",
        type: "wizard_say",
        orderIndex: 0,
        parameters: {
          message: "We'll try a short timing task next.",
          tone: "instructional",
        },
        sourceKind: "core",
        category: "wizard",
        transport: "internal",
        retryable: false,
      });

      // Step B: Parallel gestures/animation
      const stepB = await db
        .insert(schema.steps)
        .values({
          experimentId: exp2.id,
          name: "Step B • Parallel Cues",
          description: "Provide multiple cues at once (gesture + animation)",
          type: "parallel",
          orderIndex: 1,
          required: false,
          conditions: {},
        })
        .returning();
      const stepBId = stepB[0]!.id;

      await db.insert(schema.actions).values({
        stepId: stepBId,
        name: "gesture",
        description: "Wizard performs a physical gesture",
        type: "wizard_gesture",
        orderIndex: 0,
        parameters: { type: "point", direction: "participant" },
        sourceKind: "core",
        category: "wizard",
        transport: "internal",
        retryable: false,
      });

      const naoDbPluginB = await db
        .select({ id: schema.plugins.id, version: schema.plugins.version })
        .from(schema.plugins)
        .where(eq(schema.plugins.name, "NAO Humanoid Robot"))
        .limit(1);
      const naoPluginRowB = naoDbPluginB[0];

      if (naoPluginRowB) {
        await db.insert(schema.actions).values({
          stepId: stepBId,
          name: "Play Animation",
          description: "NAO plays a greeting animation",
          type: `${naoPluginRowB.id}.play_animation`,
          orderIndex: 1,
          parameters: { animation: "Hello" },
          sourceKind: "plugin",
          pluginId: naoPluginRowB.id,
          pluginVersion: naoPluginRowB.version,
          category: "robot",
          transport: "rest",
          retryable: false,
        });
      }

      // Step C: Conditional follow-up after a brief wait
      const stepC = await db
        .insert(schema.steps)
        .values({
          experimentId: exp2.id,
          name: "Step C • Conditional Follow-up",
          description: "Proceed based on observed response after timer",
          type: "conditional",
          orderIndex: 2,
          required: false,
          conditions: { predicate: "response_received", timer_ms: 3000 },
        })
        .returning();
      const stepCId = stepC[0]!.id;

      await db.insert(schema.actions).values({
        stepId: stepCId,
        name: "record note",
        description: "Wizard records a follow-up note",
        type: "wizard_record_note",
        orderIndex: 0,
        parameters: {
          note_type: "participant_response",
          prompt: "Response after parallel cues",
        },
        sourceKind: "core",
        category: "wizard",
        transport: "internal",
        retryable: false,
      });
    }

    // Create some trials for dashboard demo
    console.log("🧪 Creating sample trials...");
    const trials = [];

    for (const experiment of insertedExperiments) {
      if (!experiment) continue;

      const studyParticipants = insertedParticipants.filter(
        (p) => p.studyId === experiment.studyId,
      );

      if (studyParticipants.length > 0) {
        // Create 2-3 trials per experiment
        const trialCount = Math.min(studyParticipants.length, 3);
        for (let j = 0; j < trialCount; j++) {
          const participant = studyParticipants[j];
          if (participant) {
            const scheduledAt = new Date(
              Date.now() - Math.random() * 2 * 24 * 60 * 60 * 1000,
            );
            const startedAt = new Date(scheduledAt.getTime() + 30 * 60 * 1000); // 30 minutes after scheduled
            const completedAt = new Date(startedAt.getTime() + 45 * 60 * 1000); // 45 minutes after started

            // Vary the status: some completed, some in progress, some scheduled
            let status: "scheduled" | "in_progress" | "completed" | "aborted";
            let actualStartedAt = null;
            let actualCompletedAt = null;

            if (j === 0) {
              status = "completed";
              actualStartedAt = startedAt;
              actualCompletedAt = completedAt;
            } else if (j === 1 && trialCount > 2) {
              status = "in_progress";
              actualStartedAt = startedAt;
            } else {
              status = "scheduled";
            }

            trials.push({
              participantId: participant.id,
              experimentId: experiment.id,
              sessionNumber: j + 1,
              status,
              scheduledAt,
              startedAt: actualStartedAt,
              completedAt: actualCompletedAt,
              notes: `Trial session ${j + 1} for ${experiment.name}`,
              createdBy: seanUser.id,
            });
          }
        }
      }
    }

    const insertedTrials = await db
      .insert(schema.trials)
      .values(trials)
      .returning();
    console.log(`✅ Created ${insertedTrials.length} trials`);

    // Create trial events time series for richer dashboards
    const trialEventRows = [];
    for (const t of insertedTrials) {
      const baseStart = t.startedAt ?? new Date(Date.now() - 60 * 60 * 1000);
      const t1 = new Date(baseStart.getTime() - 2 * 60 * 1000); // 2 min before start
      const t2 = new Date(baseStart.getTime()); // start
      const t3 = new Date(baseStart.getTime() + 3 * 60 * 1000); // +3 min
      const t4 = new Date(baseStart.getTime() + 8 * 60 * 1000); // +8 min
      const t5 =
        t.completedAt ?? new Date(baseStart.getTime() + 15 * 60 * 1000); // completion

      trialEventRows.push(
        {
          trialId: t.id,
          eventType: "wizard_prompt_shown",
          actionId: null,
          timestamp: t1,
          data: { prompt: "Welcome and object demo" },
          createdBy: seanUser.id,
        },
        {
          trialId: t.id,
          eventType: "action_started",
          actionId: null,
          timestamp: t2,
          data: { label: "demo_start" },
          createdBy: seanUser.id,
        },
        {
          trialId: t.id,
          eventType: "robot_action_executed",
          actionId: null,
          timestamp: t3,
          data: { robot: "nao", action: "speak" },
          createdBy: seanUser.id,
        },
        {
          trialId: t.id,
          eventType: "action_completed",
          actionId: null,
          timestamp: t4,
          data: { label: "demo_complete" },
          createdBy: seanUser.id,
        },
        {
          trialId: t.id,
          eventType: "trial_note",
          actionId: null,
          timestamp: t5,
          data: { summary: "Session ended successfully" },
          createdBy: seanUser.id,
        },
      );
    }
    if (trialEventRows.length) {
      await db.insert(schema.trialEvents).values(trialEventRows);
      console.log(`✅ Created ${trialEventRows.length} trial events`);
    }

    // Create some activity logs for dashboard demo
    console.log("📝 Creating activity logs...");
    const activityEntries = [];

    // Study creation activities
    for (const study of insertedStudies) {
      activityEntries.push({
        studyId: study.id,
        userId: seanUser.id,
        action: "study_created",
        description: `Created study "${study.name}"`,
        createdAt: new Date(
          Date.now() - Math.random() * 7 * 24 * 60 * 60 * 1000,
        ), // Random time in last week
      });
    }

    // Experiment creation activities
    for (const experiment of insertedExperiments) {
      activityEntries.push({
        studyId: experiment.studyId,
        userId: seanUser.id,
        action: "experiment_created",
        description: `Created experiment protocol "${experiment.name}"`,
        createdAt: new Date(
          Date.now() - Math.random() * 5 * 24 * 60 * 60 * 1000,
        ), // Random time in last 5 days
      });
    }

    // Participant enrollment activities
    for (const participant of insertedParticipants) {
      activityEntries.push({
        studyId: participant.studyId,
        userId: seanUser.id,
        action: "participant_enrolled",
        description: `Enrolled participant ${participant.participantCode}`,
        createdAt: new Date(
          Date.now() - Math.random() * 3 * 24 * 60 * 60 * 1000,
        ), // Random time in last 3 days
      });
    }

    // Plugin installation activities
    for (const study of insertedStudies) {
      activityEntries.push({
        studyId: study.id,
        userId: seanUser.id,
        action: "plugin_installed",
        description: "Installed HRIStudio Core System plugin",
        createdAt: new Date(
          Date.now() - Math.random() * 2 * 24 * 60 * 60 * 1000,
        ), // Random time in last 2 days
      });
    }

    // Add some recent activities
    const firstStudy = insertedStudies[0];
    if (firstStudy) {
      activityEntries.push(
        {
          studyId: firstStudy.id,
          userId: seanUser.id,
          action: "trial_scheduled",
          description: "Scheduled new trial session",
          createdAt: new Date(Date.now() - 2 * 60 * 60 * 1000), // 2 hours ago
        },
        {
          studyId: firstStudy.id,
          userId: seanUser.id,
          action: "experiment_updated",
          description: "Updated experiment parameters",
          createdAt: new Date(Date.now() - 4 * 60 * 60 * 1000), // 4 hours ago
        },
      );
    }

    const insertedActivity = await db
      .insert(schema.activityLogs)
      .values(activityEntries)
      .returning();
    console.log(`✅ Created ${insertedActivity.length} activity log entries`);

    console.log("\n✅ Seed script completed successfully!");
    console.log("\n📊 Created:");
    console.log(`  • ${insertedRobots.length} robots`);
    console.log(`  • ${insertedUsers.length} users (Bucknell)`);
    console.log(`  • ${insertedRepos.length} plugin repositories`);
    console.log(`  • ${totalPlugins} plugins (via repository sync)`);
    console.log(`  • ${insertedStudies.length} studies`);
    console.log(`  • ${studyMemberships.length} study memberships`);
    console.log(`  • ${insertedParticipants.length} participants`);
    console.log(`  • ${insertedExperiments.length} experiments`);
    console.log(`  • ${insertedTrials.length} trials`);

    console.log("\n👤 Login credentials:");
    console.log("  Email: sean@soconnor.dev");
    console.log("  Password: password123");
    console.log("  Role: Administrator");

    console.log("\n🔄 Plugin repositories synced:");
    for (const repo of insertedRepos) {
      console.log(`  • ${repo.name}: ${repo.url}`);
    }

    console.log("\n🎯 Next steps:");
    console.log("  1. Start the development server: bun dev");
    console.log("  2. Access admin dashboard to manage repositories");
    console.log("  3. Browse plugin store to see synced plugins");
  } catch (error) {
    console.error("❌ Error running seed script:", error);
    throw error;
  } finally {
    await connection.end();
  }
}

if (import.meta.url === `file://${process.argv[1]}`) {
  void main();
}

export default main;
