import bcrypt from "bcryptjs";
import { drizzle } from "drizzle-orm/postgres-js";
import { eq, sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

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

    // Use localhost for development
    const devUrl = repoUrl.includes("core.hristudio.com")
      ? "http://localhost:3000/hristudio-core"
      : repoUrl;

    // Fetch repository metadata
    const repoResponse = await fetch(`${devUrl}/repository.json`);
    if (!repoResponse.ok) {
      throw new Error(
        `Failed to fetch repository metadata: ${repoResponse.status}`,
      );
    }
    const repoMetadata = (await repoResponse.json()) as {
      description?: string;
      author?: { name?: string };
      urls?: { git?: string };
      trust?: string;
    };

    // For core repository, create a single plugin with all block groups
    if (repoUrl.includes("core.hristudio.com")) {
      const indexResponse = await fetch(`${devUrl}/plugins/index.json`);
      if (!indexResponse.ok) {
        throw new Error(
          `Failed to fetch plugin index: ${indexResponse.status}`,
        );
      }
      const indexData = (await indexResponse.json()) as {
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

    // Create users
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
        name: "Dr. Alice Rodriguez",
        email: "alice.rodriguez@university.edu",
        password: hashedPassword,
        emailVerified: new Date(),
        image: null,
      },
      {
        name: "Dr. Bob Chen",
        email: "bob.chen@research.org",
        password: hashedPassword,
        emailVerified: new Date(),
        image: null,
      },
      {
        name: "Emily Watson",
        email: "emily.watson@lab.edu",
        password: hashedPassword,
        emailVerified: new Date(),
        image: null,
      },
      {
        name: "Dr. Maria Santos",
        email: "maria.santos@tech.edu",
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
        name: "Human-Robot Collaboration Study",
        description:
          "Investigating collaborative tasks between humans and robots in shared workspace environments",
        institution: "MIT Computer Science",
        irbProtocol: "IRB-2024-001",
        status: "active" as const,
        createdBy: seanUser.id,
      },
      {
        name: "Robot Navigation Study",
        description:
          "A comprehensive study on robot navigation and obstacle avoidance in dynamic environments",
        institution: "Stanford HCI Lab",
        irbProtocol: "IRB-2024-002",
        status: "draft" as const,
        createdBy: seanUser.id,
      },
      {
        name: "Social Robot Interaction Study",
        description:
          "Examining social dynamics between humans and humanoid robots in educational settings",
        institution: "Carnegie Mellon",
        irbProtocol: "IRB-2024-003",
        status: "active" as const,
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

    // Create basic experiments
    console.log("🧪 Creating experiments...");
    const experiments = insertedStudies.map((study, i) => ({
      studyId: study.id,
      name: `Basic Interaction Protocol ${i + 1}`,
      description: `A simple human-robot interaction experiment for ${study.name}`,
      version: 1,
      status: "ready" as const,
      estimatedDuration: 30 + i * 10,
      createdBy: seanUser.id,
    }));

    const insertedExperiments = await db
      .insert(schema.experiments)
      .values(experiments)
      .returning();
    console.log(`✅ Created ${insertedExperiments.length} experiments`);

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
    console.log(`  • ${insertedUsers.length} users`);
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
