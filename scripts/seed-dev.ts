import bcrypt from "bcryptjs";
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const sql = postgres(connectionString);
const db = drizzle(sql, { schema });

async function main() {
  console.log("🌱 Starting seed script...");

  try {
    // Clean existing data (in reverse order of dependencies)
    console.log("🧹 Cleaning existing data...");
    await db.delete(schema.trialEvents);
    await db.delete(schema.trials);
    await db.delete(schema.steps);
    await db.delete(schema.experiments);
    await db.delete(schema.participants);
    await db.delete(schema.studyMembers);
    await db.delete(schema.userSystemRoles);
    await db.delete(schema.studies);
    await db.delete(schema.users);
    await db.delete(schema.robots);

    // Create robots first
    console.log("🤖 Creating robots...");
    const robots = [
      {
        name: "NAO Robot",
        manufacturer: "SoftBank Robotics",
        model: "NAO V6",
        version: "2.8",
        capabilities: {
          speech: true,
          movement: true,
          vision: true,
          touch: true,
          leds: true,
        },
        connectionType: "wifi",
        status: "available",
      },
      {
        name: "Pepper Robot",
        manufacturer: "SoftBank Robotics",
        model: "Pepper",
        version: "2.9",
        capabilities: {
          speech: true,
          movement: true,
          vision: true,
          touch: true,
          tablet: true,
        },
        connectionType: "wifi",
        status: "available",
      },
      {
        name: "TurtleBot3",
        manufacturer: "ROBOTIS",
        model: "Burger",
        version: "1.0",
        capabilities: {
          movement: true,
          vision: true,
          lidar: true,
        },
        connectionType: "ros2",
        status: "maintenance",
      },
    ];

    await db.insert(schema.robots).values(robots);

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

    await db.insert(schema.users).values(users);

    // Assign system roles
    console.log("🎭 Assigning system roles...");
    // Get user IDs after insertion
    const insertedUsers = await db.select().from(schema.users);
    const seanUser = insertedUsers.find(
      (u) => u.email === "sean@soconnor.dev",
    )!;
    const aliceUser = insertedUsers.find(
      (u) => u.email === "alice.rodriguez@university.edu",
    )!;
    const bobUser = insertedUsers.find(
      (u) => u.email === "bob.chen@research.org",
    )!;
    const emilyUser = insertedUsers.find(
      (u) => u.email === "emily.watson@lab.edu",
    )!;
    const mariaUser = insertedUsers.find(
      (u) => u.email === "maria.santos@tech.edu",
    )!;

    const systemRoles = [
      {
        userId: seanUser.id, // Sean O'Connor
        role: "administrator" as const,
        grantedBy: seanUser.id,
      },
      {
        userId: aliceUser.id, // Alice Rodriguez
        role: "researcher" as const,
        grantedBy: seanUser.id,
      },
      {
        userId: bobUser.id, // Bob Chen
        role: "researcher" as const,
        grantedBy: seanUser.id,
      },
      {
        userId: emilyUser.id, // Emily Watson
        role: "wizard" as const,
        grantedBy: seanUser.id,
      },
      {
        userId: mariaUser.id, // Maria Santos
        role: "researcher" as const,
        grantedBy: seanUser.id,
      },
    ];

    await db.insert(schema.userSystemRoles).values(systemRoles);

    // Create studies
    console.log("📚 Creating studies...");
    const studies = [
      {
        name: "Robot-Assisted Learning in Elementary Education",
        description:
          "Investigating the effectiveness of social robots in supporting mathematics learning for elementary school students. This study examines how children interact with robotic tutors and measures learning outcomes.",
        institution: "University of Technology",
        irbProtocol: "IRB-2024-001",
        status: "active" as const,
        createdBy: aliceUser.id, // Alice Rodriguez
      },
      {
        name: "Elderly Care Robot Acceptance Study",
        description:
          "Exploring the acceptance and usability of companion robots among elderly populations in assisted living facilities. Focus on emotional responses and daily interaction patterns.",
        institution: "Research Institute for Aging",
        irbProtocol: "IRB-2024-002",
        status: "active" as const,
        createdBy: bobUser.id, // Bob Chen
      },
      {
        name: "Navigation Robot Trust Study",
        description:
          "Examining human trust in autonomous navigation robots in public spaces. Measuring behavioral indicators of trust and comfort levels during robot-guided navigation tasks.",
        institution: "Tech University",
        irbProtocol: "IRB-2024-003",
        status: "draft" as const,
        createdBy: mariaUser.id, // Maria Santos
      },
    ];

    await db.insert(schema.studies).values(studies);

    // Get study IDs after insertion
    const insertedStudies = await db.select().from(schema.studies);
    const study1 = insertedStudies.find(
      (s) => s.name === "Robot-Assisted Learning in Elementary Education",
    )!;
    const study2 = insertedStudies.find(
      (s) => s.name === "Elderly Care Robot Acceptance Study",
    )!;
    const study3 = insertedStudies.find(
      (s) => s.name === "Navigation Robot Trust Study",
    )!;

    // Create study memberships
    console.log("👥 Creating study memberships...");
    const studyMemberships = [
      // Study 1 members
      {
        studyId: study1.id,
        userId: aliceUser.id, // Alice (owner)
        role: "owner" as const,
        joinedAt: new Date(),
      },
      {
        studyId: study1.id,
        userId: emilyUser.id, // Emily (wizard)
        role: "wizard" as const,
        joinedAt: new Date(),
      },
      {
        studyId: study1.id,
        userId: seanUser.id, // Sean (researcher)
        role: "researcher" as const,
        joinedAt: new Date(),
      },

      // Study 2 members
      {
        studyId: study2.id,
        userId: bobUser.id, // Bob (owner)
        role: "owner" as const,
        joinedAt: new Date(),
      },
      {
        studyId: study2.id,
        userId: aliceUser.id, // Alice (researcher)
        role: "researcher" as const,
        joinedAt: new Date(),
      },
      {
        studyId: study2.id,
        userId: emilyUser.id, // Emily (wizard)
        role: "wizard" as const,
        joinedAt: new Date(),
      },

      // Study 3 members
      {
        studyId: study3.id,
        userId: mariaUser.id, // Maria (owner)
        role: "owner" as const,
        joinedAt: new Date(),
      },
      {
        studyId: study3.id,
        userId: seanUser.id, // Sean (researcher)
        role: "researcher" as const,
        joinedAt: new Date(),
      },
    ];

    await db.insert(schema.studyMembers).values(studyMemberships);

    // Create participants
    console.log("👤 Creating participants...");
    const participants = [
      // Study 1 participants (children)
      {
        studyId: study1.id,
        participantCode: "CHILD_001",
        name: "Alex Johnson",
        email: "parent1@email.com",
        demographics: { age: 8, gender: "male", grade: 3 },
        consentGiven: true,
        consentDate: new Date("2024-01-15"),
      },
      {
        studyId: study1.id,
        participantCode: "CHILD_002",
        name: "Emma Davis",
        email: "parent2@email.com",
        demographics: { age: 9, gender: "female", grade: 4 },
        consentGiven: true,
        consentDate: new Date("2024-01-16"),
      },
      {
        studyId: study1.id,
        participantCode: "CHILD_003",
        name: "Oliver Smith",
        email: "parent3@email.com",
        demographics: { age: 7, gender: "male", grade: 2 },
        consentGiven: true,
        consentDate: new Date("2024-01-17"),
      },

      // Study 2 participants (elderly)
      {
        studyId: study2.id,
        participantCode: "ELDERLY_001",
        name: "Margaret Thompson",
        email: "mthompson@email.com",
        demographics: {
          age: 78,
          gender: "female",
          living_situation: "assisted_living",
        },
        consentGiven: true,
        consentDate: new Date("2024-01-20"),
      },
      {
        studyId: study2.id,
        participantCode: "ELDERLY_002",
        name: "Robert Wilson",
        email: "rwilson@email.com",
        demographics: {
          age: 82,
          gender: "male",
          living_situation: "independent",
        },
        consentGiven: true,
        consentDate: new Date("2024-01-21"),
      },
      {
        studyId: study2.id,
        participantCode: "ELDERLY_003",
        name: "Dorothy Garcia",
        email: "dgarcia@email.com",
        demographics: {
          age: 75,
          gender: "female",
          living_situation: "assisted_living",
        },
        consentGiven: true,
        consentDate: new Date("2024-01-22"),
      },

      // Study 3 participants (adults)
      {
        studyId: study3.id,
        participantCode: "ADULT_001",
        name: "James Miller",
        email: "jmiller@email.com",
        demographics: { age: 28, gender: "male", occupation: "engineer" },
        consentGiven: true,
        consentDate: new Date("2024-01-25"),
      },
      {
        studyId: study3.id,
        participantCode: "ADULT_002",
        name: "Sarah Brown",
        email: "sbrown@email.com",
        demographics: { age: 34, gender: "female", occupation: "teacher" },
        consentGiven: true,
        consentDate: new Date("2024-01-26"),
      },
    ];

    await db.insert(schema.participants).values(participants);

    // Get inserted robot and participant IDs
    const insertedRobots = await db.select().from(schema.robots);
    const naoRobot = insertedRobots.find((r) => r.name === "NAO Robot")!;
    const pepperRobot = insertedRobots.find((r) => r.name === "Pepper Robot")!;

    const insertedParticipants = await db.select().from(schema.participants);

    // Create experiments
    console.log("🧪 Creating experiments...");
    const experiments = [
      {
        studyId: study1.id,
        name: "Math Tutoring Session",
        description:
          "Robot provides personalized math instruction and encouragement",
        version: 1,
        robotId: naoRobot.id, // NAO Robot
        status: "ready" as const,
        estimatedDuration: 30,
        createdBy: aliceUser.id,
      },
      {
        studyId: study1.id,
        name: "Reading Comprehension Support",
        description:
          "Robot assists with reading exercises and comprehension questions",
        version: 1,
        robotId: naoRobot.id, // NAO Robot
        status: "testing" as const,
        estimatedDuration: 25,
        createdBy: aliceUser.id,
      },
      {
        studyId: study2.id,
        name: "Daily Companion Interaction",
        description:
          "Robot engages in conversation and provides daily reminders",
        version: 1,
        robotId: pepperRobot.id, // Pepper Robot
        status: "ready" as const,
        estimatedDuration: 45,
        createdBy: bobUser.id,
      },
      {
        studyId: study2.id,
        name: "Medication Reminder Protocol",
        description: "Robot provides medication reminders and health check-ins",
        version: 1,
        robotId: pepperRobot.id, // Pepper Robot
        status: "draft" as const,
        estimatedDuration: 15,
        createdBy: bobUser.id,
      },
      {
        studyId: study3.id,
        name: "Campus Navigation Assistance",
        description:
          "Robot guides participants through campus navigation tasks",
        version: 1,
        robotId: insertedRobots.find((r) => r.name === "TurtleBot3")!.id, // TurtleBot3
        status: "ready" as const,
        estimatedDuration: 20,
        createdBy: mariaUser.id,
      },
    ];

    await db.insert(schema.experiments).values(experiments);

    // Get inserted experiment IDs
    const insertedExperiments = await db.select().from(schema.experiments);
    const experiment1 = insertedExperiments.find(
      (e) => e.name === "Math Tutoring Session",
    )!;
    const experiment2 = insertedExperiments.find(
      (e) => e.name === "Reading Comprehension Support",
    )!;
    const experiment3 = insertedExperiments.find(
      (e) => e.name === "Daily Companion Interaction",
    )!;
    const experiment4 = insertedExperiments.find(
      (e) => e.name === "Medication Reminder Protocol",
    )!;
    const experiment5 = insertedExperiments.find(
      (e) => e.name === "Campus Navigation Assistance",
    )!;

    // Create experiment steps
    console.log("📋 Creating experiment steps...");
    const steps = [
      // Math Tutoring Session steps
      {
        experimentId: experiment1.id,
        name: "Welcome and Introduction",
        description: "Robot introduces itself and explains the session",
        type: "wizard" as const,
        orderIndex: 1,
        durationEstimate: 300, // 5 minutes
        required: true,
      },
      {
        experimentId: experiment1.id,
        name: "Math Problem Presentation",
        description: "Robot presents age-appropriate math problems",
        type: "robot" as const,
        orderIndex: 2,
        durationEstimate: 1200, // 20 minutes
        required: true,
      },
      {
        experimentId: experiment1.id,
        name: "Encouragement and Feedback",
        description: "Robot provides positive feedback and encouragement",
        type: "wizard" as const,
        orderIndex: 3,
        durationEstimate: 300, // 5 minutes
        required: true,
      },

      // Daily Companion Interaction steps
      {
        experimentId: experiment3.id,
        name: "Morning Greeting",
        description: "Robot greets participant and asks about their day",
        type: "wizard" as const,
        orderIndex: 1,
        durationEstimate: 600, // 10 minutes
        required: true,
      },
      {
        experimentId: experiment3.id,
        name: "Health Check-in",
        description: "Robot asks about health and well-being",
        type: "wizard" as const,
        orderIndex: 2,
        durationEstimate: 900, // 15 minutes
        required: true,
      },
      {
        experimentId: experiment3.id,
        name: "Activity Planning",
        description: "Robot helps plan daily activities",
        type: "robot" as const,
        orderIndex: 3,
        durationEstimate: 1200, // 20 minutes
        required: true,
      },

      // Campus Navigation steps
      {
        experimentId: experiment5.id,
        name: "Navigation Instructions",
        description: "Robot explains navigation task and safety protocols",
        type: "wizard" as const,
        orderIndex: 1,
        durationEstimate: 300, // 5 minutes
        required: true,
      },
      {
        experimentId: experiment5.id,
        name: "Guided Navigation",
        description: "Robot guides participant to designated location",
        type: "robot" as const,
        orderIndex: 2,
        durationEstimate: 900, // 15 minutes
        required: true,
      },
    ];

    await db.insert(schema.steps).values(steps);

    // Get inserted step IDs
    const insertedSteps = await db.select().from(schema.steps);

    // Create trials
    console.log("🏃 Creating trials...");
    const now = new Date();
    const tomorrow = new Date(now.getTime() + 24 * 60 * 60 * 1000);
    const nextWeek = new Date(now.getTime() + 7 * 24 * 60 * 60 * 1000);

    const trials = [
      // Completed trials
      {
        experimentId: experiment1.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "CHILD_001",
        )!.id, // Alex Johnson
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "completed" as const,
        scheduledAt: new Date("2024-02-01T10:00:00Z"),
        startedAt: new Date("2024-02-01T10:05:00Z"),
        completedAt: new Date("2024-02-01T10:32:00Z"),
        duration: 27 * 60, // 27 minutes
        notes: "Participant was very engaged and showed good comprehension",
      },
      {
        experimentId: experiment1.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "CHILD_002",
        )!.id, // Emma Davis
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "completed" as const,
        scheduledAt: new Date("2024-02-01T11:00:00Z"),
        startedAt: new Date("2024-02-01T11:02:00Z"),
        completedAt: new Date("2024-02-01T11:28:00Z"),
        duration: 26 * 60, // 26 minutes
        notes:
          "Excellent performance, participant seemed to enjoy the interaction",
      },
      {
        experimentId: experiment3.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "ELDERLY_001",
        )!.id, // Margaret Thompson
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "completed" as const,
        scheduledAt: new Date("2024-02-02T14:00:00Z"),
        startedAt: new Date("2024-02-02T14:03:00Z"),
        completedAt: new Date("2024-02-02T14:48:00Z"),
        duration: 45 * 60, // 45 minutes
        notes: "Participant was initially hesitant but warmed up to the robot",
      },

      // In progress trial
      {
        experimentId: experiment1.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "CHILD_003",
        )!.id, // Sophia Martinez
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "in_progress" as const,
        scheduledAt: now,
        startedAt: new Date(now.getTime() - 10 * 60 * 1000), // Started 10 minutes ago
        completedAt: null,
        duration: null,
        notes: "Session in progress",
      },

      // Scheduled trials
      {
        experimentId: experiment3.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "ELDERLY_002",
        )!.id, // Robert Wilson
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "scheduled" as const,
        scheduledAt: tomorrow,
        startedAt: null,
        completedAt: null,
        duration: null,
        notes: null,
      },
      {
        experimentId: experiment5.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "ADULT_001",
        )!.id, // James Miller
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 1,
        status: "scheduled" as const,
        scheduledAt: nextWeek,
        startedAt: null,
        completedAt: null,
        duration: null,
        notes: null,
      },
      {
        experimentId: experiment1.id,
        participantId: insertedParticipants.find(
          (p) => p.participantCode === "CHILD_001",
        )!.id, // Alex Johnson
        wizardId: emilyUser.id, // Emily Watson
        sessionNumber: 2,
        status: "scheduled" as const,
        scheduledAt: new Date(nextWeek.getTime() + 2 * 24 * 60 * 60 * 1000),
        startedAt: null,
        completedAt: null,
        duration: null,
        notes: null,
      },
    ];

    await db.insert(schema.trials).values(trials);

    // Get inserted trial IDs
    const insertedTrials = await db.select().from(schema.trials);

    // Create trial events for completed trials
    console.log("📝 Creating trial events...");
    const trialEvents = [
      // Events for Alex Johnson's completed trial
      {
        trialId: insertedTrials[0]!.id,
        eventType: "trial_started" as const,
        timestamp: new Date("2024-02-01T10:05:00Z"),
        data: {
          experimentId: experiment1.id,
          participantId: insertedParticipants.find(
            (p) => p.participantCode === "CHILD_001",
          )!.id,
        },
      },
      {
        trialId: insertedTrials[0]!.id,
        eventType: "step_started" as const,
        timestamp: new Date("2024-02-01T10:05:30Z"),
        data: {
          stepId: insertedSteps[0]!.id,
          stepName: "Welcome and Introduction",
        },
      },
      {
        trialId: insertedTrials[0]!.id,
        eventType: "robot_action" as const,
        timestamp: new Date("2024-02-01T10:06:00Z"),
        data: {
          action: "speak",
          content: "Hello Alex! I'm excited to work on math with you today.",
        },
      },
      {
        trialId: insertedTrials[0]!.id,
        eventType: "step_completed" as const,
        timestamp: new Date("2024-02-01T10:10:30Z"),
        data: { stepId: insertedSteps[0]!.id, duration: 300 },
      },
      {
        trialId: insertedTrials[0]!.id,
        eventType: "step_started" as const,
        timestamp: new Date("2024-02-01T10:10:45Z"),
        data: {
          stepId: insertedSteps[1]!.id,
          stepName: "Math Problem Presentation",
        },
      },
      {
        trialId: insertedTrials[0]!.id,
        eventType: "trial_completed" as const,
        timestamp: new Date("2024-02-01T10:32:00Z"),
        data: { totalDuration: 27 * 60, outcome: "successful" },
      },

      // Events for Emma Davis's completed trial
      {
        trialId: insertedTrials[1]!.id,
        eventType: "trial_started" as const,
        timestamp: new Date("2024-02-01T11:02:00Z"),
        data: {
          experimentId: experiment1.id,
          participantId: insertedParticipants.find(
            (p) => p.participantCode === "CHILD_002",
          )!.id,
        },
      },
      {
        trialId: insertedTrials[1]!.id,
        eventType: "step_started" as const,
        timestamp: new Date("2024-02-01T11:02:30Z"),
        data: {
          stepId: insertedSteps[0]!.id,
          stepName: "Welcome and Introduction",
        },
      },
      {
        trialId: insertedTrials[1]!.id,
        eventType: "robot_action" as const,
        timestamp: new Date("2024-02-01T11:03:00Z"),
        data: {
          action: "speak",
          content: "Hi Emma! Are you ready for some fun math problems?",
        },
      },
      {
        trialId: insertedTrials[1]!.id,
        eventType: "trial_completed" as const,
        timestamp: new Date("2024-02-01T11:28:00Z"),
        data: { totalDuration: 26 * 60, outcome: "successful" },
      },

      // Events for Margaret Thompson's completed trial
      {
        trialId: insertedTrials[2]!.id,
        eventType: "trial_started" as const,
        timestamp: new Date("2024-02-02T14:03:00Z"),
        data: {
          experimentId: experiment3.id,
          participantId: insertedParticipants.find(
            (p) => p.participantCode === "ELDERLY_001",
          )!.id,
        },
      },
      {
        trialId: insertedTrials[2]!.id,
        eventType: "step_started" as const,
        timestamp: new Date("2024-02-02T14:03:30Z"),
        data: { stepId: insertedSteps[3]!.id, stepName: "Morning Greeting" },
      },
      {
        trialId: insertedTrials[2]!.id,
        eventType: "robot_action" as const,
        timestamp: new Date("2024-02-02T14:04:00Z"),
        data: {
          action: "speak",
          content: "Good afternoon, Margaret. How are you feeling today?",
        },
      },
      {
        trialId: insertedTrials[2]!.id,
        eventType: "trial_completed" as const,
        timestamp: new Date("2024-02-02T14:48:00Z"),
        data: { totalDuration: 45 * 60, outcome: "successful" },
      },

      // Events for in-progress trial
      {
        trialId: insertedTrials[3]!.id,
        eventType: "trial_started" as const,
        timestamp: new Date(now.getTime() - 10 * 60 * 1000),
        data: {
          experimentId: experiment1.id,
          participantId: insertedParticipants.find(
            (p) => p.participantCode === "CHILD_003",
          )!.id,
        },
      },
      {
        trialId: insertedTrials[3]!.id,
        eventType: "step_started" as const,
        timestamp: new Date(now.getTime() - 9 * 60 * 1000),
        data: {
          stepId: insertedSteps[0]!.id,
          stepName: "Welcome and Introduction",
        },
      },
      {
        trialId: insertedTrials[3]!.id,
        eventType: "step_completed" as const,
        timestamp: new Date(now.getTime() - 5 * 60 * 1000),
        data: { stepId: insertedSteps[0]!.id, duration: 240 },
      },
      {
        trialId: insertedTrials[3]!.id,
        eventType: "step_started" as const,
        timestamp: new Date(now.getTime() - 5 * 60 * 1000),
        data: {
          stepId: insertedSteps[1]!.id,
          stepName: "Math Problem Presentation",
        },
      },
    ];

    await db.insert(schema.trialEvents).values(trialEvents);

    console.log("✅ Seed script completed successfully!");
    console.log("\n📊 Created:");
    console.log(`  • ${insertedRobots.length} robots`);
    console.log(`  • ${insertedUsers.length} users`);
    console.log(`  • ${systemRoles.length} system roles`);
    console.log(`  • ${insertedStudies.length} studies`);
    console.log(`  • ${studyMemberships.length} study memberships`);
    console.log(`  • ${insertedParticipants.length} participants`);
    console.log(`  • ${insertedExperiments.length} experiments`);
    console.log(`  • ${insertedSteps.length} experiment steps`);
    console.log(`  • ${insertedTrials.length} trials`);
    console.log(`  • ${trialEvents.length} trial events`);

    console.log("\n👤 Login credentials:");
    console.log("  Email: sean@soconnor.dev");
    console.log("  Password: password123");
    console.log("  Role: Administrator");

    console.log("\n🎭 Other test users:");
    console.log("  • alice.rodriguez@university.edu (Researcher)");
    console.log("  • bob.chen@research.org (Researcher)");
    console.log("  • emily.watson@lab.edu (Wizard)");
    console.log("  • maria.santos@tech.edu (Researcher)");
    console.log("  All users have the same password: password123");
  } catch (error) {
    console.error("❌ Error running seed script:", error);
    throw error;
  } finally {
    await sql.end();
  }
}

main()
  .then(() => {
    console.log("🎉 Seed script finished successfully");
    process.exit(0);
  })
  .catch((error) => {
    console.error("💥 Seed script failed:", error);
    process.exit(1);
  });
