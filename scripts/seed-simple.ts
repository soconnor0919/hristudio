#!/usr/bin/env tsx

/**
 * HRIStudio Database Seed Script (Simplified)
 *
 * This script seeds the database with comprehensive test data for the experiment designer,
 * using raw SQL to avoid NextAuth import issues.
 */

import bcrypt from "bcryptjs";
import postgres from "postgres";

// Database connection
const connectionString =
  process.env.DATABASE_URL ??
  "postgresql://postgres:postgres@localhost:5140/hristudio";
const sql = postgres(connectionString);

console.log("🌱 Starting HRIStudio database seeding...");

async function clearDatabase() {
  console.log("🧹 Clearing existing data...");

  // Delete in reverse dependency order
  await sql`DELETE FROM hs_trial_event`;
  await sql`DELETE FROM hs_action`;
  await sql`DELETE FROM hs_step`;
  await sql`DELETE FROM hs_trial`;
  await sql`DELETE FROM hs_participant`;
  await sql`DELETE FROM hs_experiment`;
  await sql`DELETE FROM hs_study_member`;
  await sql`DELETE FROM hs_study`;
  await sql`DELETE FROM hs_user_system_role`;
  await sql`DELETE FROM hs_user`;

  console.log("✅ Database cleared");
}

async function seedUsers() {
  console.log("👥 Seeding users...");

  // Hash password "password123" for all test users
  const hashedPassword = await bcrypt.hash("password123", 12);

  const users = [
    {
      id: "550e8400-e29b-41d4-a716-446655440001",
      name: "Dr. Sarah Chen",
      email: "sarah.chen@university.edu",
      emailVerified: new Date(),
      password: hashedPassword,
    },
    {
      id: "550e8400-e29b-41d4-a716-446655440002",
      name: "Dr. Michael Rodriguez",
      email: "m.rodriguez@research.org",
      emailVerified: new Date(),
      password: hashedPassword,
    },
    {
      id: "550e8400-e29b-41d4-a716-446655440003",
      name: "Emma Thompson",
      email: "emma.thompson@university.edu",
      emailVerified: new Date(),
      password: hashedPassword,
    },
    {
      id: "550e8400-e29b-41d4-a716-446655440004",
      name: "Dr. James Wilson",
      email: "james.wilson@university.edu",
      emailVerified: new Date(),
      password: hashedPassword,
    },
  ];

  for (const user of users) {
    await sql`
      INSERT INTO hs_user (id, name, email, email_verified, password, created_at, updated_at)
      VALUES (${user.id}, ${user.name}, ${user.email}, ${user.emailVerified}, ${user.password}, NOW(), NOW())
    `;
  }

  // Add user roles
  const userRoles = [
    {
      userId: "550e8400-e29b-41d4-a716-446655440001",
      role: "administrator",
    },
    {
      userId: "550e8400-e29b-41d4-a716-446655440002",
      role: "researcher",
    },
    {
      userId: "550e8400-e29b-41d4-a716-446655440003",
      role: "wizard",
    },
    {
      userId: "550e8400-e29b-41d4-a716-446655440004",
      role: "observer",
    },
  ];

  for (const userRole of userRoles) {
    await sql`
      INSERT INTO hs_user_system_role (user_id, role, granted_at)
      VALUES (${userRole.userId}, ${userRole.role}, NOW())
    `;
  }

  console.log(`✅ Created ${users.length} users with roles`);
}

async function seedStudies() {
  console.log("📚 Seeding studies...");

  const studies = [
    {
      id: "650e8400-e29b-41d4-a716-446655440001",
      name: "Robot Navigation Assistance Study",
      description:
        "Investigating how robots can effectively assist humans with indoor navigation tasks using multimodal interaction.",
      institution: "MIT Computer Science",
      irbProtocolNumber: "IRB-2024-001",
      status: "active",
      createdBy: "550e8400-e29b-41d4-a716-446655440002",
      metadata: {
        duration: "6 months",
        targetParticipants: 50,
        robotPlatform: "TurtleBot3",
        environment: "Indoor office building",
      },
    },
    {
      id: "650e8400-e29b-41d4-a716-446655440002",
      name: "Social Robot Interaction Patterns",
      description:
        "Exploring how different personality traits in robots affect human-robot collaboration in workplace settings.",
      institution: "Stanford HCI Lab",
      irbProtocolNumber: "IRB-2024-002",
      status: "draft",
      createdBy: "550e8400-e29b-41d4-a716-446655440002",
      metadata: {
        duration: "4 months",
        targetParticipants: 30,
        robotPlatform: "Pepper",
        environment: "Office collaboration space",
      },
    },
    {
      id: "650e8400-e29b-41d4-a716-446655440003",
      name: "Elderly Care Assistant Robot Study",
      description:
        "Evaluating the effectiveness of companion robots in assisted living facilities for elderly residents.",
      institution: "MIT Computer Science",
      irbProtocolNumber: "IRB-2024-003",
      status: "completed",
      createdBy: "550e8400-e29b-41d4-a716-446655440001",
      metadata: {
        duration: "8 months",
        targetParticipants: 25,
        robotPlatform: "NAO",
        environment: "Assisted living facility",
      },
    },
  ];

  for (const study of studies) {
    await sql`
      INSERT INTO hs_study (id, name, description, institution, irb_protocol, status, created_by, metadata, created_at, updated_at)
      VALUES (${study.id}, ${study.name}, ${study.description}, ${study.institution}, ${study.irbProtocolNumber}, ${study.status}, ${study.createdBy}, ${JSON.stringify(study.metadata)}, NOW(), NOW())
    `;
  }

  // Add study members
  const studyMembers = [
    // Navigation Study Team
    {
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      userId: "550e8400-e29b-41d4-a716-446655440002",
      role: "owner",
    },
    {
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      userId: "550e8400-e29b-41d4-a716-446655440003",
      role: "wizard",
    },
    {
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      userId: "550e8400-e29b-41d4-a716-446655440004",
      role: "observer",
    },
    // Social Robots Study Team
    {
      studyId: "650e8400-e29b-41d4-a716-446655440002",
      userId: "550e8400-e29b-41d4-a716-446655440002",
      role: "owner",
    },
    {
      studyId: "650e8400-e29b-41d4-a716-446655440002",
      userId: "550e8400-e29b-41d4-a716-446655440001",
      role: "researcher",
    },
    // Elderly Care Study Team
    {
      studyId: "650e8400-e29b-41d4-a716-446655440003",
      userId: "550e8400-e29b-41d4-a716-446655440001",
      role: "owner",
    },
  ];

  for (const member of studyMembers) {
    await sql`
      INSERT INTO hs_study_member (study_id, user_id, role, joined_at)
      VALUES (${member.studyId}, ${member.userId}, ${member.role}, NOW())
    `;
  }

  console.log(`✅ Created ${studies.length} studies with team members`);
}

async function seedExperiments() {
  console.log("🧪 Seeding experiments...");

  const experiments = [
    {
      id: "750e8400-e29b-41d4-a716-446655440001",
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      name: "Baseline Navigation Task",
      description:
        "Participants navigate independently without robot assistance to establish baseline performance metrics.",
      version: 1,
      status: "ready",
      estimatedDuration: 15,
      createdBy: "550e8400-e29b-41d4-a716-446655440002",
      metadata: {
        condition: "control",
        environment: "Building A, Floor 2",
        equipment: ["motion capture", "eye tracker"],
        instructions: "Find the conference room using only building signs",
      },
    },
    {
      id: "750e8400-e29b-41d4-a716-446655440002",
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      name: "Robot-Assisted Navigation",
      description:
        "Participants navigate with robot providing verbal and gestural guidance to test effectiveness of robot assistance.",
      version: 2,
      status: "testing",
      estimatedDuration: 20,
      createdBy: "550e8400-e29b-41d4-a716-446655440002",
      metadata: {
        condition: "robot_assistance",
        environment: "Building A, Floor 2",
        equipment: ["motion capture", "eye tracker", "TurtleBot3"],
        instructions: "Follow robot guidance to find the conference room",
      },
    },
    {
      id: "750e8400-e29b-41d4-a716-446655440003",
      studyId: "650e8400-e29b-41d4-a716-446655440002",
      name: "Robot Personality Variants",
      description:
        "Testing different robot personality types (friendly, professional, neutral) in collaborative tasks.",
      version: 1,
      status: "draft",
      estimatedDuration: 30,
      createdBy: "550e8400-e29b-41d4-a716-446655440002",
      metadata: {
        condition: "personality_comparison",
        personalities: ["friendly", "professional", "neutral"],
        tasks: ["document review", "scheduling", "problem solving"],
      },
    },
    {
      id: "750e8400-e29b-41d4-a716-446655440004",
      studyId: "650e8400-e29b-41d4-a716-446655440003",
      name: "Daily Companion Interaction",
      description:
        "Evaluating robot as daily companion for elderly residents including conversation and activity reminders.",
      version: 3,
      status: "ready",
      estimatedDuration: 45,
      createdBy: "550e8400-e29b-41d4-a716-446655440001",
      metadata: {
        condition: "companion_interaction",
        activities: ["conversation", "medication reminder", "exercise prompts"],
        duration_days: 14,
      },
    },
  ];

  for (const experiment of experiments) {
    await sql`
      INSERT INTO hs_experiment (id, study_id, name, description, version, status, estimated_duration, created_by, metadata, created_at, updated_at)
      VALUES (${experiment.id}, ${experiment.studyId}, ${experiment.name}, ${experiment.description}, ${experiment.version}, ${experiment.status}, ${experiment.estimatedDuration}, ${experiment.createdBy}, ${JSON.stringify(experiment.metadata)}, NOW(), NOW())
    `;
  }

  console.log(`✅ Created ${experiments.length} experiments`);
}

async function seedStepsAndActions() {
  console.log("📋 Seeding experiment steps and actions...");

  // Baseline Navigation Experiment Steps
  const steps = [
    {
      id: "850e8400-e29b-41d4-a716-446655440001",
      experimentId: "750e8400-e29b-41d4-a716-446655440001",
      name: "Welcome & Consent",
      description:
        "Greet participant, explain study, and obtain informed consent",
      type: "wizard",
      orderIndex: 0,
      durationEstimate: 300,
      required: true,
      conditions: {
        environment: "lab_room",
        setup: "consent_forms_ready",
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440002",
      experimentId: "750e8400-e29b-41d4-a716-446655440001",
      name: "Equipment Setup",
      description: "Attach motion capture markers and calibrate eye tracker",
      type: "wizard",
      orderIndex: 1,
      durationEstimate: 180,
      required: true,
      conditions: {
        equipment: ["motion_capture", "eye_tracker"],
        calibration_required: true,
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440003",
      experimentId: "750e8400-e29b-41d4-a716-446655440001",
      name: "Task Instructions",
      description: "Explain navigation task and destination to participant",
      type: "wizard",
      orderIndex: 2,
      durationEstimate: 120,
      required: true,
      conditions: {
        destination: "Conference Room B-201",
        starting_point: "Building A Lobby",
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440004",
      experimentId: "750e8400-e29b-41d4-a716-446655440001",
      name: "Independent Navigation",
      description:
        "Participant navigates independently while data is collected",
      type: "parallel",
      orderIndex: 3,
      durationEstimate: 600,
      required: true,
      conditions: {
        data_collection: ["position", "gaze", "time"],
        assistance: "none",
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440005",
      experimentId: "750e8400-e29b-41d4-a716-446655440002",
      name: "Robot Introduction",
      description:
        "Robot introduces itself and explains its role as navigation assistant",
      type: "robot",
      orderIndex: 0,
      durationEstimate: 180,
      required: true,
      conditions: {
        robot_behavior: "friendly_introduction",
        voice_enabled: true,
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440006",
      experimentId: "750e8400-e29b-41d4-a716-446655440002",
      name: "Guided Navigation",
      description:
        "Robot provides turn-by-turn navigation guidance with gestures and speech",
      type: "robot",
      orderIndex: 1,
      durationEstimate: 480,
      required: true,
      conditions: {
        guidance_type: "multimodal",
        gestures: true,
        speech: true,
        adaptation: "user_pace",
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440007",
      experimentId: "750e8400-e29b-41d4-a716-446655440003",
      name: "Personality Calibration",
      description:
        "Robot adjusts behavior based on assigned personality condition",
      type: "conditional",
      orderIndex: 0,
      durationEstimate: 60,
      required: true,
      conditions: {
        personality_variants: ["friendly", "professional", "neutral"],
        behavior_parameters: {
          friendly: { warmth: 0.8, formality: 0.3 },
          professional: { warmth: 0.4, formality: 0.9 },
          neutral: { warmth: 0.5, formality: 0.5 },
        },
      },
    },
    {
      id: "850e8400-e29b-41d4-a716-446655440008",
      experimentId: "750e8400-e29b-41d4-a716-446655440003",
      name: "Collaborative Task",
      description: "Human and robot work together on document review task",
      type: "parallel",
      orderIndex: 1,
      durationEstimate: 1200,
      required: true,
      conditions: {
        task_type: "document_review",
        collaboration_level: "equal_partners",
        performance_metrics: ["accuracy", "efficiency", "satisfaction"],
      },
    },
  ];

  for (const step of steps) {
    await sql`
      INSERT INTO hs_step (id, experiment_id, name, description, type, order_index, duration_estimate, required, conditions, created_at, updated_at)
      VALUES (${step.id}, ${step.experimentId}, ${step.name}, ${step.description}, ${step.type}, ${step.orderIndex}, ${step.durationEstimate}, ${step.required}, ${JSON.stringify(step.conditions)}, NOW(), NOW())
    `;
  }

  console.log("✅ Created experiment steps");

  // Create actions for each step
  const actions = [
    {
      id: "950e8400-e29b-41d4-a716-446655440001",
      stepId: "850e8400-e29b-41d4-a716-446655440001",
      name: "Greet Participant",
      description: "Welcome participant and introduce research team",
      type: "wizard_speech",
      orderIndex: 0,
      parameters: {
        script:
          "Hello! Welcome to our navigation study. I'm [NAME] and I'll be guiding you through today's session.",
        tone: "friendly_professional",
      },
    },
    {
      id: "950e8400-e29b-41d4-a716-446655440002",
      stepId: "850e8400-e29b-41d4-a716-446655440001",
      name: "Explain Study",
      description: "Provide overview of study purpose and procedures",
      type: "wizard_speech",
      orderIndex: 1,
      parameters: {
        script:
          "Today we're studying how people navigate indoor environments. You'll be asked to find a specific location in the building.",
        documentation_required: true,
      },
    },
    {
      id: "950e8400-e29b-41d4-a716-446655440003",
      stepId: "850e8400-e29b-41d4-a716-446655440005",
      name: "Robot Self-Introduction",
      description: "Robot introduces itself with friendly demeanor",
      type: "robot_speech",
      orderIndex: 0,
      parameters: {
        text: "Hello! I'm your navigation assistant. My name is Robi and I'm here to help you find your destination.",
        gesture: "wave",
        eye_contact: true,
        voice_parameters: {
          pitch: 0.7,
          speed: 0.8,
          emotion: "friendly",
        },
      },
    },
    {
      id: "950e8400-e29b-41d4-a716-446655440004",
      stepId: "850e8400-e29b-41d4-a716-446655440006",
      name: "Start Navigation",
      description: "Robot begins guiding participant toward destination",
      type: "robot_movement",
      orderIndex: 0,
      parameters: {
        movement_type: "lead",
        speed: "slow_human_pace",
        path_planning: "optimal_with_explanations",
        safety_distance: 1.5,
      },
    },
    {
      id: "950e8400-e29b-41d4-a716-446655440005",
      stepId: "850e8400-e29b-41d4-a716-446655440007",
      name: "Load Personality Profile",
      description: "Configure robot behavior based on personality condition",
      type: "robot_config",
      orderIndex: 0,
      parameters: {
        config_type: "personality_parameters",
        profiles: {
          friendly: {
            greeting_style: "warm",
            speech_patterns: "casual",
            gesture_frequency: "high",
          },
          professional: {
            greeting_style: "formal",
            speech_patterns: "business",
            gesture_frequency: "moderate",
          },
          neutral: {
            greeting_style: "standard",
            speech_patterns: "neutral",
            gesture_frequency: "low",
          },
        },
      },
    },
  ];

  for (const action of actions) {
    await sql`
      INSERT INTO hs_action (id, step_id, name, description, type, order_index, parameters, created_at, updated_at)
      VALUES (${action.id}, ${action.stepId}, ${action.name}, ${action.description}, ${action.type}, ${action.orderIndex}, ${JSON.stringify(action.parameters)}, NOW(), NOW())
    `;
  }

  console.log(`✅ Created ${actions.length} actions for steps`);
}

async function seedParticipants() {
  console.log("👤 Seeding participants...");

  const participants = [
    {
      id: "a50e8400-e29b-41d4-a716-446655440001",
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      participantCode: "NAV001",
      name: "Alex Johnson",
      email: "alex.johnson@email.com",
      demographics: {
        age: 28,
        gender: "non-binary",
        education: "bachelor",
        tech_experience: "high",
        robot_experience: "medium",
        mobility: "none",
      },
      consentGiven: true,
      consentDate: new Date("2024-01-15"),
      notes: "Interested in robotics, works in tech industry",
    },
    {
      id: "a50e8400-e29b-41d4-a716-446655440002",
      studyId: "650e8400-e29b-41d4-a716-446655440001",
      participantCode: "NAV002",
      name: "Maria Santos",
      email: "maria.santos@email.com",
      demographics: {
        age: 34,
        gender: "female",
        education: "master",
        tech_experience: "medium",
        robot_experience: "low",
        mobility: "none",
      },
      consentGiven: true,
      consentDate: new Date("2024-01-16"),
      notes: "Architecture background, good spatial reasoning",
    },
    {
      id: "a50e8400-e29b-41d4-a716-446655440003",
      studyId: "650e8400-e29b-41d4-a716-446655440002",
      participantCode: "SOC001",
      name: "Jennifer Liu",
      email: "jennifer.liu@email.com",
      demographics: {
        age: 29,
        gender: "female",
        education: "bachelor",
        tech_experience: "medium",
        robot_experience: "low",
        work_environment: "office",
      },
      consentGiven: true,
      consentDate: new Date("2024-01-20"),
      notes: "Project manager, interested in workplace automation",
    },
  ];

  for (const participant of participants) {
    await sql`
      INSERT INTO hs_participant (id, study_id, participant_code, name, email, demographics, consent_given, consent_date, notes, created_at, updated_at)
      VALUES (${participant.id}, ${participant.studyId}, ${participant.participantCode}, ${participant.name}, ${participant.email}, ${JSON.stringify(participant.demographics)}, ${participant.consentGiven}, ${participant.consentDate}, ${participant.notes}, NOW(), NOW())
    `;
  }

  console.log(`✅ Created ${participants.length} participants`);
}

async function seedTrials() {
  console.log("🎯 Seeding trials...");

  const trials = [
    {
      id: "b50e8400-e29b-41d4-a716-446655440001",
      experimentId: "750e8400-e29b-41d4-a716-446655440001",
      participantId: "a50e8400-e29b-41d4-a716-446655440001",
      wizardId: "550e8400-e29b-41d4-a716-446655440003",
      sessionNumber: 1,
      status: "completed",
      scheduledAt: new Date("2024-01-15T10:00:00"),
      startedAt: new Date("2024-01-15T10:05:00"),
      completedAt: new Date("2024-01-15T10:20:00"),
      notes: "Participant completed successfully, good baseline performance",
      metadata: {
        condition: "control",
        completion_time: 893,
        errors: 1,
        assistance_requests: 0,
      },
    },
    {
      id: "b50e8400-e29b-41d4-a716-446655440002",
      experimentId: "750e8400-e29b-41d4-a716-446655440002",
      participantId: "a50e8400-e29b-41d4-a716-446655440001",
      wizardId: "550e8400-e29b-41d4-a716-446655440003",
      sessionNumber: 2,
      status: "completed",
      scheduledAt: new Date("2024-01-15T10:30:00"),
      startedAt: new Date("2024-01-15T10:35:00"),
      completedAt: new Date("2024-01-15T10:58:00"),
      notes: "Robot assistance worked well, participant very satisfied",
      metadata: {
        condition: "robot_assistance",
        completion_time: 654,
        errors: 0,
        assistance_requests: 2,
        robot_performance: "excellent",
      },
    },
    {
      id: "b50e8400-e29b-41d4-a716-446655440003",
      experimentId: "750e8400-e29b-41d4-a716-446655440003",
      participantId: "a50e8400-e29b-41d4-a716-446655440003",
      wizardId: "550e8400-e29b-41d4-a716-446655440003",
      sessionNumber: 1,
      status: "scheduled",
      scheduledAt: new Date("2024-01-25T11:00:00"),
      startedAt: null,
      completedAt: null,
      notes: "Personality condition: friendly",
      metadata: {
        condition: "friendly_personality",
        personality_type: "friendly",
      },
    },
  ];

  for (const trial of trials) {
    await sql`
      INSERT INTO hs_trial (id, experiment_id, participant_id, wizard_id, session_number, status, scheduled_at, started_at, completed_at, notes, metadata, created_at, updated_at)
      VALUES (${trial.id}, ${trial.experimentId}, ${trial.participantId}, ${trial.wizardId}, ${trial.sessionNumber}, ${trial.status}, ${trial.scheduledAt}, ${trial.startedAt}, ${trial.completedAt}, ${trial.notes}, ${JSON.stringify(trial.metadata)}, NOW(), NOW())
    `;
  }

  console.log(`✅ Created ${trials.length} trials`);
}

async function main() {
  try {
    console.log("🚀 HRIStudio Database Seeding Started");
    console.log("📍 Database:", connectionString.replace(/:[^:]*@/, ":***@"));

    await clearDatabase();
    await seedUsers();
    await seedStudies();
    await seedExperiments();
    await seedStepsAndActions();
    await seedParticipants();
    await seedTrials();

    console.log("✅ Database seeding completed successfully!");
    console.log("\n📋 Summary:");
    console.log("   👥 Users: 4 (admin, researcher, wizard, observer)");
    console.log("   📚 Studies: 3 (navigation, social robots, elderly care)");
    console.log("   🧪 Experiments: 4 (with comprehensive test scenarios)");
    console.log("   📋 Steps: 8 (covering all experiment types)");
    console.log("   ⚡ Actions: 5 (detailed robot and wizard actions)");
    console.log("   👤 Participants: 3 (diverse demographics)");
    console.log("   🎯 Trials: 3 (completed, scheduled)");
    console.log("🔑 Test Login Credentials:");
    console.log("   Admin: sarah.chen@university.edu / password123");
    console.log("   Researcher: m.rodriguez@research.org / password123");
    console.log("   Wizard: emma.thompson@university.edu / password123");
    console.log("   Observer: james.wilson@university.edu / password123");
    console.log("\n🧪 Test Experiment Designer with:");
    console.log(
      "   📍 /experiments/750e8400-e29b-41d4-a716-446655440001/designer",
    );
    console.log(
      "   📍 /experiments/750e8400-e29b-41d4-a716-446655440002/designer",
    );
    console.log(
      "   📍 /experiments/750e8400-e29b-41d4-a716-446655440003/designer",
    );
    console.log("\n🚀 Ready to test the experiment designer!");
  } catch (error) {
    console.error("❌ Seeding failed:", error);
    process.exit(1);
  } finally {
    await sql.end();
  }
}

// Run the seeding
main().catch(console.error);
