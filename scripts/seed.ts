#!/usr/bin/env tsx

/**
 * HRIStudio Database Seed Script
 *
 * This script seeds the database with comprehensive test data for the experiment designer,
 * including users, studies, experiments, steps, actions, and participants.
 */

import { drizzle } from "drizzle-orm/postgres-js";
import { sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

// Database connection
const connectionString =
  process.env.DATABASE_URL ??
  "postgresql://postgres:postgres@localhost:5140/hristudio";
const client = postgres(connectionString);
const db = drizzle(client, { schema });

console.log("🌱 Starting HRIStudio database seeding...");

async function clearDatabase() {
  console.log("🧹 Clearing existing data...");

  // Delete in reverse dependency order using TRUNCATE for safety
  await db.execute(sql`TRUNCATE TABLE hs_trial_event CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_action CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_step CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_trial CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_participant CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_experiment CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_study_member CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_study CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_user_system_role CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_user CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_robot CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_plugin_repository CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_plugin CASCADE`);
  await db.execute(sql`TRUNCATE TABLE hs_study_plugin CASCADE`);

  console.log("✅ Database cleared");
}

async function seedUsers() {
  console.log("👥 Seeding users...");

  const users = [
    {
      id: "01234567-89ab-cdef-0123-456789abcde0",
      name: "Sean O'Connor",
      email: "sean@soconnor.dev",
      emailVerified: new Date(),
      institution: "HRIStudio",
      activeStudyId: null,
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde1",
      name: "Dr. Sarah Chen",
      email: "sarah.chen@university.edu",
      emailVerified: new Date(),
      institution: "MIT Computer Science",
      activeStudyId: null,
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde2",
      name: "Dr. Michael Rodriguez",
      email: "m.rodriguez@research.org",
      emailVerified: new Date(),
      institution: "Stanford HCI Lab",
      activeStudyId: null,
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde3",
      name: "Emma Thompson",
      email: "emma.thompson@university.edu",
      emailVerified: new Date(),
      institution: "MIT Computer Science",
      activeStudyId: null,
    },
    {
      id: "01234567-89ab-cdef-0123-456789abcde4",
      name: "Dr. James Wilson",
      email: "james.wilson@university.edu",
      emailVerified: new Date(),
      institution: "MIT Computer Science",
      activeStudyId: null,
    },
  ];

  await db.insert(schema.users).values(users);

  // Add user roles
  const userRoles = [
    {
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "administrator" as const,
      assignedAt: new Date(),
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0", // Sean as admin
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde1",
      role: "researcher" as const,
      assignedAt: new Date(),
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher" as const,
      assignedAt: new Date(),
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde3",
      role: "wizard" as const,
      assignedAt: new Date(),
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      userId: "01234567-89ab-cdef-0123-456789abcde4",
      role: "observer" as const,
      assignedAt: new Date(),
      assignedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
  ];

  await db.insert(schema.userSystemRoles).values(userRoles);

  console.log(`✅ Created ${users.length} users with roles`);
}

async function seedStudies() {
  console.log("📚 Seeding studies...");

  const studies = [
    {
      id: "11234567-89ab-cdef-0123-456789abcde1",
      name: "Robot Navigation Assistance Study",
      description:
        "Investigating how robots can effectively assist humans with indoor navigation tasks using multimodal interaction.",
      institution: "MIT Computer Science",
      irbProtocolNumber: "IRB-2024-001",
      status: "active" as const,
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
      metadata: {
        duration: "6 months",
        targetParticipants: 50,
        robotPlatform: "TurtleBot3",
        environment: "Indoor office building",
      },
    },
    {
      id: "11234567-89ab-cdef-0123-456789abcde2",
      name: "Social Robot Interaction Patterns",
      description:
        "Exploring how different personality traits in robots affect human-robot collaboration in workplace settings.",
      institution: "Stanford HCI Lab",
      irbProtocolNumber: "IRB-2024-002",
      status: "draft" as const,
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
      metadata: {
        duration: "4 months",
        targetParticipants: 30,
        robotPlatform: "Pepper",
        environment: "Office collaboration space",
      },
    },
    {
      id: "11234567-89ab-cdef-0123-456789abcde3",
      name: "Assistive Robotics for Elderly Care",
      description:
        "Evaluating the effectiveness of companion robots in assisted living facilities for improving quality of life.",
      institution: "University of Washington",
      irbProtocolNumber: "IRB-2024-003",
      status: "completed" as const,
      createdBy: "01234567-89ab-cdef-0123-456789abcde0",
      metadata: {
        duration: "12 months",
        targetParticipants: 40,
        robotPlatform: "Companion Robot",
        environment: "Assisted living facility",
      },
    },
  ];

  await db.insert(schema.studies).values(studies);

  // Add study members
  const studyMembers = [
    // Sean as admin/owner of all studies
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner" as const,
      joinedAt: new Date(),
      invitedBy: null,
    },
    // Navigation Study Team
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde3",
      role: "wizard" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde1",
      userId: "01234567-89ab-cdef-0123-456789abcde4",
      role: "observer" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },

    // Sean as admin/owner of Social Robots Study
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde2",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner" as const,
      joinedAt: new Date(),
      invitedBy: null,
    },
    // Social Robots Study Team
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde2",
      userId: "01234567-89ab-cdef-0123-456789abcde2",
      role: "researcher" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde2",
      userId: "01234567-89ab-cdef-0123-456789abcde1",
      role: "researcher" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },

    // Sean as admin/owner of Elderly Care Study
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde3",
      userId: "01234567-89ab-cdef-0123-456789abcde0",
      role: "owner" as const,
      joinedAt: new Date(),
      invitedBy: null,
    },
    // Elderly Care Study Team
    {
      studyId: "11234567-89ab-cdef-0123-456789abcde3",
      userId: "01234567-89ab-cdef-0123-456789abcde1",
      role: "researcher" as const,
      joinedAt: new Date(),
      invitedBy: "01234567-89ab-cdef-0123-456789abcde0",
    },
  ];

  await db.insert(schema.studyMembers).values(studyMembers);

  console.log(`✅ Created ${studies.length} studies with team members`);
}

async function seedExperiments() {
  console.log("🧪 Seeding experiments...");

  const experiments = [
    {
      id: "exp-navigation-baseline",
      studyId: "study-hri-navigation",
      name: "Baseline Navigation Task",
      description:
        "Participants navigate independently without robot assistance to establish baseline performance metrics.",
      version: 1,
      robotId: null,
      status: "ready" as const,
      estimatedDuration: 15, // minutes
      createdBy: "user-researcher-1",
      metadata: {
        condition: "control",
        environment: "Building A, Floor 2",
        equipment: ["motion capture", "eye tracker"],
        instructions: "Find the conference room using only building signs",
      },
    },
    {
      id: "exp-navigation-robot",
      studyId: "study-hri-navigation",
      name: "Robot-Assisted Navigation",
      description:
        "Participants navigate with robot providing verbal and gestural guidance to test effectiveness of robot assistance.",
      version: 2,
      robotId: null,
      status: "testing" as const,
      estimatedDuration: 20,
      createdBy: "user-researcher-1",
      metadata: {
        condition: "robot_assistance",
        environment: "Building A, Floor 2",
        equipment: ["motion capture", "eye tracker", "TurtleBot3"],
        instructions: "Follow robot guidance to find the conference room",
      },
    },
    {
      id: "exp-social-personality",
      studyId: "study-social-robots",
      name: "Robot Personality Variants",
      description:
        "Testing different robot personality types (friendly, professional, neutral) in collaborative tasks.",
      version: 1,
      robotId: null,
      status: "draft" as const,
      estimatedDuration: 30,
      createdBy: "user-researcher-1",
      metadata: {
        condition: "personality_comparison",
        personalities: ["friendly", "professional", "neutral"],
        tasks: ["document review", "scheduling", "problem solving"],
      },
    },
    {
      id: "exp-elderly-companion",
      studyId: "study-elderly-assistance",
      name: "Daily Companion Interaction",
      description:
        "Evaluating robot as daily companion for elderly residents including conversation and activity reminders.",
      version: 3,
      robotId: null,
      status: "ready" as const,
      estimatedDuration: 45,
      createdBy: "user-admin-1",
      metadata: {
        condition: "companion_interaction",
        activities: ["conversation", "medication reminder", "exercise prompts"],
        duration_days: 14,
      },
    },
  ];

  await db.insert(schema.experiments).values(experiments);
  console.log(`✅ Created ${experiments.length} experiments`);
}

async function seedStepsAndActions() {
  console.log("📋 Seeding experiment steps and actions...");

  // Baseline Navigation Experiment Steps
  const baselineSteps = [
    {
      id: "step-baseline-1",
      experimentId: "exp-navigation-baseline",
      name: "Welcome & Consent",
      description:
        "Greet participant, explain study, and obtain informed consent",
      type: "wizard" as const,
      orderIndex: 0,
      durationEstimate: 300, // 5 minutes in seconds
      required: true,
      conditions: {
        environment: "lab_room",
        setup: "consent_forms_ready",
      },
    },
    {
      id: "step-baseline-2",
      experimentId: "exp-navigation-baseline",
      name: "Equipment Setup",
      description: "Attach motion capture markers and calibrate eye tracker",
      type: "wizard" as const,
      orderIndex: 1,
      durationEstimate: 180,
      required: true,
      conditions: {
        equipment: ["motion_capture", "eye_tracker"],
        calibration_required: true,
      },
    },
    {
      id: "step-baseline-3",
      experimentId: "exp-navigation-baseline",
      name: "Task Instructions",
      description: "Explain navigation task and destination to participant",
      type: "wizard" as const,
      orderIndex: 2,
      durationEstimate: 120,
      required: true,
      conditions: {
        destination: "Conference Room B-201",
        starting_point: "Building A Lobby",
      },
    },
    {
      id: "step-baseline-4",
      experimentId: "exp-navigation-baseline",
      name: "Independent Navigation",
      description:
        "Participant navigates independently while data is collected",
      type: "parallel" as const,
      orderIndex: 3,
      durationEstimate: 600,
      required: true,
      conditions: {
        data_collection: ["position", "gaze", "time"],
        assistance: "none",
      },
    },
    {
      id: "step-baseline-5",
      experimentId: "exp-navigation-baseline",
      name: "Post-Task Survey",
      description:
        "Participant completes questionnaire about navigation experience",
      type: "wizard" as const,
      orderIndex: 4,
      durationEstimate: 240,
      required: true,
      conditions: {
        survey_type: "navigation_experience",
        questions: ["difficulty", "confidence", "stress_level"],
      },
    },
  ];

  await db.insert(schema.steps).values(baselineSteps);

  // Robot-Assisted Navigation Experiment Steps
  const robotSteps = [
    {
      id: "step-robot-1",
      experimentId: "exp-navigation-robot",
      name: "Robot Introduction",
      description:
        "Robot introduces itself and explains its role as navigation assistant",
      type: "robot" as const,
      orderIndex: 0,
      durationEstimate: 180,
      required: true,
      conditions: {
        robot_behavior: "friendly_introduction",
        voice_enabled: true,
      },
    },
    {
      id: "step-robot-2",
      experimentId: "exp-navigation-robot",
      name: "Guided Navigation",
      description:
        "Robot provides turn-by-turn navigation guidance with gestures and speech",
      type: "robot" as const,
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
      id: "step-robot-3",
      experimentId: "exp-navigation-robot",
      name: "Arrival Confirmation",
      description:
        "Robot confirms successful arrival and asks about experience",
      type: "robot" as const,
      orderIndex: 2,
      durationEstimate: 120,
      required: true,
      conditions: {
        confirmation_required: true,
        feedback_collection: "immediate",
      },
    },
  ];

  await db.insert(schema.steps).values(robotSteps);

  // Social Robot Personality Steps
  const socialSteps = [
    {
      id: "step-social-1",
      experimentId: "exp-social-personality",
      name: "Personality Calibration",
      description:
        "Robot adjusts behavior based on assigned personality condition",
      type: "conditional" as const,
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
      id: "step-social-2",
      experimentId: "exp-social-personality",
      name: "Collaborative Task",
      description: "Human and robot work together on document review task",
      type: "parallel" as const,
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

  await db.insert(schema.steps).values(socialSteps);

  console.log("✅ Created experiment steps");

  // Create actions for each step
  const actions = [
    // Baseline Navigation Actions
    {
      id: "action-baseline-1-1",
      stepId: "step-baseline-1",
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
      id: "action-baseline-1-2",
      stepId: "step-baseline-1",
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
      id: "action-baseline-1-3",
      stepId: "step-baseline-1",
      name: "Obtain Consent",
      description: "Review consent form and obtain participant signature",
      type: "wizard_form",
      orderIndex: 2,
      parameters: {
        form_type: "informed_consent",
        signature_required: true,
        questions_allowed: true,
      },
    },

    // Robot Navigation Actions
    {
      id: "action-robot-1-1",
      stepId: "step-robot-1",
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
      id: "action-robot-1-2",
      stepId: "step-robot-1",
      name: "Explain Robot Role",
      description: "Robot explains how it will assist with navigation",
      type: "robot_speech",
      orderIndex: 1,
      parameters: {
        text: "I'll guide you to the conference room using gestures and directions. Please follow me and let me know if you need clarification.",
        gesture: "pointing",
        led_indicators: true,
      },
    },
    {
      id: "action-robot-2-1",
      stepId: "step-robot-2",
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
      id: "action-robot-2-2",
      stepId: "step-robot-2",
      name: "Provide Turn Instructions",
      description:
        "Robot gives clear directional instructions at decision points",
      type: "robot_speech",
      orderIndex: 1,
      parameters: {
        instruction_type: "turn_by_turn",
        gesture_coordination: true,
        confirmation_requests: ["ready_to_continue", "understand_direction"],
        adaptive_repetition: true,
      },
    },

    // Social Robot Actions
    {
      id: "action-social-1-1",
      stepId: "step-social-1",
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
    {
      id: "action-social-2-1",
      stepId: "step-social-2",
      name: "Initiate Collaboration",
      description: "Robot starts collaborative document review task",
      type: "robot_interaction",
      orderIndex: 0,
      parameters: {
        task_initiation: "collaborative",
        document_type: "research_proposal",
        review_criteria: ["clarity", "feasibility", "innovation"],
        interaction_style: "personality_dependent",
      },
    },
  ];

  await db.insert(schema.actions).values(actions);
  console.log(`✅ Created ${actions.length} actions for steps`);
}

async function seedParticipants() {
  console.log("👤 Seeding participants...");

  const participants = [
    {
      id: "participant-1",
      studyId: "study-hri-navigation",
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
      id: "participant-2",
      studyId: "study-hri-navigation",
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
      id: "participant-3",
      studyId: "study-hri-navigation",
      participantCode: "NAV003",
      name: "David Kim",
      email: "david.kim@email.com",
      demographics: {
        age: 45,
        gender: "male",
        education: "phd",
        tech_experience: "high",
        robot_experience: "high",
        mobility: "none",
      },
      consentGiven: true,
      consentDate: new Date("2024-01-17"),
      notes: "Computer science professor, very familiar with robots",
    },
    {
      id: "participant-4",
      studyId: "study-social-robots",
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
    {
      id: "participant-5",
      studyId: "study-elderly-assistance",
      participantCode: "ELD001",
      name: "Robert Thompson",
      email: "robert.thompson@email.com",
      demographics: {
        age: 72,
        gender: "male",
        education: "high_school",
        tech_experience: "low",
        robot_experience: "none",
        living_situation: "assisted_living",
        health_conditions: ["arthritis", "mild_hearing_loss"],
      },
      consentGiven: true,
      consentDate: new Date("2024-01-10"),
      notes: "Retired teacher, very social and cooperative",
    },
  ];

  await db.insert(schema.participants).values(participants);
  console.log(`✅ Created ${participants.length} participants`);
}

async function seedTrials() {
  console.log("🎯 Seeding trials...");

  const trials = [
    // Navigation Study Trials
    {
      id: "trial-nav-001",
      experimentId: "exp-navigation-baseline",
      participantId: "participant-1",
      wizardId: "user-wizard-1",
      sessionNumber: 1,
      status: "completed" as const,
      scheduledAt: new Date("2024-01-15T10:00:00"),
      startedAt: new Date("2024-01-15T10:05:00"),
      completedAt: new Date("2024-01-15T10:20:00"),
      notes: "Participant completed successfully, good baseline performance",
      metadata: {
        condition: "control",
        completion_time: 893, // seconds
        errors: 1,
        assistance_requests: 0,
      },
    },
    {
      id: "trial-nav-002",
      experimentId: "exp-navigation-robot",
      participantId: "participant-1",
      wizardId: "user-wizard-1",
      sessionNumber: 2,
      status: "completed" as const,
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
      id: "trial-nav-003",
      experimentId: "exp-navigation-baseline",
      participantId: "participant-2",
      wizardId: "user-wizard-1",
      sessionNumber: 1,
      status: "completed" as const,
      scheduledAt: new Date("2024-01-16T14:00:00"),
      startedAt: new Date("2024-01-16T14:03:00"),
      completedAt: new Date("2024-01-16T14:18:00"),
      notes: "Good spatial reasoning, minimal difficulty",
      metadata: {
        condition: "control",
        completion_time: 720,
        errors: 0,
        assistance_requests: 0,
      },
    },
    {
      id: "trial-nav-004",
      experimentId: "exp-navigation-robot",
      participantId: "participant-2",
      wizardId: "user-wizard-1",
      sessionNumber: 2,
      status: "in_progress" as const,
      scheduledAt: new Date("2024-01-16T14:30:00"),
      startedAt: new Date("2024-01-16T14:35:00"),
      completedAt: null,
      notes: "Currently in progress",
      metadata: {
        condition: "robot_assistance",
      },
    },
    {
      id: "trial-soc-001",
      experimentId: "exp-social-personality",
      participantId: "participant-4",
      wizardId: "user-wizard-1",
      sessionNumber: 1,
      status: "scheduled" as const,
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

  await db.insert(schema.trials).values(trials);
  console.log(`✅ Created ${trials.length} trials`);
}

async function seedTrialEvents() {
  console.log("📊 Seeding trial events...");

  const trialEvents = [
    // Events for completed navigation trial
    {
      id: "event-1",
      trialId: "trial-nav-001",
      stepId: "step-baseline-1",
      actionId: "action-baseline-1-1",
      eventType: "step_start" as const,
      timestamp: new Date("2024-01-15T10:05:00"),
      data: {
        step_name: "Welcome & Consent",
        wizard_id: "user-wizard-1",
      },
    },
    {
      id: "event-2",
      trialId: "trial-nav-001",
      stepId: "step-baseline-1",
      actionId: "action-baseline-1-1",
      eventType: "custom" as const,
      timestamp: new Date("2024-01-15T10:06:30"),
      data: {
        action_name: "Greet Participant",
        duration: 90,
        success: true,
      },
    },
    {
      id: "event-3",
      trialId: "trial-nav-001",
      stepId: "step-baseline-4",
      actionId: null,
      eventType: "step_start" as const,
      timestamp: new Date("2024-01-15T10:10:00"),
      data: {
        step_name: "Independent Navigation",
        starting_location: "Building A Lobby",
      },
    },
    {
      id: "event-4",
      trialId: "trial-nav-001",
      stepId: "step-baseline-4",
      actionId: null,
      eventType: "custom" as const,
      timestamp: new Date("2024-01-15T10:12:30"),
      data: {
        event_type: "wrong_turn",
        location: "Hallway B",
        correction_time: 45,
      },
    },
    {
      id: "event-5",
      trialId: "trial-nav-001",
      stepId: "step-baseline-4",
      actionId: null,
      eventType: "step_end" as const,
      timestamp: new Date("2024-01-15T10:18:53"),
      data: {
        step_name: "Independent Navigation",
        destination_reached: true,
        total_time: 533,
        path_efficiency: 0.78,
      },
    },

    // Events for robot-assisted trial
    {
      id: "event-6",
      trialId: "trial-nav-002",
      stepId: "step-robot-1",
      actionId: "action-robot-1-1",
      eventType: "custom" as const,
      timestamp: new Date("2024-01-15T10:36:30"),
      data: {
        action_name: "Robot Self-Introduction",
        robot_speech: "Hello! I'm your navigation assistant...",
        participant_response: "positive",
        engagement_level: "high",
      },
    },
    {
      id: "event-7",
      trialId: "trial-nav-002",
      stepId: "step-robot-2",
      actionId: "action-robot-2-1",
      eventType: "custom" as const,
      timestamp: new Date("2024-01-15T10:45:15"),
      data: {
        event_type: "robot_guidance",
        instruction: "Turn right at the end of this hallway",
        gesture_performed: "pointing_right",
        participant_compliance: true,
        response_time: 2.3,
      },
    },
  ];

  await db.insert(schema.trialEvents).values(trialEvents);
  console.log(`✅ Created ${trialEvents.length} trial events`);
}

async function seedRobots() {
  console.log("🤖 Seeding robots...");

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
  ];

  await db.insert(schema.robots).values(robots);
  console.log(`✅ Created ${robots.length} robots`);
}

async function main() {
  try {
    console.log("🚀 HRIStudio Database Seeding Started");
    console.log("📍 Database:", connectionString.replace(/:[^:]*@/, ":***@"));

    await clearDatabase();
    await seedUsers();
    await seedStudies();
    await seedRobots();
    await seedExperiments();
    await seedStepsAndActions();
    await seedParticipants();
    await seedTrials();
    await seedTrialEvents();

    console.log("✅ Database seeding completed successfully!");
    console.log("\n📋 Summary:");
    console.log("   👥 Users: 4 (admin, researcher, wizard, observer)");
    console.log("   📚 Studies: 3 (navigation, social robots, elderly care)");
    console.log("   🤖 Robots: 2 (TurtleBot3, NAO)");
    console.log("   🧪 Experiments: 4 (with comprehensive test scenarios)");
    console.log("   📋 Steps: 10 (covering all experiment types)");
    console.log("   ⚡ Actions: 12 (detailed robot and wizard actions)");
    console.log("   👤 Participants: 5 (diverse demographics)");
    console.log("   🎯 Trials: 5 (completed, in-progress, scheduled)");
    console.log("   📊 Events: 7 (detailed trial execution data)");
    console.log("\n🔑 Test Login Credentials:");
    console.log("   Admin: sarah.chen@university.edu");
    console.log("   Researcher: m.rodriguez@research.org");
    console.log("   Wizard: emma.thompson@university.edu");
    console.log("   Observer: james.wilson@university.edu");
    console.log("\n🧪 Test Experiment Designer with:");
    console.log("   📍 /experiments/exp-navigation-baseline/designer");
    console.log("   📍 /experiments/exp-navigation-robot/designer");
    console.log("   📍 /experiments/exp-social-personality/designer");
    console.log("\n🚀 Ready to test the experiment designer!");
  } catch (error) {
    console.error("❌ Seeding failed:", error);
    process.exit(1);
  } finally {
    await client.end();
  }
}

// Run the seeding
main().catch(console.error);
