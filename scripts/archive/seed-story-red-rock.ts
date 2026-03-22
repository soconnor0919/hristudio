import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../../src/server/db/schema";
import { sql } from "drizzle-orm";
import { v4 as uuidv4 } from "uuid";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
  console.log("🌱 Seeding 'Story: Red Rock' experiment...");

  try {
    // 1. Find Admin User & Study
    const user = await db.query.users.findFirst({
      where: (users, { eq }) => eq(users.email, "sean@soconnor.dev"),
    });
    if (!user) throw new Error("Admin user 'sean@soconnor.dev' not found.");

    const study = await db.query.studies.findFirst({
      where: (studies, { eq }) => eq(studies.name, "Comparative WoZ Study"),
    });
    if (!study) throw new Error("Study 'Comparative WoZ Study' not found.");

    const robot = await db.query.robots.findFirst({
      where: (robots, { eq }) => eq(robots.name, "NAO6"),
    });
    if (!robot) throw new Error("Robot 'NAO6' not found.");

    // 2. Create Experiment
    const [experiment] = await db
      .insert(schema.experiments)
      .values({
        studyId: study.id,
        name: "Story: Red Rock",
        description:
          "A story about a red rock on Mars with comprehension check and branching.",
        version: 1,
        status: "draft",
        robotId: robot.id,
        createdBy: user.id,
      })
      .returning();

    if (!experiment) throw new Error("Failed to create experiment");
    console.log(`✅ Created Experiment: ${experiment.id}`);

    // 3. Create Steps (in reverse for ID references if needed, but we'll use uuid placeholders)
    const conclusionId = uuidv4();
    const branchAId = uuidv4();
    const branchBId = uuidv4();
    const checkId = uuidv4();

    // Step 1: The Hook
    const [step1] = await db
      .insert(schema.steps)
      .values({
        experimentId: experiment.id,
        name: "The Hook",
        type: "wizard",
        orderIndex: 0,
      })
      .returning();

    // Step 2: The Narrative
    const [step2] = await db
      .insert(schema.steps)
      .values({
        experimentId: experiment.id,
        name: "The Narrative",
        type: "wizard",
        orderIndex: 1,
      })
      .returning();

    // Step 3: Comprehension Check (Conditional)
    const [step3] = await db
      .insert(schema.steps)
      .values({
        id: checkId,
        experimentId: experiment.id,
        name: "Comprehension Check",
        type: "conditional",
        orderIndex: 2,
        conditions: {
          variable: "last_wizard_response",
          options: [
            {
              label: "Answer: Red (Correct)",
              value: "Red",
              variant: "default",
              nextStepId: branchAId,
            },
            {
              label: "Answer: Other (Incorrect)",
              value: "Incorrect",
              variant: "destructive",
              nextStepId: branchBId,
            },
          ],
        },
      })
      .returning();

    // Step 4: Branch A (Correct)
    const [step4] = await db
      .insert(schema.steps)
      .values({
        id: branchAId,
        experimentId: experiment.id,
        name: "Branch A: Correct Response",
        type: "wizard",
        orderIndex: 3,
        conditions: { nextStepId: conclusionId }, // SKIP BRANCH B
      })
      .returning();

    // Step 5: Branch B (Incorrect)
    const [step5] = await db
      .insert(schema.steps)
      .values({
        id: branchBId,
        experimentId: experiment.id,
        name: "Branch B: Incorrect Response",
        type: "wizard",
        orderIndex: 4,
        conditions: { nextStepId: conclusionId },
      })
      .returning();

    // Step 6: Conclusion
    const [step6] = await db
      .insert(schema.steps)
      .values({
        id: conclusionId,
        experimentId: experiment.id,
        name: "Conclusion",
        type: "wizard",
        orderIndex: 5,
      })
      .returning();

    // 4. Create Actions

    // The Hook
    await db.insert(schema.actions).values([
      {
        stepId: step1!.id,
        name: "Say Hello",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Hello! Are you ready for a story?" },
      },
      {
        stepId: step1!.id,
        name: "Wave",
        type: "nao6-ros2.move_arm",
        orderIndex: 1,
        parameters: { arm: "right", shoulder_pitch: 0.5 },
      },
    ]);

    // The Narrative
    await db.insert(schema.actions).values([
      {
        stepId: step2!.id,
        name: "The Story",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: {
          text: "Once, a traveler went to Mars. He found a bright red rock that glowed.",
        },
      },
      {
        stepId: step2!.id,
        name: "Look Left",
        type: "nao6-ros2.turn_head",
        orderIndex: 1,
        parameters: { yaw: 0.5, speed: 0.3 },
      },
      {
        stepId: step2!.id,
        name: "Look Right",
        type: "nao6-ros2.turn_head",
        orderIndex: 2,
        parameters: { yaw: -0.5, speed: 0.3 },
      },
    ]);

    // Comprehension Check
    await db.insert(schema.actions).values([
      {
        stepId: step3!.id,
        name: "Ask Color",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "What color was the rock I found on Mars?" },
      },
      {
        stepId: step3!.id,
        name: "Wait for Color",
        type: "wizard_wait_for_response",
        orderIndex: 1,
        parameters: {
          options: ["Red", "Blue", "Green", "Incorrect"],
          prompt_text: "What color did the participant say?",
        },
      },
    ]);

    // Branch A (Using say_with_emotion)
    await db
      .insert(schema.actions)
      .values([
        {
          stepId: step4!.id,
          name: "Happy Response",
          type: "nao6-ros2.say_with_emotion",
          orderIndex: 0,
          parameters: {
            text: "Exacty! It was a glowing red rock.",
            emotion: "happy",
          },
        },
      ]);

    // Branch B
    await db.insert(schema.actions).values([
      {
        stepId: step5!.id,
        name: "Correct them",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "Actually, it was red." },
      },
      {
        stepId: step5!.id,
        name: "Shake Head",
        type: "nao6-ros2.turn_head",
        orderIndex: 1,
        parameters: { yaw: 0.3, speed: 0.5 },
      },
    ]);

    // Conclusion
    await db.insert(schema.actions).values([
      {
        stepId: step6!.id,
        name: "Final Goodbye",
        type: "nao6-ros2.say_text",
        orderIndex: 0,
        parameters: { text: "That is all for today. Goodbye!" },
      },
      {
        stepId: step6!.id,
        name: "Rest",
        type: "nao6-ros2.move_arm",
        orderIndex: 1,
        parameters: { shoulder_pitch: 1.5 },
      },
    ]);

    console.log("✅ Seed completed successfully!");
  } catch (err) {
    console.error("❌ Seed failed:", err);
    process.exit(1);
  } finally {
    await connection.end();
  }
}

main();
