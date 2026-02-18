import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../../src/server/db/schema";
import { sql } from "drizzle-orm";

// Database connection
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
    console.log("🌱 Seeding 'Control Flow Demo' experiment...");

    try {
        // 1. Find Admin User & Study
        const user = await db.query.users.findFirst({
            where: (users, { eq }) => eq(users.email, "sean@soconnor.dev")
        });
        if (!user) throw new Error("Admin user 'sean@soconnor.dev' not found. Run seed-dev.ts first.");

        const study = await db.query.studies.findFirst({
            where: (studies, { eq }) => eq(studies.name, "Comparative WoZ Study")
        });
        if (!study) throw new Error("Study 'Comparative WoZ Study' not found. Run seed-dev.ts first.");

        // Find Robot
        const robot = await db.query.robots.findFirst({
            where: (robots, { eq }) => eq(robots.name, "NAO6")
        });
        if (!robot) throw new Error("Robot 'NAO6' not found. Run seed-dev.ts first.");


        // 2. Create Experiment
        const [experiment] = await db.insert(schema.experiments).values({
            studyId: study.id,
            name: "Control Flow Demo",
            description: "Demonstration of enhanced control flow actions: Sequence, Parallel, Wait, Loop, Branch.",
            version: 1,
            status: "draft",
            robotId: robot.id,
            createdBy: user.id,
        }).returning();

        if (!experiment) throw new Error("Failed to create experiment");
        console.log(`✅ Created Experiment: ${experiment.id}`);

        // 3. Create Steps

        // Step 1: Sequence & Parallel
        const [step1] = await db.insert(schema.steps).values({
            experimentId: experiment.id,
            name: "Complex Action Structures",
            description: "Demonstrating Sequence and Parallel groups",
            type: "robot",
            orderIndex: 0,
            required: true,
            durationEstimate: 30
        }).returning();

        // Step 2: Loops & Waits
        const [step2] = await db.insert(schema.steps).values({
            experimentId: experiment.id,
            name: "Repetition & Delays",
            description: "Demonstrating Loop and Wait actions",
            type: "robot",
            orderIndex: 1,
            required: true,
            durationEstimate: 45
        }).returning();

        // 4. Create Actions

        // --- Step 1 Actions ---

        // Top-level Sequence
        const seqId = `seq-${Date.now()}`;
        await db.insert(schema.actions).values({
            stepId: step1!.id,
            name: "Introduction Sequence",
            type: "sequence", // New type
            orderIndex: 0,
            parameters: {},
            pluginId: "hristudio-core",
            category: "control",
            // No explicit children column in schema? 
            // Wait, schema.actions has "children" as jsonb or it's a recursive relationship?
            // Let's check schema/types.
            // Looking at ActionChip, it expects `action.children`. 
            // In DB, it's likely stored in `children` jsonb column if it exists, OR we need to perform recursive inserts if schema supports parentId.
            // Checking `types.ts` or schema...
            // Assuming flat list references for now or JSONB. 
            // Wait, `ExperimentAction` in types has `children?: ExperimentAction[]`.
            // If the DB schema `actions` table handles nesting via `parameters` or specific column, I need to know.
            // Defaulting to "children" property in JSON parameter if DB doesn't have parentId.
            // Checking `schema.ts`: "children" is likely NOT a column if I haven't seen it in seed-dev.
            // However, `ActionChip` uses `action.children`. Steps map to `actions`.
            // If `actions` table has `parentId` or `children` JSONB.
            // I will assume `children` is part of the `parameters` or a simplified representation for now, 
            // BUT `FlowWorkspace` treats `action.children` as real actions.
            // Let's check `schema.ts` quickly.
        });

        // I need to check schema.actions definition effectively.
        // For this pass, I will insert them as flat actions since I can't confirm nesting storage without checking schema.
        // But the user WANTS to see the nesting (Sequence, Parallel).
        // The `SortableActionChip` renders `action.children`.
        // The `TrialExecutionEngine` executes `action.children`.
        // So the data MUST include children.
        // Most likely `actions` table has a `children` JSONB column.

        // I will insert a Parallel action with embedded children in the `children` column (if it exists) or `parameters`.
        // Re-reading `scripts/seed-dev.ts`: It doesn't show any nested actions.
        // I will read `src/server/db/schema.ts` to be sure.

    } catch (err) {
        console.error(err);
        process.exit(1);
    }
}

// I'll write the file AFTER checking schema to ensure I structure the nested actions correctly.
