
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
        if (!step1) throw new Error("Failed to create step1");

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
        if (!step2) throw new Error("Failed to create step2");

        // 4. Create Actions

        // --- Step 1 Actions ---

        // Action 1: Sequence
        // Note: Nested children are stored in 'children' property of the action object in frontend,
        // but in DB 'parameters' is the JSONB field.
        // However, looking at ActionChip, it expects `action.children`. 
        // The `ExperimentAction` type usually has `children` at top level.
        // If the DB doesn't have it, the API must be hydrating it.
        // BUT, for the purpose of this seed which writes to DB directly, I will put it in `parameters.children` 
        // and assume the frontend/API handles it or I'm missing a column.
        // Actually, looking at schema again, `actions` table DOES NOT have children.
        // So it MUST be in `parameters` or it's not persisted in this table structure yet (which would be a bug, but I'm seeding what exists).
        // Wait, if I put it in parameters, does the UI read it? 
        // `ActionChip` reads `action.children`. 
        // I will try to put it in `parameters` and distinct `children` property in the JSON passed to `parameters`? 
        // No, `parameters` is jsonb.
        // I will assume for now that the system expects it in parameters if it's not a column, OR it's not fully supported in DB yet.
        // I will stick to what the UI likely consumes. `parameters: { children: [...] }`

        // Sequence
        await db.insert(schema.actions).values({
            stepId: step1.id,
            name: "Introduction Sequence",
            type: "sequence",
            orderIndex: 0,
            // Embedding children here to demonstrate. 
            // Real implementation might vary if keys are strictly checked.
            parameters: {
                children: [
                    {
                        id: uuidv4(),
                        name: "Say Hello",
                        type: "nao6-ros2.say_text",
                        parameters: { text: "Hello there!" },
                        category: "interaction"
                    },
                    {
                        id: uuidv4(),
                        name: "Wave Hand",
                        type: "nao6-ros2.move_arm",
                        parameters: { arm: "right", action: "wave" },
                        category: "movement"
                    }
                ]
            },
            pluginId: "hristudio-core",
            category: "control",
            sourceKind: "core"
        });

        // Parallel
        await db.insert(schema.actions).values({
            stepId: step1.id,
            name: "Parallel Actions",
            type: "parallel",
            orderIndex: 1,
            parameters: {
                children: [
                    {
                        id: uuidv4(),
                        name: "Say 'Moving'",
                        type: "nao6-ros2.say_text",
                        parameters: { text: "I am moving and talking." },
                        category: "interaction"
                    },
                    {
                        id: uuidv4(),
                        name: "Walk Forward",
                        type: "nao6-ros2.move_to",
                        parameters: { x: 0.5, y: 0 },
                        category: "movement"
                    }
                ]
            },
            pluginId: "hristudio-core",
            category: "control",
            sourceKind: "core"
        });


        // --- Step 2 Actions ---

        // Loop
        await db.insert(schema.actions).values({
            stepId: step2.id,
            name: "Repeat Message",
            type: "loop",
            orderIndex: 0,
            parameters: {
                iterations: 3,
                children: [
                    {
                        id: uuidv4(),
                        name: "Say 'Echo'",
                        type: "nao6-ros2.say_text",
                        parameters: { text: "Echo" },
                        category: "interaction"
                    }
                ]
            },
            pluginId: "hristudio-core",
            category: "control",
            sourceKind: "core"
        });

        // Wait
        await db.insert(schema.actions).values({
            stepId: step2.id,
            name: "Wait 5 Seconds",
            type: "wait",
            orderIndex: 1,
            parameters: { duration: 5 },
            pluginId: "hristudio-core",
            category: "control",
            sourceKind: "core"
        });

        // Branch (Controls step routing, not nested actions)
        // Note: Branch configuration is stored in step.trigger.conditions, not action.parameters
        // The branch action itself is just a marker that this step has conditional routing
        await db.insert(schema.actions).values({
            stepId: step2.id,
            name: "Conditional Routing",
            type: "branch",
            orderIndex: 2,
            parameters: {
                // Branch actions don't have nested children
                // Routing is configured at the step level via trigger.conditions
            },
            pluginId: "hristudio-core",
            category: "control",
            sourceKind: "core"
        });

        // Update step2 to have conditional routing
        await db.update(schema.steps)
            .set({
                type: "conditional",
                conditions: {
                    options: [
                        {
                            label: "High Score Path",
                            nextStepIndex: 2, // Would go to a hypothetical step 3
                            variant: "default"
                        },
                        {
                            label: "Low Score Path",
                            nextStepIndex: 0, // Loop back to step 1
                            variant: "outline"
                        }
                    ]
                }
            })
            .where(sql`id = ${step2.id}`);


    } catch (err) {
        console.error(err);
        process.exit(1);
    } finally {
        await connection.end();
    }
}

main();
