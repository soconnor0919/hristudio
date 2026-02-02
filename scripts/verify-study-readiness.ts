import { drizzle } from "drizzle-orm/postgres-js";
import { eq, sql } from "drizzle-orm";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";

const connectionString = process.env.DATABASE_URL!;
const client = postgres(connectionString);
const db = drizzle(client, { schema });

async function verify() {
    console.log("🔍 Verifying Study Readiness...");

    // 1. Check Study
    const study = await db.query.studies.findFirst({
        where: eq(schema.studies.name, "Comparative WoZ Study")
    });

    if (!study) {
        console.error("❌ Study 'Comparative WoZ Study' not found.");
        process.exit(1);
    }
    console.log("✅ Study found:", study.name);

    // 2. Check Experiment
    const experiment = await db.query.experiments.findFirst({
        where: eq(schema.experiments.name, "The Interactive Storyteller")
    });

    if (!experiment) {
        console.error("❌ Experiment 'The Interactive Storyteller' not found.");
        process.exit(1);
    }
    console.log("✅ Experiment found:", experiment.name);

    // 3. Check Steps
    const steps = await db.query.steps.findMany({
        where: eq(schema.steps.experimentId, experiment.id),
        orderBy: schema.steps.orderIndex
    });

    console.log(`ℹ️ Found ${steps.length} steps.`);
    if (steps.length < 5) {
        console.error("❌ Expected at least 5 steps, found " + steps.length);
        process.exit(1);
    }

    // Verify Step Names
    const expectedSteps = ["The Hook", "The Narrative - Part 1", "Comprehension Check", "Positive Feedback", "Conclusion"];
    for (let i = 0; i < expectedSteps.length; i++) {
        if (steps[i].name !== expectedSteps[i]) {
            console.error(`❌ Step mismatch at index ${i}. Expected '${expectedSteps[i]}', got '${steps[i].name}'`);
        } else {
            console.log(`✅ Step ${i + 1}: ${steps[i].name}`);
        }
    }

    // 4. Check Plugin Actions
    // Find the NAO6 plugin
    const plugin = await db.query.plugins.findFirst({
        where: (plugins, { eq, and }) => and(eq(plugins.name, "NAO6 Robot (Enhanced ROS2 Integration)"), eq(plugins.status, "active"))
    });

    if (!plugin) {
        console.error("❌ NAO6 Plugin not found.");
        process.exit(1);
    }

    const actions = plugin.actionDefinitions as any[];
    const requiredActions = ["nao_nod", "nao_shake_head", "nao_bow", "nao_open_hand"];

    for (const actionId of requiredActions) {
        const found = actions.find(a => a.id === actionId);
        if (!found) {
            console.error(`❌ Plugin missing action: ${actionId}`);
            process.exit(1);
        }
        console.log(`✅ Plugin has action: ${actionId}`);
    }

    console.log("🎉 Verification Complete: Platform is ready for the study!");
    process.exit(0);
}

verify().catch((e) => {
    console.error(e);
    process.exit(1);
});
