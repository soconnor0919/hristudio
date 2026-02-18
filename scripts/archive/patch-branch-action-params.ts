
import { db } from "~/server/db";
import { actions, steps } from "~/server/db/schema";
import { eq, sql } from "drizzle-orm";

async function patchActionParams() {
    console.log("Patching Action Parameters for Interactive Storyteller...");

    // Target Step IDs
    const step3CondId = "b9d43f8c-c40c-4f1c-9fdc-9076338d3c85"; // Step 3: Comprehension Check
    const actionId = "10851aef-e720-45fc-ba5e-05e1e3425dab"; // Action: Wait for Choice

    // 1. Get the authoritative conditions from the Step
    const step = await db.query.steps.findFirst({
        where: eq(steps.id, step3CondId)
    });

    if (!step) {
        console.error("Step 3 not found!");
        return;
    }

    const conditions = step.conditions as any;
    const richOptions = conditions?.options;

    if (!richOptions || !Array.isArray(richOptions)) {
        console.error("Step 3 conditions are missing valid options!");
        return;
    }

    console.log("Found rich options in Step:", JSON.stringify(richOptions, null, 2));

    // 2. Get the Action
    const action = await db.query.actions.findFirst({
        where: eq(actions.id, actionId)
    });

    if (!action) {
        console.error("Action not found!");
        return;
    }

    console.log("Current Action Parameters:", JSON.stringify(action.parameters, null, 2));

    // 3. Patch the Action Parameters
    // We replace the simple string options with the rich object options
    const currentParams = action.parameters as any;
    const newParams = {
        ...currentParams,
        options: richOptions // Overwrite with rich options from step
    };

    console.log("New Action Parameters:", JSON.stringify(newParams, null, 2));

    await db.execute(sql`
        UPDATE hs_action
        SET parameters = ${JSON.stringify(newParams)}::jsonb
        WHERE id = ${actionId}
    `);

    console.log("Action parameters successfully patched.");
}

patchActionParams()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
