

import { db } from "~/server/db";
import { steps, experiments } from "~/server/db/schema";
import { eq, asc } from "drizzle-orm";

async function inspectExperimentSteps() {
    // Find experiment by ID
    const experiment = await db.query.experiments.findFirst({
        where: eq(experiments.id, "961d0cb1-256d-4951-8387-6d855a0ae603")
    });

    if (!experiment) {
        console.log("Experiment not found!");
        return;
    }

    console.log(`Inspecting Experiment: ${experiment.name} (${experiment.id})`);

    const experimentSteps = await db.query.steps.findMany({
        where: eq(steps.experimentId, experiment.id),
        orderBy: [asc(steps.orderIndex)],
        with: {
            actions: {
                orderBy: (actions, { asc }) => [asc(actions.orderIndex)]
            }
        }
    });

    console.log(`Found ${experimentSteps.length} steps.`);

    for (const step of experimentSteps) {
        console.log("--------------------------------------------------");
        console.log(`Step [${step.orderIndex}] ID: ${step.id}`);
        console.log(`Name: ${step.name}`);
        console.log(`Type: ${step.type}`);
        console.log(`NextStepId: ${step.nextStepId}`);

        if (step.type === 'conditional') {
            console.log("Conditions:", JSON.stringify(step.conditions, null, 2));
        }

        if (step.actions.length > 0) {
            console.log("Actions:");
            for (const action of step.actions) {
                console.log(`  - [${action.orderIndex}] ${action.name} (${action.type})`);
                if (action.type === 'wizard_wait_for_response') {
                    console.log("    Parameters:", JSON.stringify(action.parameters, null, 2));
                }
            }
        }
    }
}

inspectExperimentSteps()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });

