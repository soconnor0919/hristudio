
import { db } from "~/server/db";
import { steps, experiments, actions } from "~/server/db/schema";
import { eq, asc } from "drizzle-orm";

async function debugExperimentStructure() {
    console.log("Debugging Experiment Structure for Interactive Storyteller...");

    // Find the experiment
    const experiment = await db.query.experiments.findFirst({
        where: eq(experiments.name, "The Interactive Storyteller"),
        with: {
            steps: {
                orderBy: [asc(steps.orderIndex)],
                with: {
                    actions: {
                        orderBy: [asc(actions.orderIndex)],
                    }
                }
            }
        }
    });

    if (!experiment) {
        console.error("Experiment not found!");
        return;
    }

    console.log(`Experiment: ${experiment.name} (${experiment.id})`);
    console.log(`Plugin Dependencies:`, experiment.pluginDependencies);
    console.log("---------------------------------------------------");

    experiment.steps.forEach((step, index) => {
        console.log(`Step ${index + 1}: ${step.name}`);
        console.log(`  ID: ${step.id}`);
        console.log(`  Type: ${step.type}`);
        console.log(`  Order: ${step.orderIndex}`);
        console.log(`  Conditions:`, JSON.stringify(step.conditions, null, 2));

        if (step.actions && step.actions.length > 0) {
            console.log(`  Actions (${step.actions.length}):`);
            step.actions.forEach((action, actionIndex) => {
                console.log(`    ${actionIndex + 1}. [${action.type}] ${action.name}`);
                if (action.type === 'wizard_wait_for_response') {
                    console.log(`       Options:`, JSON.stringify((action.parameters as any)?.options, null, 2));
                }
            });
        }
        console.log("---------------------------------------------------");
    });
}

debugExperimentStructure()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
