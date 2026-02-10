
import { db } from "~/server/db";
import { experiments, steps, actions } from "~/server/db/schema";
import { eq, asc, desc } from "drizzle-orm";
import { convertDatabaseToSteps } from "~/lib/experiment-designer/block-converter";

async function verifyTrpcLogic() {
    console.log("Verifying TRPC Logic for Interactive Storyteller...");

    // 1. Simulate the DB Query from experiments.ts
    const experiment = await db.query.experiments.findFirst({
        where: eq(experiments.name, "The Interactive Storyteller"),
        with: {
            study: {
                columns: {
                    id: true,
                    name: true,
                },
            },
            createdBy: {
                columns: {
                    id: true,
                    name: true,
                    email: true,
                },
            },
            robot: true,
            steps: {
                with: {
                    actions: {
                        orderBy: [asc(actions.orderIndex)],
                    },
                },
                orderBy: [asc(steps.orderIndex)],
            },
        },
    });

    if (!experiment) {
        console.error("Experiment not found!");
        return;
    }

    // 2. Simulate the Transformation
    console.log("Transforming DB steps to Designer steps...");
    const transformedSteps = convertDatabaseToSteps(experiment.steps);

    // 3. Inspect Step 4 (Branch A)
    // Step index 3 (0-based) is Branch A
    const branchAStep = transformedSteps[3];
    console.log("Step 4 (Branch A):", branchAStep.name);
    console.log("  Type:", branchAStep.type);
    console.log("  Trigger:", JSON.stringify(branchAStep.trigger, null, 2));

    // Check conditions specifically
    const conditions = branchAStep.trigger?.conditions as any;
    if (conditions?.nextStepId) {
        console.log("SUCCESS: nextStepId found in conditions:", conditions.nextStepId);
    } else {
        console.error("FAILURE: nextStepId MISSING in conditions!");
    }

    // Inspect Step 5 (Branch B) for completeness
    const branchBStep = transformedSteps[4];
    console.log("Step 5 (Branch B):", branchBStep.name);
    console.log("  Trigger:", JSON.stringify(branchBStep.trigger, null, 2));
}

verifyTrpcLogic()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
