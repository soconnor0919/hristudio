
import { db } from "../src/server/db";
import { experiments, steps } from "../src/server/db/schema";
import { eq } from "drizzle-orm";

async function inspectAllSteps() {
    const result = await db.query.experiments.findMany({
        with: {
            steps: {
                orderBy: (steps, { asc }) => [asc(steps.orderIndex)],
                columns: {
                    id: true,
                    name: true,
                    type: true,
                    orderIndex: true,
                    conditions: true,
                }
            }
        }
    });

    console.log(`Found ${result.length} experiments.`);

    for (const exp of result) {
        console.log(`Experiment: ${exp.name} (${exp.id})`);
        for (const step of exp.steps) {
            // Only print conditional steps or the first step
            if (step.type === 'conditional' || step.orderIndex === 0) {
                console.log(`  [${step.orderIndex}] ${step.name} (${step.type})`);
                console.log(`     Conditions: ${JSON.stringify(step.conditions)}`);
            }
        }
        console.log('---');
    }
}

inspectAllSteps()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
