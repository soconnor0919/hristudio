
import { db } from "../../src/server/db";
import { experiments } from "../../src/server/db/schema";
import { eq, asc } from "drizzle-orm";
import { convertDatabaseToSteps } from "../../src/lib/experiment-designer/block-converter";

async function verifyConversion() {
    const experiment = await db.query.experiments.findFirst({
        with: {
            steps: {
                orderBy: (steps, { asc }) => [asc(steps.orderIndex)],
                with: {
                    actions: {
                        orderBy: (actions, { asc }) => [asc(actions.orderIndex)],
                    }
                }
            }
        }
    });

    if (!experiment) {
        console.log("No experiment found");
        return;
    }

    console.log("Raw DB Steps Count:", experiment.steps.length);
    const converted = convertDatabaseToSteps(experiment.steps);

    console.log("Converted Steps:");
    converted.forEach((s, idx) => {
        console.log(`[${idx}] ${s.name} (${s.type})`);
        console.log(`    Trigger:`, JSON.stringify(s.trigger));
        if (s.type === 'conditional') {
            console.log(`    Conditions populated?`, Object.keys(s.trigger.conditions).length > 0);
        }
    });
}

verifyConversion()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
