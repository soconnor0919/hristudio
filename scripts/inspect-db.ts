
import { db } from "../src/server/db";
import { steps } from "../src/server/db/schema";
import { eq, like } from "drizzle-orm";

async function checkSteps() {
    const allSteps = await db.select().from(steps).where(like(steps.name, "%Comprehension Check%"));

    console.log("Found steps:", allSteps.length);

    for (const step of allSteps) {
        console.log("Step Name:", step.name);
        console.log("Type:", step.type);
        console.log("Conditions (typeof):", typeof step.conditions);
        console.log("Conditions (value):", JSON.stringify(step.conditions, null, 2));
    }
}

checkSteps()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
