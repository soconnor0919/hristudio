
import { db } from "~/server/db";
import { steps } from "~/server/db/schema";
import { eq, inArray } from "drizzle-orm";

async function inspectBranchSteps() {
    console.log("Inspecting Steps 4 (Branch A) and 5 (Branch B)...");

    const step4Id = "3a2dc0b7-a43e-4236-9b9e-f957abafc1e5";
    const step5Id = "3ae2fe8a-fc5d-4a04-baa5-699a21f19e30";

    const branchSteps = await db.query.steps.findMany({
        where: inArray(steps.id, [step4Id, step5Id])
    });

    branchSteps.forEach(step => {
        console.log(`Step: ${step.name} (${step.id})`);
        console.log(`  Type: ${step.type}`);
        console.log(`  Order: ${step.orderIndex}`);
        console.log(`  Conditions:`, JSON.stringify(step.conditions, null, 2));
        console.log("---------------------------------------------------");
    });
}

inspectBranchSteps()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
