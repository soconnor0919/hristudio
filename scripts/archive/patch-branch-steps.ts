
import { db } from "~/server/db";
import { steps } from "~/server/db/schema";
import { eq, sql } from "drizzle-orm";

async function patchBranchSteps() {
    console.log("Patching branch steps for Interactive Storyteller...");

    // Target Step IDs (From debug output)
    const step3CondId = "b9d43f8c-c40c-4f1c-9fdc-9076338d3c85"; // Step 3: Comprehension Check
    const stepBranchAId = "3a2dc0b7-a43e-4236-9b9e-f957abafc1e5"; // Step 4: Branch A (Correct)
    const stepBranchBId = "3ae2fe8a-fc5d-4a04-baa5-699a21f19e30"; // Step 5: Branch B (Incorrect)
    const stepConclusionId = "cc3fbc7f-29e5-45e0-8d46-e80813c54292"; // Step 6: Conclusion

    // Update Step 3 (The Conditional Step)
    console.log("Updating Step 3 (Conditional Step)...");
    const step3Conditional = await db.query.steps.findFirst({
        where: eq(steps.id, step3CondId)
    });

    if (step3Conditional) {
        const currentConditions = (step3Conditional.conditions as any) || {};
        const options = currentConditions.options || [];

        // Patch options to point to real step IDs
        const newOptions = options.map((opt: any) => {
            if (opt.value === "Correct") return { ...opt, nextStepId: stepBranchAId };
            if (opt.value === "Incorrect") return { ...opt, nextStepId: stepBranchBId };
            return opt;
        });

        const newConditions = { ...currentConditions, options: newOptions };

        await db.execute(sql`
            UPDATE hs_step 
            SET conditions = ${JSON.stringify(newConditions)}::jsonb
            WHERE id = ${step3CondId}
        `);
        console.log("Step 3 (Conditional) updated links.");
    } else {
        console.log("Step 3 (Conditional) not found.");
    }

    // Update Step 4 (Branch A)
    console.log("Updating Step 4 (Branch A)...");
    /* 
       Note: We already patched Step 4 in previous run but under wrong assumption?
       Let's re-patch to be safe.
       Debug output showed ID: 3a2dc0b7-a43e-4236-9b9e-f957abafc1e5
       It should jump to Conclusion (cc3fbc7f...)
    */
    const stepBranchA = await db.query.steps.findFirst({
        where: eq(steps.id, stepBranchAId)
    });

    if (stepBranchA) {
        const currentConditions = (stepBranchA.conditions as Record<string, unknown>) || {};
        const newConditions = { ...currentConditions, nextStepId: stepConclusionId };

        await db.execute(sql`
            UPDATE hs_step 
            SET conditions = ${JSON.stringify(newConditions)}::jsonb
            WHERE id = ${stepBranchAId}
        `);
        console.log("Step 4 (Branch A) updated jump target.");
    }

    // Update Step 5 (Branch B)
    console.log("Updating Step 5 (Branch B)...");
    const stepBranchB = await db.query.steps.findFirst({
        where: eq(steps.id, stepBranchBId)
    });

    if (stepBranchB) {
        const currentConditions = (stepBranchB.conditions as Record<string, unknown>) || {};
        const newConditions = { ...currentConditions, nextStepId: stepConclusionId };

        await db.execute(sql`
            UPDATE hs_step 
            SET conditions = ${JSON.stringify(newConditions)}::jsonb
            WHERE id = ${stepBranchBId}
        `);
        console.log("Step 5 (Branch B) updated jump target.");
    }
}

patchBranchSteps()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
