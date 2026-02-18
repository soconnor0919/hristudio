
import { db } from "../../src/server/db";
import { experiments } from "../../src/server/db/schema";
import { eq } from "drizzle-orm";

async function inspectVisualDesign() {
    const exps = await db.select().from(experiments);

    for (const exp of exps) {
        console.log(`Experiment: ${exp.name}`);
        if (exp.visualDesign) {
            const vd = exp.visualDesign as any;
            console.log("Visual Design Steps:");
            if (vd.steps && Array.isArray(vd.steps)) {
                vd.steps.forEach((s: any, i: number) => {
                    console.log(`  [${i}] ${s.name} (${s.type})`);
                    console.log(`      Trigger: ${JSON.stringify(s.trigger)}`);
                });
            } else {
                console.log("  No steps in visualDesign or invalid format.");
            }
        } else {
            console.log("  No visualDesign blob.");
        }
    }
}

inspectVisualDesign()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
