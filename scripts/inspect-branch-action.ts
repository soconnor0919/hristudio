
import { db } from "~/server/db";
import { actions, steps } from "~/server/db/schema";
import { eq } from "drizzle-orm";

async function inspectAction() {
    console.log("Inspecting Action 10851aef-e720-45fc-ba5e-05e1e3425dab...");

    const actionId = "10851aef-e720-45fc-ba5e-05e1e3425dab";

    const action = await db.query.actions.findFirst({
        where: eq(actions.id, actionId),
        with: {
            step: {
                columns: {
                    id: true,
                    name: true,
                    type: true,
                    conditions: true
                }
            }
        }
    });

    if (!action) {
        console.error("Action not found!");
        return;
    }

    console.log("Action Found:");
    console.log("  Name:", action.name);
    console.log("  Type:", action.type);
    console.log("  Parameters:", JSON.stringify(action.parameters, null, 2));

    console.log("Parent Step:");
    console.log("  ID:", action.step.id);
    console.log("  Name:", action.step.name);
    console.log("  Type:", action.step.type);
    console.log("  Conditions:", JSON.stringify(action.step.conditions, null, 2));
}

inspectAction()
    .then(() => process.exit(0))
    .catch((err) => {
        console.error(err);
        process.exit(1);
    });
