
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../../src/server/db/schema";
import { eq } from "drizzle-orm";

const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
    console.log("🔍 Checking seeded actions...");

    const actions = await db.query.actions.findMany({
        where: (actions, { or, eq, like }) => or(
            eq(actions.type, "sequence"),
            eq(actions.type, "parallel"),
            eq(actions.type, "loop"),
            eq(actions.type, "branch"),
            like(actions.type, "hristudio-core%")
        ),
        limit: 10
    });

    console.log(`Found ${actions.length} control actions.`);

    for (const action of actions) {
        console.log(`\nAction: ${action.name} (${action.type})`);
        console.log(`ID: ${action.id}`);
        // Explicitly log parameters to check structure
        console.log("Parameters:", JSON.stringify(action.parameters, null, 2));

        const params = action.parameters as any;
        if (params.children) {
            console.log(`✅ Has ${params.children.length} children in parameters.`);
        } else if (params.trueBranch || params.falseBranch) {
            console.log(`✅ Has branches in parameters.`);
        } else {
            console.log(`❌ No children/branches found in parameters.`);
        }
    }

    await connection.end();
}

main();
