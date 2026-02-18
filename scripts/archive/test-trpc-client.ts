
import { appRouter } from "../../src/server/api/root";
import { createCallerFactory } from "../../src/server/api/trpc";
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../../src/server/db/schema";
import { eq } from "drizzle-orm";

// 1. Setup DB Context
const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

// 2. Mock Session
const mockSession = {
    user: {
        id: "0e830889-ab46-4b48-a8ba-1d4bd3e665ed", // Admin user ID from seed
        name: "Sean O'Connor",
        email: "sean@soconnor.dev"
    },
    expires: new Date().toISOString()
};

// 3. Create Caller
const createCaller = createCallerFactory(appRouter);
const caller = createCaller({
    db,
    session: mockSession as any,
    headers: new Headers()
});

async function main() {
    console.log("🔍 Fetching experiment via TRPC caller...");

    // Get ID first
    const exp = await db.query.experiments.findFirst({
        where: eq(schema.experiments.name, "Control Flow Demo"),
        columns: { id: true }
    });

    if (!exp) {
        console.error("❌ Experiment not found");
        return;
    }

    const result = await caller.experiments.get({ id: exp.id });

    console.log(`✅ Fetched experiment: ${result.name} (${result.id})`);

    if (result.steps && result.steps.length > 0) {
        console.log(`Checking ${result.steps.length} steps...`);
        const actions = result.steps[0]!.actions; // Step 1 actions
        console.log(`Step 1 has ${actions.length} actions.`);

        actions.forEach(a => {
            if (["sequence", "parallel", "loop", "branch"].includes(a.type)) {
                console.log(`\nAction: ${a.name} (${a.type})`);
                console.log(`Children Count: ${a.children ? a.children.length : 'UNDEFINED'}`);
                if (a.children && a.children.length > 0) {
                    console.log(`First Child: ${a.children[0]!.name} (${a.children[0]!.type})`);
                }
            }
        });
    } else {
        console.error("❌ No steps found in result.");
    }

    await connection.end();
}

main();
