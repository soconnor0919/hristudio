
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../../src/server/db/schema";
import { eq } from "drizzle-orm";

const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
    console.log("🔍 Checking Database State...");

    // 1. Check Plugin
    const plugins = await db.query.plugins.findMany();
    console.log(`\nFound ${plugins.length} plugins.`);

    const expectedKeys = new Set<string>();

    for (const p of plugins) {
        const meta = p.metadata as any;
        const defs = p.actionDefinitions as any[];

        console.log(`Plugin [${p.name}] (ID: ${p.id}):`);
        console.log(`  - Robot ID (Column): ${p.robotId}`);
        console.log(`  - Metadata.robotId: ${meta?.robotId}`);
        console.log(`  - Action Definitions: ${defs?.length ?? 0} found.`);

        if (defs && meta?.robotId) {
            defs.forEach(d => {
                const key = `${meta.robotId}.${d.id}`;
                expectedKeys.add(key);
                // console.log(`    -> Registers: ${key}`);
            });
        }
    }

    // 2. Check Actions
    const actions = await db.query.actions.findMany();
    console.log(`\nFound ${actions.length} actions.`);
    let errorCount = 0;
    for (const a of actions) {
        // Only check plugin actions
        if (a.sourceKind === 'plugin' || a.type.includes(".")) {
            const isRegistered = expectedKeys.has(a.type);
            const pluginIdMatch = a.pluginId === 'nao6-ros2';

            console.log(`Action [${a.name}] (Type: ${a.type}):`);
            console.log(`  - PluginId: ${a.pluginId} ${pluginIdMatch ? '✅' : '❌'}`);
            console.log(`  - In Registry: ${isRegistered ? '✅' : '❌'}`);

            if (!isRegistered || !pluginIdMatch) errorCount++;
        }
    }

    if (errorCount > 0) {
        console.log(`\n❌ Found ${errorCount} actions with issues.`);
    } else {
        console.log("\n✅ All plugin actions validated successfully against registry definitions.");
    }

    await connection.end();
}

main().catch(console.error);
