
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";
import { eq } from "drizzle-orm";

const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
    const exp = await db.query.experiments.findFirst({
        where: eq(schema.experiments.name, "Control Flow Demo"),
        columns: { id: true }
    });

    if (exp) {
        console.log(`Experiment ID: ${exp.id}`);
    } else {
        console.error("Experiment not found");
    }

    await connection.end();
}

main();
