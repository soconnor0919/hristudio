
import { drizzle } from "drizzle-orm/postgres-js";
import postgres from "postgres";
import * as schema from "../src/server/db/schema";
import { eq } from "drizzle-orm";

const connectionString = process.env.DATABASE_URL!;
const connection = postgres(connectionString);
const db = drizzle(connection, { schema });

async function main() {
    const user = await db.query.users.findFirst({
        where: eq(schema.users.email, "sean@soconnor.dev"),
        columns: { id: true }
    });

    if (user) {
        console.log(`User ID: ${user.id}`);
    } else {
        console.error("User not found");
    }

    await connection.end();
}

main();
