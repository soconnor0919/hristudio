import { db } from "~/server/db";
import { sql } from "drizzle-orm";

async function migrate() {
  console.log("Adding identifier column to hs_plugin...");

  try {
    await db.execute(sql`ALTER TABLE hs_plugin ADD COLUMN identifier varchar(100)`);
    console.log("✓ Added identifier column");
  } catch (e: any) {
    console.log("Column may already exist:", e.message);
  }

  try {
    await db.execute(sql`UPDATE hs_plugin SET identifier = name WHERE identifier IS NULL`);
    console.log("✓ Copied name to identifier");
  } catch (e: any) {
    console.log("Error copying:", e.message);
  }

  try {
    await db.execute(sql`ALTER TABLE hs_plugin ADD CONSTRAINT hs_plugin_identifier_unique UNIQUE (identifier)`);
    console.log("✓ Added unique constraint");
  } catch (e: any) {
    console.log("Constraint may already exist:", e.message);
  }

  console.log("Migration complete!");
}

migrate().catch(console.error);
