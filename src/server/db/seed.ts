import { db } from "./index";
import { pluginRepositories } from "./schema/store";

async function seed() {
  console.log("🌱 Seeding database...");

  // Seed official repository with minimal info
  // The store will load the full metadata from GitHub Pages when initialized
  await db.insert(pluginRepositories).values({
    id: "hristudio-official",
    url: "https://soconnor0919.github.io/robot-plugins",
    trust: "official",
    enabled: true,
    lastSyncedAt: new Date(),
  }).onConflictDoUpdate({
    target: pluginRepositories.id,
    set: {
      url: "https://soconnor0919.github.io/robot-plugins",
      lastSyncedAt: new Date(),
    }
  });

  console.log("✅ Database seeded!");
}

seed().catch((error) => {
  console.error("Failed to seed database:", error);
  process.exit(1);
});