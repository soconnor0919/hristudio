import { jsonb, text, timestamp, boolean } from "drizzle-orm/pg-core";
import { createId } from "@paralleldrive/cuid2";
import { createTable } from "../utils";

export const pluginRepositories = createTable("plugin_repositories", {
  id: text("id")
    .primaryKey()
    .$defaultFn(() => createId()),
  urls: jsonb("urls").notNull().$type<{ git: string; repository: string }>(),
  trust: text("trust", { enum: ["official", "verified", "community"] })
    .notNull()
    .default("community"),
  enabled: boolean("enabled").notNull().default(true),
  lastSyncedAt: timestamp("last_synced_at"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  updatedAt: timestamp("updated_at").notNull().defaultNow(),
});

export const installedPlugins = createTable("installed_plugins", {
  id: text("id")
    .primaryKey()
    .$defaultFn(() => createId()),
  robotId: text("robot_id").notNull(),
  repositoryId: text("repository_id")
    .notNull()
    .references(() => pluginRepositories.id, { onDelete: "cascade" }),
  enabled: boolean("enabled").notNull().default(true),
  config: jsonb("config").notNull().default({}),
  lastSyncedAt: timestamp("last_synced_at"),
  createdAt: timestamp("created_at").notNull().defaultNow(),
  updatedAt: timestamp("updated_at").notNull().defaultNow(),
});
