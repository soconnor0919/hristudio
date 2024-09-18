// Example model schema from the Drizzle docs
// https://orm.drizzle.team/docs/sql-schema-declaration

import { pgTableCreator } from "drizzle-orm/pg-core";
import { ColumnBaseConfig, ColumnDataType, SQL, sql } from "drizzle-orm";
import {
  index,
  pgTable,
  serial,
  integer,
  timestamp,
  varchar,
  ExtraConfigColumn,
} from "drizzle-orm/pg-core";

/**
 * This is an example of how to use the multi-project schema feature of Drizzle ORM. Use the same
 * database instance for multiple projects.
 *
 * @see https://orm.drizzle.team/docs/goodies#multi-project-schema
 */
export const createTable = pgTableCreator((name) => `hristudio_${name}`);

export const posts = createTable(
  "post",
  {
    id: serial("id").primaryKey(),
    name: varchar("name", { length: 256 }),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true }).$onUpdate(
      () => new Date()
    ),
  },
  (example: { name: SQL<unknown> | Partial<ExtraConfigColumn<ColumnBaseConfig<ColumnDataType, string>>>; }) => ({
    nameIndex: index("name_idx").on(example.name),
  })
);

export const studies = createTable(
  "study",
  {
    id: serial("id").primaryKey(),
    title: varchar("title", { length: 256 }).notNull(),
    description: varchar("description", { length: 1000 }),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true }).$onUpdate(
      () => new Date()
    ),
  },
  (study: { title: SQL<unknown> | Partial<ExtraConfigColumn<ColumnBaseConfig<ColumnDataType, string>>>; }) => ({
    titleIndex: index("title_idx").on(study.title),
  })
);