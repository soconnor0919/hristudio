// Example model schema from the Drizzle docs
// https://orm.drizzle.team/docs/sql-schema-declaration

import { pgTable } from "drizzle-orm/pg-core";
import { 
  serial, 
  varchar, 
  timestamp, 
  integer 
} from "drizzle-orm/pg-core";
import { sql } from "drizzle-orm";

/**
 * This is an example of how to use the multi-project schema feature of Drizzle ORM. Use the same
 * database instance for multiple projects.
 *
 * @see https://orm.drizzle.team/docs/goodies#multi-project-schema
 */
export const studies = pgTable(
  "study",
  {
    id: serial("id").primaryKey(),
    title: varchar("title", { length: 256 }).notNull(),
    description: varchar("description", { length: 1000 }),
    userId: varchar("user_id", { length: 256 }).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true }).$onUpdate(
      () => new Date()
    ),
  }
);

export const participants = pgTable(
  "participant",
  {
    id: serial("id").primaryKey(),
    name: varchar("name", { length: 256 }).notNull(),
    studyId: integer("study_id").references(() => studies.id).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  }
);

export const contentTypes = pgTable(
  "content_type",
  {
    id: serial("id").primaryKey(),
    name: varchar("name", { length: 50 }).notNull().unique(),
  }
);

export const contents = pgTable(
  "content",
  {
    id: serial("id").primaryKey(),
    contentTypeId: integer("content_type_id").references(() => contentTypes.id).notNull(),
    uploader: varchar("uploader", { length: 256 }).notNull(),
    location: varchar("location", { length: 1000 }).notNull(),
    previewLocation: varchar("preview_location", { length: 1000 }),
    title: varchar("title", { length: 256 }).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  }
);

export const informedConsentForms = pgTable(
  "informed_consent_form",
  {
    id: serial("id").primaryKey(),
    studyId: integer("study_id").references(() => studies.id).notNull(),
    participantId: integer("participant_id").references(() => participants.id).notNull(),
    contentId: integer("content_id").references(() => contents.id).notNull(),
    uploadedAt: timestamp("uploaded_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  }
);

export const users = pgTable(
  "user",
  {
    id: serial("id").primaryKey(),
    email: varchar("email", { length: 256 }).notNull().unique(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  }
);