import { pgTable, integer, varchar, text, timestamp } from "drizzle-orm/pg-core";
import { createEnum } from "drizzle-orm/pg-core";
import { ROLES } from "~/lib/permissions/constants";

// Create enum from role values, excluding PRINCIPAL_INVESTIGATOR and ASSISTANT
// which are handled through permissions
export const studyRoleEnum = createEnum("study_role", [
  ROLES.ADMIN,
  ROLES.RESEARCHER,
  ROLES.WIZARD,
  ROLES.OBSERVER,
]);

export const studies = pgTable("studies", {
  id: integer("id").primaryKey(),
  name: varchar("name", { length: 255 }).notNull(),
  description: text("description"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
}); 