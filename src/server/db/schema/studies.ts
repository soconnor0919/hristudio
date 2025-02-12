import { relations } from "drizzle-orm";
import { integer, pgEnum, text, timestamp, varchar, serial } from "drizzle-orm/pg-core";
import { ROLES } from "~/lib/permissions/constants";
import { createTable } from "../utils";
import { users } from "./auth";

// Create enum from role values
export const studyRoleEnum = pgEnum("study_role", [
  ROLES.OWNER,
  ROLES.ADMIN,
  ROLES.PRINCIPAL_INVESTIGATOR,
  ROLES.WIZARD,
  ROLES.RESEARCHER,
  ROLES.OBSERVER,
]);

// Create enum for participant status
export const participantStatusEnum = pgEnum("participant_status", [
  "active",
  "inactive",
  "completed",
  "withdrawn",
]);

// Create enum for activity types
export const activityTypeEnum = pgEnum("activity_type", [
  "study_created",
  "study_updated",
  "study_deleted",
  "ownership_transferred",
  "member_added",
  "member_removed",
  "member_role_changed",
  "participant_added",
  "participant_updated",
  "participant_removed",
  "experiment_created",
  "experiment_updated",
  "experiment_deleted",
  "trial_started",
  "trial_completed",
  "trial_cancelled",
  "invitation_sent",
  "invitation_accepted",
  "invitation_declined",
  "invitation_expired",
  "invitation_revoked",
  "consent_form_added",
  "consent_form_signed",
  "metadata_updated",
  "data_exported",
]);

// Create enum for invitation status
export const invitationStatusEnum = pgEnum("invitation_status", [
  "pending",
  "accepted",
  "declined",
  "expired",
  "revoked",
]);

export const studyActivityTypeEnum = pgEnum("study_activity_type", [
  "member_added",
  "member_role_changed",
  "study_updated",
  "participant_added",
  "participant_updated",
  "invitation_sent",
  "invitation_accepted",
  "invitation_declined",
  "invitation_expired",
  "invitation_revoked",
]);

export const studies = createTable("study", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  createdById: varchar("created_by", { length: 255 }).notNull().references(() => users.id),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true }),
});

export const studyMembers = createTable("study_member", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id").notNull().references(() => studies.id, { onDelete: "cascade" }),
  userId: varchar("user_id", { length: 255 }).notNull().references(() => users.id, { onDelete: "cascade" }),
  role: studyRoleEnum("role").notNull(),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
});

export const studyMetadata = createTable("study_metadata", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id").notNull().references(() => studies.id, { onDelete: "cascade" }),
  key: varchar("key", { length: 256 }).notNull(),
  value: text("value"),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true }),
});

export const studyActivities = createTable("study_activity", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id").notNull().references(() => studies.id, { onDelete: "cascade" }),
  userId: varchar("user_id", { length: 255 }).notNull().references(() => users.id),
  type: activityTypeEnum("type").notNull(),
  description: text("description").notNull(),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
});

export const participants = createTable("participant", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id").notNull().references(() => studies.id, { onDelete: "cascade" }),
  // Identifiable information - only visible to roles with VIEW_PARTICIPANT_NAMES permission
  identifier: varchar("identifier", { length: 256 }),
  email: varchar("email", { length: 256 }),
  firstName: varchar("first_name", { length: 256 }),
  lastName: varchar("last_name", { length: 256 }),
  // Non-identifiable information - visible to all study members
  notes: text("notes"),
  status: participantStatusEnum("status").notNull().default("active"),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true }),
});

export const studyInvitations = createTable("study_invitation", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id").notNull().references(() => studies.id, { onDelete: "cascade" }),
  email: varchar("email", { length: 255 }).notNull(),
  role: studyRoleEnum("role").notNull(),
  token: varchar("token", { length: 255 }).notNull().unique(),
  status: invitationStatusEnum("status").notNull().default("pending"),
  expiresAt: timestamp("expires_at", { withTimezone: true }).notNull(),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow().notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true }),
  createdById: varchar("created_by", { length: 255 }).notNull().references(() => users.id),
});

// Relations
export const studiesRelations = relations(studies, ({ one, many }) => ({
  creator: one(users, { fields: [studies.createdById], references: [users.id] }),
  members: many(studyMembers),
  participants: many(participants),
  invitations: many(studyInvitations),
}));

export const studyMembersRelations = relations(studyMembers, ({ one }) => ({
  study: one(studies, { fields: [studyMembers.studyId], references: [studies.id] }),
  user: one(users, { fields: [studyMembers.userId], references: [users.id] }),
}));

export const participantsRelations = relations(participants, ({ one }) => ({
  study: one(studies, { fields: [participants.studyId], references: [studies.id] }),
}));

export const studyInvitationsRelations = relations(studyInvitations, ({ one }) => ({
  study: one(studies, { fields: [studyInvitations.studyId], references: [studies.id] }),
  creator: one(users, { fields: [studyInvitations.createdById], references: [users.id] }),
})); 