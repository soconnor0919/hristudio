import { relations } from "drizzle-orm";
import {
  integer,
  pgEnum,
  text,
  timestamp,
  varchar
} from "drizzle-orm/pg-core";
import { participants } from "../schema";
import { createTable } from "../utils";
import { users } from "./auth";
import { studies } from "./studies";

// Enums
export const experimentStatusEnum = pgEnum("experiment_status", [
  "draft",
  "active",
  "archived"
]);

export const stepTypeEnum = pgEnum("step_type", [
  "instruction",
  "robot-action",
  "wizard-action"
]);

export const actionTypeEnum = pgEnum("action_type", [
  "movement",
  "speech",
  "wait",
  "input"
]);

export const trialStatusEnum = pgEnum("trial_status", [
  "pending",
  "in-progress",
  "completed",
  "cancelled"
]);

// Tables
export const experiments = createTable("experiments", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  version: integer("version").notNull().default(1),
  status: varchar("status", { length: 50 })
    .notNull()
    .default("draft")
    .$type<typeof experimentStatusEnum.enumValues[number]>(),
  createdById: varchar("created_by", { length: 255 })
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const steps = createTable("steps", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  experimentId: integer("experiment_id")
    .notNull()
    .references(() => experiments.id, { onDelete: "cascade" }),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  order: integer("order").notNull(),
  type: varchar("type", { length: 50 })
    .notNull()
    .$type<typeof stepTypeEnum.enumValues[number]>(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const actions = createTable("actions", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  stepId: integer("step_id")
    .notNull()
    .references(() => steps.id, { onDelete: "cascade" }),
  type: varchar("type", { length: 50 })
    .notNull()
    .$type<typeof actionTypeEnum.enumValues[number]>(),
  parameters: text("parameters"), // JSON string of action parameters
  order: integer("order").notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const trials = createTable("trials", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  experimentId: integer("experiment_id")
    .notNull()
    .references(() => experiments.id, { onDelete: "cascade" }),
  participantId: integer("participant_id")
    .notNull()
    .references(() => participants.id, { onDelete: "cascade" }),
  wizardId: varchar("wizard_id", { length: 255 })
    .notNull()
    .references(() => users.id),
  status: varchar("status", { length: 50 })
    .notNull()
    .default("pending")
    .$type<typeof trialStatusEnum.enumValues[number]>(),
  startedAt: timestamp("started_at"),
  completedAt: timestamp("completed_at"),
  notes: text("notes"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const trialEvents = createTable("trial_events", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  trialId: integer("trial_id")
    .notNull()
    .references(() => trials.id, { onDelete: "cascade" }),
  type: varchar("type", { length: 50 }).notNull(),
  actionId: integer("action_id").references(() => actions.id),
  data: text("data"), // JSON string of event data
  timestamp: timestamp("timestamp").defaultNow().notNull(),
});

// Relations
export const experimentsRelations = relations(experiments, ({ one, many }) => ({
  study: one(studies, { fields: [experiments.studyId], references: [studies.id] }),
  creator: one(users, { fields: [experiments.createdById], references: [users.id] }),
  steps: many(steps),
  trials: many(trials),
}));

export const stepsRelations = relations(steps, ({ one, many }) => ({
  experiment: one(experiments, { fields: [steps.experimentId], references: [experiments.id] }),
  actions: many(actions),
}));

export const actionsRelations = relations(actions, ({ one }) => ({
  step: one(steps, { fields: [actions.stepId], references: [steps.id] }),
}));

export const trialsRelations = relations(trials, ({ one, many }) => ({
  experiment: one(experiments, { fields: [trials.experimentId], references: [experiments.id] }),
  participant: one(participants, { fields: [trials.participantId], references: [participants.id] }),
  wizard: one(users, { fields: [trials.wizardId], references: [users.id] }),
  events: many(trialEvents),
}));

export const trialEventsRelations = relations(trialEvents, ({ one }) => ({
  trial: one(trials, { fields: [trialEvents.trialId], references: [trials.id] }),
  action: one(actions, { fields: [trialEvents.actionId], references: [actions.id] }),
})); 