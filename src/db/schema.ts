import { sql, relations } from 'drizzle-orm';
import { integer, pgTable, serial, text, timestamp, varchar } from "drizzle-orm/pg-core";

export const usersTable = pgTable("users", {
  id: integer().primaryKey().generatedAlwaysAsIdentity(),
  name: varchar({ length: 255 }).notNull(),
  age: integer().notNull(),
  email: varchar({ length: 255 }).notNull().unique(),
});

export const studyTable = pgTable("study", {
  id: serial("id").primaryKey(),
  title: varchar("title", { length: 256 }).notNull(),
  description: varchar("description", { length: 1000 }),
  userId: varchar("user_id", { length: 256 }).notNull(),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .$onUpdate(() => new Date()),
});

export const participants = pgTable(
  "participant",
  {
    id: serial("id").primaryKey(),
    name: varchar("name", { length: 256 }).notNull(),
    studyId: integer("study_id").references(() => studyTable.id).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  }
);

export const roles = pgTable("roles", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 256 }).notNull().unique(),
  description: text("description"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

export const permissions = pgTable("permissions", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 256 }).notNull().unique(),
  description: text("description"),
  code: varchar("code", { length: 100 }).notNull().unique(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

export const rolePermissions = pgTable("role_permissions", {
  roleId: integer("role_id").references(() => roles.id).notNull(),
  permissionId: integer("permission_id").references(() => permissions.id).notNull(),
});

export const userRoles = pgTable("user_roles", {
  userId: varchar("user_id", { length: 256 }).notNull(),
  roleId: integer("role_id").references(() => roles.id).notNull(),
});

// Add relations
export const rolesRelations = relations(roles, ({ many }) => ({
  permissions: many(rolePermissions),
  users: many(userRoles),
}));

export const permissionsRelations = relations(permissions, ({ many }) => ({
  roles: many(rolePermissions),
}));