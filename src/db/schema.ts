import { sql, relations } from 'drizzle-orm';
import { integer, pgTable, serial, text, timestamp, varchar, primaryKey } from "drizzle-orm/pg-core";

export const usersTable = pgTable("users", {
  id: varchar("id", { length: 256 }).primaryKey(),
  name: varchar("name", { length: 256 }),
  email: varchar("email", { length: 256 }).notNull(),
  imageUrl: varchar("image_url", { length: 512 }),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").$onUpdate(() => new Date()),
});

export const studyTable = pgTable("study", {
  id: serial("id").primaryKey(),
  title: varchar("title", { length: 256 }).notNull(),
  description: varchar("description", { length: 1000 }),
  userId: varchar("user_id", { length: 256 })
    .references(() => usersTable.id)
    .notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").$onUpdate(() => new Date()),
});

export const participantsTable = pgTable("participant", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 256 }).notNull(),
  studyId: integer("study_id")
    .references(() => studyTable.id)
    .notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

export const rolesTable = pgTable("roles", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 256 }).notNull().unique(),
  description: text("description"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

export const permissionsTable = pgTable("permissions", {
  id: serial("id").primaryKey(),
  name: varchar("name", { length: 256 }).notNull().unique(),
  description: text("description"),
  code: varchar("code", { length: 100 }).notNull().unique(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});

export const rolePermissionsTable = pgTable("role_permissions", {
  roleId: integer("role_id")
    .references(() => rolesTable.id)
    .notNull(),
  permissionId: integer("permission_id")
    .references(() => permissionsTable.id)
    .notNull(),
}, (table) => ({
  pk: primaryKey({ columns: [table.roleId, table.permissionId] }),
}));

export const userRolesTable = pgTable("user_roles", {
  userId: varchar("user_id", { length: 256 })
    .references(() => usersTable.id)
    .notNull(),
  roleId: integer("role_id")
    .references(() => rolesTable.id)
    .notNull(),
}, (table) => ({
  pk: primaryKey({ columns: [table.userId, table.roleId] }),
}));

export const usersRelations = relations(usersTable, ({ many }) => ({
  studies: many(studyTable),
  userRoles: many(userRolesTable),
}));

export const studyRelations = relations(studyTable, ({ one, many }) => ({
  user: one(usersTable, {
    fields: [studyTable.userId],
    references: [usersTable.id],
  }),
  participants: many(participantsTable),
}));

export const participantRelations = relations(participantsTable, ({ one }) => ({
  study: one(studyTable, {
    fields: [participantsTable.studyId],
    references: [studyTable.id],
  }),
}));

export const rolesRelations = relations(rolesTable, ({ many }) => ({
  rolePermissions: many(rolePermissionsTable),
  userRoles: many(userRolesTable),
}));

export const permissionsRelations = relations(permissionsTable, ({ many }) => ({
  rolePermissions: many(rolePermissionsTable),
}));

export const rolePermissionsRelations = relations(rolePermissionsTable, ({ one }) => ({
  role: one(rolesTable, {
    fields: [rolePermissionsTable.roleId],
    references: [rolesTable.id],
  }),
  permission: one(permissionsTable, {
    fields: [rolePermissionsTable.permissionId],
    references: [permissionsTable.id],
  }),
}));

export const userRolesRelations = relations(userRolesTable, ({ one }) => ({
  user: one(usersTable, {
    fields: [userRolesTable.userId],
    references: [usersTable.id],
  }),
  role: one(rolesTable, {
    fields: [userRolesTable.roleId],
    references: [rolesTable.id],
  }),
}));