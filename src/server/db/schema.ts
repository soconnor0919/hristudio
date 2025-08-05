import { relations, sql } from "drizzle-orm";
import {
  bigint,
  boolean,
  index,
  inet,
  integer,
  jsonb,
  pgEnum,
  pgTableCreator,
  primaryKey,
  text,
  timestamp,
  unique,
  uuid,
  varchar,
} from "drizzle-orm/pg-core";
import { type AdapterAccount } from "next-auth/adapters";

/**
 * This is an example of how to use the multi-project schema feature of Drizzle ORM. Use the same
 * database instance for multiple projects.
 *
 * @see https://orm.drizzle.team/docs/goodies#multi-project-schema
 */
export const createTable = pgTableCreator((name) => `hs_${name}`);

// Enums
export const systemRoleEnum = pgEnum("system_role", [
  "administrator",
  "researcher",
  "wizard",
  "observer",
]);

export const studyStatusEnum = pgEnum("study_status", [
  "draft",
  "active",
  "completed",
  "archived",
]);

export const studyMemberRoleEnum = pgEnum("study_member_role", [
  "owner",
  "researcher",
  "wizard",
  "observer",
]);

export const experimentStatusEnum = pgEnum("experiment_status", [
  "draft",
  "testing",
  "ready",
  "deprecated",
]);

export const trialStatusEnum = pgEnum("trial_status", [
  "scheduled",
  "in_progress",
  "completed",
  "aborted",
  "failed",
]);

export const stepTypeEnum = pgEnum("step_type", [
  "wizard",
  "robot",
  "parallel",
  "conditional",
]);

export const communicationProtocolEnum = pgEnum("communication_protocol", [
  "rest",
  "ros2",
  "custom",
]);

export const trustLevelEnum = pgEnum("trust_level", [
  "official",
  "verified",
  "community",
]);

export const pluginStatusEnum = pgEnum("plugin_status", [
  "active",
  "deprecated",
  "disabled",
]);

export const blockShapeEnum = pgEnum("block_shape", [
  "action",
  "control",
  "value",
  "boolean",
  "hat",
  "cap",
]);

export const blockCategoryEnum = pgEnum("block_category", [
  "wizard",
  "robot",
  "control",
  "sensor",
  "logic",
  "event",
]);

export const mediaTypeEnum = pgEnum("media_type", ["video", "audio", "image"]);

export const exportStatusEnum = pgEnum("export_status", [
  "pending",
  "processing",
  "completed",
  "failed",
]);

// Users and Authentication
export const users = createTable("user", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  email: varchar("email", { length: 255 }).notNull().unique(),
  emailVerified: timestamp("email_verified", {
    mode: "date",
    withTimezone: true,
  }),
  name: varchar("name", { length: 255 }),
  image: text("image"),
  password: varchar("password", { length: 255 }),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  deletedAt: timestamp("deleted_at", { withTimezone: true }),
});

export const accounts = createTable(
  "account",
  {
    userId: uuid("user_id")
      .notNull()
      .references(() => users.id, { onDelete: "cascade" }),
    type: varchar("type", { length: 255 })
      .$type<AdapterAccount["type"]>()
      .notNull(),
    provider: varchar("provider", { length: 255 }).notNull(),
    providerAccountId: varchar("provider_account_id", {
      length: 255,
    }).notNull(),
    refreshToken: text("refresh_token"),
    accessToken: text("access_token"),
    expiresAt: integer("expires_at"),
    tokenType: varchar("token_type", { length: 255 }),
    scope: varchar("scope", { length: 255 }),
    idToken: text("id_token"),
    sessionState: varchar("session_state", { length: 255 }),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    compoundKey: primaryKey({
      columns: [table.provider, table.providerAccountId],
    }),
    userIdIdx: index("account_user_id_idx").on(table.userId),
  }),
);

export const sessions = createTable(
  "session",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    sessionToken: varchar("session_token", { length: 255 }).notNull().unique(),
    userId: uuid("user_id")
      .notNull()
      .references(() => users.id, { onDelete: "cascade" }),
    expires: timestamp("expires", {
      mode: "date",
      withTimezone: true,
    }).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    userIdIdx: index("session_user_id_idx").on(table.userId),
  }),
);

export const verificationTokens = createTable(
  "verification_token",
  {
    identifier: varchar("identifier", { length: 255 }).notNull(),
    token: varchar("token", { length: 255 }).notNull().unique(),
    expires: timestamp("expires", {
      mode: "date",
      withTimezone: true,
    }).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    compoundKey: primaryKey({ columns: [table.identifier, table.token] }),
  }),
);

// Roles and Permissions
export const userSystemRoles = createTable(
  "user_system_role",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    userId: uuid("user_id")
      .notNull()
      .references(() => users.id, { onDelete: "cascade" }),
    role: systemRoleEnum("role").notNull(),
    grantedAt: timestamp("granted_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    grantedBy: uuid("granted_by").references(() => users.id),
  },
  (table) => ({
    userRoleUnique: unique().on(table.userId, table.role),
  }),
);

export const permissions = createTable("permission", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  name: varchar("name", { length: 100 }).notNull().unique(),
  description: text("description"),
  resource: varchar("resource", { length: 50 }).notNull(),
  action: varchar("action", { length: 50 }).notNull(),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const rolePermissions = createTable(
  "role_permission",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    role: systemRoleEnum("role").notNull(),
    permissionId: uuid("permission_id")
      .notNull()
      .references(() => permissions.id, { onDelete: "cascade" }),
  },
  (table) => ({
    rolePermissionUnique: unique().on(table.role, table.permissionId),
  }),
);

// Study Hierarchy
export const studies = createTable("study", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  name: varchar("name", { length: 255 }).notNull(),
  description: text("description"),
  institution: varchar("institution", { length: 255 }),
  irbProtocol: varchar("irb_protocol", { length: 100 }),
  status: studyStatusEnum("status").default("draft").notNull(),
  createdBy: uuid("created_by")
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  metadata: jsonb("metadata").default({}),
  settings: jsonb("settings").default({}),
  deletedAt: timestamp("deleted_at", { withTimezone: true }),
});

export const studyMembers = createTable(
  "study_member",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id")
      .notNull()
      .references(() => studies.id, { onDelete: "cascade" }),
    userId: uuid("user_id")
      .notNull()
      .references(() => users.id, { onDelete: "cascade" }),
    role: studyMemberRoleEnum("role").notNull(),
    permissions: jsonb("permissions").default([]),
    joinedAt: timestamp("joined_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    invitedBy: uuid("invited_by").references(() => users.id),
  },
  (table) => ({
    studyUserUnique: unique().on(table.studyId, table.userId),
  }),
);

export const robots = createTable("robot", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  name: varchar("name", { length: 255 }).notNull(),
  manufacturer: varchar("manufacturer", { length: 255 }),
  model: varchar("model", { length: 255 }),
  description: text("description"),
  capabilities: jsonb("capabilities").default([]),
  communicationProtocol: communicationProtocolEnum("communication_protocol"),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const robotPlugins = createTable("robot_plugin", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  name: varchar("name", { length: 255 }).notNull(),
  version: varchar("version", { length: 50 }).notNull(),
  manufacturer: varchar("manufacturer", { length: 255 }),
  description: text("description"),
  robotId: uuid("robot_id").references(() => robots.id),
  communicationProtocol: communicationProtocolEnum("communication_protocol"),
  status: pluginStatusEnum("status").default("active").notNull(),
  configSchema: jsonb("config_schema"),
  capabilities: jsonb("capabilities").default([]),
  trustLevel: trustLevelEnum("trust_level").default("community").notNull(),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const blockRegistry = createTable(
  "block_registry",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    blockType: varchar("block_type", { length: 100 }).notNull(),
    pluginId: uuid("plugin_id").references(() => robotPlugins.id),
    shape: blockShapeEnum("shape").notNull(),
    category: blockCategoryEnum("category").notNull(),
    displayName: varchar("display_name", { length: 255 }).notNull(),
    description: text("description"),
    icon: varchar("icon", { length: 100 }),
    color: varchar("color", { length: 50 }),
    config: jsonb("config").notNull(),
    parameterSchema: jsonb("parameter_schema").notNull(),
    executionHandler: varchar("execution_handler", { length: 100 }),
    timeout: integer("timeout"),
    retryPolicy: jsonb("retry_policy"),
    requiresConnection: boolean("requires_connection").default(false),
    previewMode: boolean("preview_mode").default(true),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    blockTypeUnique: unique().on(table.blockType, table.pluginId),
    categoryIdx: index("block_registry_category_idx").on(table.category),
  }),
);

export const experiments = createTable(
  "experiment",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id")
      .notNull()
      .references(() => studies.id, { onDelete: "cascade" }),
    name: varchar("name", { length: 255 }).notNull(),
    description: text("description"),
    version: integer("version").default(1).notNull(),
    robotId: uuid("robot_id").references(() => robots.id),
    status: experimentStatusEnum("status").default("draft").notNull(),
    estimatedDuration: integer("estimated_duration"), // in minutes
    createdBy: uuid("created_by")
      .notNull()
      .references(() => users.id),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    metadata: jsonb("metadata").default({}),
    visualDesign: jsonb("visual_design"),
    executionGraph: jsonb("execution_graph"),
    pluginDependencies: text("plugin_dependencies").array(),
    deletedAt: timestamp("deleted_at", { withTimezone: true }),
  },
  (table) => ({
    studyNameVersionUnique: unique().on(
      table.studyId,
      table.name,
      table.version,
    ),
    visualDesignIdx: index("experiment_visual_design_idx").using(
      "gin",
      table.visualDesign,
    ),
  }),
);

export const participants = createTable(
  "participant",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id")
      .notNull()
      .references(() => studies.id, { onDelete: "cascade" }),
    participantCode: varchar("participant_code", { length: 50 }).notNull(),
    email: varchar("email", { length: 255 }),
    name: varchar("name", { length: 255 }),
    demographics: jsonb("demographics").default({}), // encrypted
    consentGiven: boolean("consent_given").default(false).notNull(),
    consentDate: timestamp("consent_date", { withTimezone: true }),
    notes: text("notes"), // encrypted
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    studyParticipantCodeUnique: unique().on(
      table.studyId,
      table.participantCode,
    ),
  }),
);

export const trials = createTable("trial", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  experimentId: uuid("experiment_id")
    .notNull()
    .references(() => experiments.id),
  participantId: uuid("participant_id").references(() => participants.id),
  wizardId: uuid("wizard_id").references(() => users.id),
  sessionNumber: integer("session_number").default(1).notNull(),
  status: trialStatusEnum("status").default("scheduled").notNull(),
  scheduledAt: timestamp("scheduled_at", { withTimezone: true }),
  startedAt: timestamp("started_at", { withTimezone: true }),
  completedAt: timestamp("completed_at", { withTimezone: true }),
  duration: integer("duration"), // actual duration in seconds
  notes: text("notes"),
  parameters: jsonb("parameters").default({}),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  metadata: jsonb("metadata").default({}),
});

export const steps = createTable(
  "step",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    experimentId: uuid("experiment_id")
      .notNull()
      .references(() => experiments.id, { onDelete: "cascade" }),
    name: varchar("name", { length: 255 }).notNull(),
    description: text("description"),
    type: stepTypeEnum("type").notNull(),
    orderIndex: integer("order_index").notNull(),
    durationEstimate: integer("duration_estimate"), // in seconds
    required: boolean("required").default(true).notNull(),
    conditions: jsonb("conditions").default({}),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    experimentOrderUnique: unique().on(table.experimentId, table.orderIndex),
  }),
);

export const actions = createTable(
  "action",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    stepId: uuid("step_id")
      .notNull()
      .references(() => steps.id, { onDelete: "cascade" }),
    name: varchar("name", { length: 255 }).notNull(),
    description: text("description"),
    type: varchar("type", { length: 100 }).notNull(), // e.g., 'speak', 'move', 'wait', 'collect_data'
    orderIndex: integer("order_index").notNull(),
    parameters: jsonb("parameters").default({}),
    validationSchema: jsonb("validation_schema"),
    timeout: integer("timeout"), // in seconds
    retryCount: integer("retry_count").default(0).notNull(),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    stepOrderUnique: unique().on(table.stepId, table.orderIndex),
  }),
);

// Participants and Data Protection
export const consentForms = createTable(
  "consent_form",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id")
      .notNull()
      .references(() => studies.id, { onDelete: "cascade" }),
    version: integer("version").default(1).notNull(),
    title: varchar("title", { length: 255 }).notNull(),
    content: text("content").notNull(),
    active: boolean("active").default(true).notNull(),
    createdBy: uuid("created_by")
      .notNull()
      .references(() => users.id),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    storagePath: text("storage_path"), // path in MinIO
  },
  (table) => ({
    studyVersionUnique: unique().on(table.studyId, table.version),
  }),
);

export const participantConsents = createTable(
  "participant_consent",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    participantId: uuid("participant_id")
      .notNull()
      .references(() => participants.id, { onDelete: "cascade" }),
    consentFormId: uuid("consent_form_id")
      .notNull()
      .references(() => consentForms.id),
    signedAt: timestamp("signed_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    signatureData: text("signature_data"), // encrypted
    ipAddress: inet("ip_address"),
    storagePath: text("storage_path"), // path to signed PDF in MinIO
  },
  (table) => ({
    participantFormUnique: unique().on(
      table.participantId,
      table.consentFormId,
    ),
  }),
);

// Robot Platform Integration
export const plugins = createTable(
  "plugin",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    robotId: uuid("robot_id").references(() => robots.id, {
      onDelete: "cascade",
    }),
    name: varchar("name", { length: 255 }).notNull(),
    version: varchar("version", { length: 50 }).notNull(),
    description: text("description"),
    author: varchar("author", { length: 255 }),
    repositoryUrl: text("repository_url"),
    trustLevel: trustLevelEnum("trust_level"),
    status: pluginStatusEnum("status").default("active").notNull(),
    configurationSchema: jsonb("configuration_schema"),
    actionDefinitions: jsonb("action_definitions").default([]),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    updatedAt: timestamp("updated_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    metadata: jsonb("metadata").default({}),
  },
  (table) => ({
    nameVersionUnique: unique().on(table.name, table.version),
  }),
);

export const studyPlugins = createTable(
  "study_plugin",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id")
      .notNull()
      .references(() => studies.id, { onDelete: "cascade" }),
    pluginId: uuid("plugin_id")
      .notNull()
      .references(() => plugins.id),
    configuration: jsonb("configuration").default({}),
    installedAt: timestamp("installed_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    installedBy: uuid("installed_by")
      .notNull()
      .references(() => users.id),
  },
  (table) => ({
    studyPluginUnique: unique().on(table.studyId, table.pluginId),
  }),
);

// Experiment Execution and Data Capture
export const trialEvents = createTable(
  "trial_event",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    trialId: uuid("trial_id")
      .notNull()
      .references(() => trials.id, { onDelete: "cascade" }),
    eventType: varchar("event_type", { length: 50 }).notNull(), // 'action_started', 'action_completed', 'error', 'intervention'
    actionId: uuid("action_id").references(() => actions.id),
    timestamp: timestamp("timestamp", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
    data: jsonb("data").default({}),
    createdBy: uuid("created_by").references(() => users.id), // NULL for system events
  },
  (table) => ({
    trialTimestampIdx: index("trial_events_trial_timestamp_idx").on(
      table.trialId,
      table.timestamp,
    ),
  }),
);

export const wizardInterventions = createTable("wizard_intervention", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  trialId: uuid("trial_id")
    .notNull()
    .references(() => trials.id, { onDelete: "cascade" }),
  wizardId: uuid("wizard_id")
    .notNull()
    .references(() => users.id),
  interventionType: varchar("intervention_type", { length: 100 }).notNull(),
  description: text("description"),
  timestamp: timestamp("timestamp", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  parameters: jsonb("parameters").default({}),
  reason: text("reason"),
});

export const mediaCaptures = createTable("media_capture", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  trialId: uuid("trial_id")
    .notNull()
    .references(() => trials.id, { onDelete: "cascade" }),
  mediaType: mediaTypeEnum("media_type"),
  storagePath: text("storage_path").notNull(), // MinIO path
  fileSize: bigint("file_size", { mode: "number" }),
  duration: integer("duration"), // in seconds for video/audio
  format: varchar("format", { length: 20 }),
  resolution: varchar("resolution", { length: 20 }), // for video
  startTimestamp: timestamp("start_timestamp", { withTimezone: true }),
  endTimestamp: timestamp("end_timestamp", { withTimezone: true }),
  metadata: jsonb("metadata").default({}),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const sensorData = createTable(
  "sensor_data",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    trialId: uuid("trial_id")
      .notNull()
      .references(() => trials.id, { onDelete: "cascade" }),
    sensorType: varchar("sensor_type", { length: 50 }).notNull(),
    timestamp: timestamp("timestamp", { withTimezone: true }).notNull(),
    data: jsonb("data").notNull(),
    robotState: jsonb("robot_state").default({}),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    sensorTrialTimestampIdx: index("sensor_data_trial_timestamp_idx").on(
      table.trialId,
      table.timestamp,
    ),
  }),
);

export const annotations = createTable("annotation", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  trialId: uuid("trial_id")
    .notNull()
    .references(() => trials.id, { onDelete: "cascade" }),
  annotatorId: uuid("annotator_id")
    .notNull()
    .references(() => users.id),
  timestampStart: timestamp("timestamp_start", {
    withTimezone: true,
  }).notNull(),
  timestampEnd: timestamp("timestamp_end", { withTimezone: true }),
  category: varchar("category", { length: 100 }),
  label: varchar("label", { length: 100 }),
  description: text("description"),
  tags: jsonb("tags").default([]),
  metadata: jsonb("metadata").default({}),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Collaboration and Activity Tracking
export const activityLogs = createTable(
  "activity_log",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    studyId: uuid("study_id").references(() => studies.id, {
      onDelete: "cascade",
    }),
    userId: uuid("user_id").references(() => users.id),
    action: varchar("action", { length: 100 }).notNull(),
    resourceType: varchar("resource_type", { length: 50 }),
    resourceId: uuid("resource_id"),
    description: text("description"),
    ipAddress: inet("ip_address"),
    userAgent: text("user_agent"),
    metadata: jsonb("metadata").default({}),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    activityStudyCreatedIdx: index("activity_logs_study_created_idx").on(
      table.studyId,
      table.createdAt,
    ),
  }),
);

export const comments = createTable("comment", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  parentId: uuid("parent_id"),
  resourceType: varchar("resource_type", { length: 50 }).notNull(), // 'experiment', 'trial', 'annotation'
  resourceId: uuid("resource_id").notNull(),
  authorId: uuid("author_id")
    .notNull()
    .references(() => users.id),
  content: text("content").notNull(),
  metadata: jsonb("metadata"),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const attachments = createTable("attachment", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  resourceType: varchar("resource_type", { length: 50 }).notNull(),
  resourceId: uuid("resource_id").notNull(),
  fileName: varchar("file_name", { length: 255 }).notNull(),
  fileSize: bigint("file_size", { mode: "number" }).notNull(),
  filePath: text("file_path").notNull(),
  contentType: varchar("content_type", { length: 100 }),
  description: text("description"),
  uploadedBy: uuid("uploaded_by")
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// Data Export and Sharing
export const exportJobs = createTable("export_job", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  studyId: uuid("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  requestedBy: uuid("requested_by")
    .notNull()
    .references(() => users.id),
  exportType: varchar("export_type", { length: 50 }).notNull(), // 'full', 'trials', 'analysis', 'media'
  format: varchar("format", { length: 20 }).notNull(), // 'json', 'csv', 'zip'
  filters: jsonb("filters").default({}),
  status: exportStatusEnum("status").default("pending").notNull(),
  storagePath: text("storage_path"),
  expiresAt: timestamp("expires_at", { withTimezone: true }),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
  completedAt: timestamp("completed_at", { withTimezone: true }),
  errorMessage: text("error_message"),
});

export const sharedResources = createTable("shared_resource", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  studyId: uuid("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  resourceType: varchar("resource_type", { length: 50 }).notNull(),
  resourceId: uuid("resource_id").notNull(),
  sharedBy: uuid("shared_by")
    .notNull()
    .references(() => users.id),
  shareToken: varchar("share_token", { length: 255 }).unique(),
  permissions: jsonb("permissions").default(["read"]),
  expiresAt: timestamp("expires_at", { withTimezone: true }),
  accessCount: integer("access_count").default(0).notNull(),
  createdAt: timestamp("created_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

// System Configuration
export const systemSettings = createTable("system_setting", {
  id: uuid("id").notNull().primaryKey().defaultRandom(),
  key: varchar("key", { length: 100 }).notNull().unique(),
  value: jsonb("value").notNull(),
  description: text("description"),
  updatedBy: uuid("updated_by").references(() => users.id),
  updatedAt: timestamp("updated_at", { withTimezone: true })
    .default(sql`CURRENT_TIMESTAMP`)
    .notNull(),
});

export const auditLogs = createTable(
  "audit_log",
  {
    id: uuid("id").notNull().primaryKey().defaultRandom(),
    userId: uuid("user_id").references(() => users.id),
    action: varchar("action", { length: 100 }).notNull(),
    resourceType: varchar("resource_type", { length: 50 }),
    resourceId: uuid("resource_id"),
    changes: jsonb("changes").default({}),
    ipAddress: inet("ip_address"),
    userAgent: text("user_agent"),
    createdAt: timestamp("created_at", { withTimezone: true })
      .default(sql`CURRENT_TIMESTAMP`)
      .notNull(),
  },
  (table) => ({
    auditCreatedIdx: index("audit_logs_created_idx").on(table.createdAt),
  }),
);

// Relations
export const usersRelations = relations(users, ({ many }) => ({
  accounts: many(accounts),
  sessions: many(sessions),
  systemRoles: many(userSystemRoles, {
    relationName: "user",
  }),
  grantedRoles: many(userSystemRoles, {
    relationName: "grantedByUser",
  }),
  createdStudies: many(studies),
  studyMemberships: many(studyMembers),
  createdExperiments: many(experiments),
  wizardTrials: many(trials),
  activityLogs: many(activityLogs),
  comments: many(comments),
  attachments: many(attachments),
  annotations: many(annotations),
  interventions: many(wizardInterventions),
}));

export const accountsRelations = relations(accounts, ({ one }) => ({
  user: one(users, { fields: [accounts.userId], references: [users.id] }),
}));

export const sessionsRelations = relations(sessions, ({ one }) => ({
  user: one(users, { fields: [sessions.userId], references: [users.id] }),
}));

export const userSystemRolesRelations = relations(
  userSystemRoles,
  ({ one }) => ({
    user: one(users, {
      fields: [userSystemRoles.userId],
      references: [users.id],
      relationName: "user",
    }),
    grantedByUser: one(users, {
      fields: [userSystemRoles.grantedBy],
      references: [users.id],
      relationName: "grantedByUser",
    }),
  }),
);

export const rolePermissionsRelations = relations(
  rolePermissions,
  ({ one }) => ({
    permission: one(permissions, {
      fields: [rolePermissions.permissionId],
      references: [permissions.id],
    }),
  }),
);

export const studiesRelations = relations(studies, ({ one, many }) => ({
  createdBy: one(users, {
    fields: [studies.createdBy],
    references: [users.id],
  }),
  members: many(studyMembers),
  experiments: many(experiments),
  participants: many(participants),
  consentForms: many(consentForms),
  plugins: many(studyPlugins),
  activityLogs: many(activityLogs),
  comments: many(comments),
  attachments: many(attachments),
  exportJobs: many(exportJobs),
  sharedResources: many(sharedResources),
}));

export const studyMembersRelations = relations(studyMembers, ({ one }) => ({
  study: one(studies, {
    fields: [studyMembers.studyId],
    references: [studies.id],
  }),
  user: one(users, { fields: [studyMembers.userId], references: [users.id] }),
  invitedBy: one(users, {
    fields: [studyMembers.invitedBy],
    references: [users.id],
  }),
}));

export const experimentsRelations = relations(experiments, ({ one, many }) => ({
  study: one(studies, {
    fields: [experiments.studyId],
    references: [studies.id],
  }),
  robot: one(robots, {
    fields: [experiments.robotId],
    references: [robots.id],
  }),
  createdBy: one(users, {
    fields: [experiments.createdBy],
    references: [users.id],
  }),
  trials: many(trials),
  steps: many(steps),
}));

export const participantsRelations = relations(
  participants,
  ({ one, many }) => ({
    study: one(studies, {
      fields: [participants.studyId],
      references: [studies.id],
    }),
    trials: many(trials),
    consents: many(participantConsents),
  }),
);

export const trialsRelations = relations(trials, ({ one, many }) => ({
  experiment: one(experiments, {
    fields: [trials.experimentId],
    references: [experiments.id],
  }),
  participant: one(participants, {
    fields: [trials.participantId],
    references: [participants.id],
  }),
  wizard: one(users, { fields: [trials.wizardId], references: [users.id] }),
  events: many(trialEvents),
  interventions: many(wizardInterventions),
  mediaCaptures: many(mediaCaptures),
  sensorData: many(sensorData),
  annotations: many(annotations),
}));

export const stepsRelations = relations(steps, ({ one, many }) => ({
  experiment: one(experiments, {
    fields: [steps.experimentId],
    references: [experiments.id],
  }),
  actions: many(actions),
}));

export const actionsRelations = relations(actions, ({ one, many }) => ({
  step: one(steps, { fields: [actions.stepId], references: [steps.id] }),
  events: many(trialEvents),
}));

export const consentFormsRelations = relations(
  consentForms,
  ({ one, many }) => ({
    study: one(studies, {
      fields: [consentForms.studyId],
      references: [studies.id],
    }),
    createdBy: one(users, {
      fields: [consentForms.createdBy],
      references: [users.id],
    }),
    participantConsents: many(participantConsents),
  }),
);

export const participantConsentsRelations = relations(
  participantConsents,
  ({ one }) => ({
    participant: one(participants, {
      fields: [participantConsents.participantId],
      references: [participants.id],
    }),
    consentForm: one(consentForms, {
      fields: [participantConsents.consentFormId],
      references: [consentForms.id],
    }),
  }),
);

export const robotsRelations = relations(robots, ({ many }) => ({
  experiments: many(experiments),
  plugins: many(plugins),
}));

export const pluginsRelations = relations(plugins, ({ one, many }) => ({
  robot: one(robots, { fields: [plugins.robotId], references: [robots.id] }),
  studyPlugins: many(studyPlugins),
}));

export const studyPluginsRelations = relations(studyPlugins, ({ one }) => ({
  study: one(studies, {
    fields: [studyPlugins.studyId],
    references: [studies.id],
  }),
  plugin: one(plugins, {
    fields: [studyPlugins.pluginId],
    references: [plugins.id],
  }),
  installedBy: one(users, {
    fields: [studyPlugins.installedBy],
    references: [users.id],
  }),
}));

export const trialEventsRelations = relations(trialEvents, ({ one }) => ({
  trial: one(trials, {
    fields: [trialEvents.trialId],
    references: [trials.id],
  }),
  action: one(actions, {
    fields: [trialEvents.actionId],
    references: [actions.id],
  }),
  createdBy: one(users, {
    fields: [trialEvents.createdBy],
    references: [users.id],
  }),
}));

export const wizardInterventionsRelations = relations(
  wizardInterventions,
  ({ one }) => ({
    trial: one(trials, {
      fields: [wizardInterventions.trialId],
      references: [trials.id],
    }),
    wizard: one(users, {
      fields: [wizardInterventions.wizardId],
      references: [users.id],
    }),
  }),
);

export const mediaCapturesRelations = relations(mediaCaptures, ({ one }) => ({
  trial: one(trials, {
    fields: [mediaCaptures.trialId],
    references: [trials.id],
  }),
}));

export const sensorDataRelations = relations(sensorData, ({ one }) => ({
  trial: one(trials, { fields: [sensorData.trialId], references: [trials.id] }),
}));

export const annotationsRelations = relations(annotations, ({ one }) => ({
  trial: one(trials, {
    fields: [annotations.trialId],
    references: [trials.id],
  }),
  annotator: one(users, {
    fields: [annotations.annotatorId],
    references: [users.id],
  }),
}));

export const activityLogsRelations = relations(activityLogs, ({ one }) => ({
  study: one(studies, {
    fields: [activityLogs.studyId],
    references: [studies.id],
  }),
  user: one(users, { fields: [activityLogs.userId], references: [users.id] }),
}));

export const commentsRelations = relations(comments, ({ one, many }) => ({
  parent: one(comments, {
    fields: [comments.parentId],
    references: [comments.id],
  }),
  author: one(users, { fields: [comments.authorId], references: [users.id] }),
  replies: many(comments),
}));

export const attachmentsRelations = relations(attachments, ({ one }) => ({
  uploadedBy: one(users, {
    fields: [attachments.uploadedBy],
    references: [users.id],
  }),
}));

export const exportJobsRelations = relations(exportJobs, ({ one }) => ({
  study: one(studies, {
    fields: [exportJobs.studyId],
    references: [studies.id],
  }),
  requestedBy: one(users, {
    fields: [exportJobs.requestedBy],
    references: [users.id],
  }),
}));

export const sharedResourcesRelations = relations(
  sharedResources,
  ({ one }) => ({
    study: one(studies, {
      fields: [sharedResources.studyId],
      references: [studies.id],
    }),
    sharedBy: one(users, {
      fields: [sharedResources.sharedBy],
      references: [users.id],
    }),
  }),
);

export const systemSettingsRelations = relations(systemSettings, ({ one }) => ({
  updatedBy: one(users, {
    fields: [systemSettings.updatedBy],
    references: [users.id],
  }),
}));

export const auditLogsRelations = relations(auditLogs, ({ one }) => ({
  user: one(users, { fields: [auditLogs.userId], references: [users.id] }),
}));
