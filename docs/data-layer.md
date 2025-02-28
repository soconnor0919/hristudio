# Data Layer

## Overview

HRIStudio's data layer is built on PostgreSQL using Drizzle ORM for type-safe database operations. The system implements a comprehensive schema design that supports studies, experiments, participants, and plugin management while maintaining data integrity and proper relationships.

## Database Schema

### Core Tables

#### Studies

```typescript
export const studies = createTable("study", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  createdById: varchar("created_by", { length: 255 })
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const studyMembers = createTable("study_member", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  userId: varchar("user_id", { length: 255 })
    .notNull()
    .references(() => users.id, { onDelete: "cascade" }),
  role: studyRoleEnum("role").notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
});
```

#### Experiments

```typescript
export const experiments = createTable("experiment", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  version: integer("version").notNull().default(1),
  status: experimentStatusEnum("status").notNull().default("draft"),
  steps: jsonb("steps").$type<Step[]>().default([]),
  createdById: varchar("created_by", { length: 255 })
    .notNull()
    .references(() => users.id),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});
```

#### Participants

```typescript
export const participants = createTable("participant", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  studyId: integer("study_id")
    .notNull()
    .references(() => studies.id, { onDelete: "cascade" }),
  identifier: varchar("identifier", { length: 256 }),
  email: varchar("email", { length: 256 }),
  firstName: varchar("first_name", { length: 256 }),
  lastName: varchar("last_name", { length: 256 }),
  notes: text("notes"),
  status: participantStatusEnum("status").notNull().default("active"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});
```

### Plugin Store Tables

```typescript
export const pluginRepositories = createTable("plugin_repository", {
  id: varchar("id", { length: 255 }).primaryKey(),
  name: varchar("name", { length: 255 }).notNull(),
  description: text("description"),
  url: varchar("url", { length: 255 }).notNull(),
  official: boolean("official").default(false).notNull(),
  author: jsonb("author").notNull().$type<{
    name: string;
    email?: string;
    url?: string;
    organization?: string;
  }>(),
  maintainers: jsonb("maintainers").$type<Array<{
    name: string;
    email?: string;
    url?: string;
  }>>(),
  compatibility: jsonb("compatibility").notNull().$type<{
    hristudio: {
      min: string;
      recommended?: string;
    };
    ros2?: {
      distributions: string[];
      recommended?: string;
    };
  }>(),
  stats: jsonb("stats").$type<{
    downloads: number;
    stars: number;
    plugins: number;
  }>(),
  addedById: varchar("added_by", { length: 255 })
    .references(() => users.id),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});
```

## Relations

### Study Relations

```typescript
export const studiesRelations = relations(studies, ({ one, many }) => ({
  creator: one(users, {
    fields: [studies.createdById],
    references: [users.id],
  }),
  members: many(studyMembers),
  participants: many(participants),
  experiments: many(experiments),
}));

export const studyMembersRelations = relations(studyMembers, ({ one }) => ({
  study: one(studies, {
    fields: [studyMembers.studyId],
    references: [studies.id],
  }),
  user: one(users, {
    fields: [studyMembers.userId],
    references: [users.id],
  }),
}));
```

## Data Validation

### Zod Schemas

```typescript
const studySchema = z.object({
  title: z.string().min(1, "Title is required").max(256, "Title is too long"),
  description: z.string().optional(),
});

const participantSchema = z.object({
  identifier: z.string().optional(),
  email: z.string().email().optional(),
  firstName: z.string().optional(),
  lastName: z.string().optional(),
  notes: z.string().optional(),
  status: z.enum(["active", "inactive", "completed", "withdrawn"]),
});

const experimentSchema = z.object({
  title: z.string().min(1, "Title is required").max(256, "Title is too long"),
  description: z.string().optional(),
  steps: z.array(stepSchema),
  status: z.enum(["draft", "active", "archived"]),
});
```

## Query Building

### Type-Safe Queries

```typescript
const study = await db.query.studies.findFirst({
  where: eq(studies.id, studyId),
  with: {
    creator: true,
    members: {
      with: {
        user: true,
      },
    },
    participants: true,
  },
});
```

### Mutations

```typescript
const newStudy = await db.transaction(async (tx) => {
  const [study] = await tx.insert(studies).values({
    title,
    description,
    createdById: userId,
  }).returning();

  await tx.insert(studyMembers).values({
    studyId: study.id,
    userId,
    role: "OWNER",
  });

  return study;
});
```

## Error Handling

### Database Errors

```typescript
try {
  await db.insert(participants).values({
    studyId,
    identifier,
    email,
  });
} catch (error) {
  if (error instanceof PostgresError) {
    if (error.code === "23505") { // Unique violation
      throw new TRPCError({
        code: "CONFLICT",
        message: "A participant with this identifier already exists",
      });
    }
  }
  throw error;
}
```

## Migrations

### Migration Structure

```typescript
import { sql } from "drizzle-orm";
import { createTable, integer, text, timestamp, varchar } from "drizzle-orm/pg-core";

export async function up(db: Database) {
  await db.schema.createTable("study")
    .addColumn("id", "serial", (col) => col.primaryKey())
    .addColumn("title", "varchar(256)", (col) => col.notNull())
    .addColumn("description", "text")
    .addColumn("created_by", "varchar(255)", (col) => 
      col.notNull().references("user.id"))
    .addColumn("created_at", "timestamp", (col) => 
      col.notNull().defaultTo(sql`CURRENT_TIMESTAMP`))
    .addColumn("updated_at", "timestamp");
}

export async function down(db: Database) {
  await db.schema.dropTable("study");
}
```

## Best Practices

1. **Type Safety:**
   - Use Drizzle's type inference
   - Define explicit types for complex queries
   - Validate input with Zod schemas

2. **Performance:**
   - Use appropriate indexes
   - Optimize complex queries
   - Implement caching where needed

3. **Data Integrity:**
   - Use transactions for related operations
   - Implement proper cascading
   - Validate data before insertion

4. **Security:**
   - Sanitize user input
   - Implement proper access control
   - Use parameterized queries

## Future Enhancements

1. **Query Optimization:**
   - Query caching
   - Materialized views
   - Query analysis tools

2. **Data Management:**
   - Data archiving
   - Backup strategies
   - Data export tools

3. **Schema Evolution:**
   - Zero-downtime migrations
   - Schema versioning
   - Backward compatibility

4. **Monitoring:**
   - Query performance metrics
   - Error tracking
   - Usage analytics 