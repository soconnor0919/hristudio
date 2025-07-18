# HRIStudio Technical Implementation Guide

## Overview

This guide provides step-by-step technical instructions for implementing HRIStudio using the T3 stack with Next.js, tRPC, Drizzle ORM, NextAuth.js v5, and supporting infrastructure.

## Table of Contents

1. [Project Setup](#project-setup)
2. [Development Environment](#development-environment)
3. [Database Implementation](#database-implementation)
4. [Authentication System](#authentication-system)
5. [tRPC API Implementation](#trpc-api-implementation)
6. [Frontend Architecture](#frontend-architecture)
7. [Real-time Features](#real-time-features)
8. [File Storage System](#file-storage-system)
9. [Plugin System](#plugin-system)
10. [Testing Strategy](#testing-strategy)
11. [Deployment](#deployment)
12. [Performance Optimization](#performance-optimization)
13. [Security Implementation](#security-implementation)

## Project Setup

### 1. Initialize Project

```bash
# Create new Next.js project with T3 stack
bunx create-t3-app@latest hristudio \
  --nextjs \
  --tailwind \
  --trpc \
  --drizzle \
  --app-router \
  --package-manager bun \
  --typescript

# Update to Next.js 15
cd hristudio
bun add next@15 react@rc react-dom@rc

cd hristudio
```

### 2. Install Additional Dependencies

```bash
# Core dependencies
bun add @auth/drizzle-adapter next-auth@beta
bun add @radix-ui/react-dialog @radix-ui/react-dropdown-menu @radix-ui/react-label @radix-ui/react-select @radix-ui/react-slot @radix-ui/react-tabs @radix-ui/react-toast
bun add class-variance-authority clsx tailwind-merge
bun add nuqs zod react-hook-form @hookform/resolvers
bun add @aws-sdk/client-s3 @aws-sdk/s3-request-presigner
bun add ws @types/ws
bun add date-fns
bun add recharts
bun add roslib @types/roslib  # For ROS2 integration via rosbridge

# Development dependencies
bun add -d @types/node
bun add -d drizzle-kit
bun add -d @faker-js/faker
bun add -d vitest @vitest/ui
bun add -d @testing-library/react @testing-library/jest-dom
bun add -d playwright @playwright/test
```

### 3. Project Structure

```
src/
├── app/
│   ├── (auth)/
│   │   ├── login/
│   │   ├── register/
│   │   └── verify/
│   ├── (dashboard)/
│   │   ├── studies/
│   │   ├── experiments/
│   │   ├── trials/
│   │   └── admin/
│   ├── api/
│   │   ├── auth/[...nextauth]/
│   │   ├── trpc/[trpc]/
│   │   └── ws/
│   └── layout.tsx
├── components/
│   ├── ui/                    # Shadcn components
│   ├── experiment/
│   │   ├── designer/
│   │   ├── step-editor/
│   │   └── action-library/
│   ├── trial/
│   │   ├── wizard-interface/
│   │   ├── execution-panel/
│   │   └── quick-actions/
│   └── layout/
│       ├── navigation/
│       ├── sidebar/
│       └── breadcrumbs/
├── features/
│   ├── auth/
│   ├── studies/
│   ├── experiments/
│   ├── trials/
│   ├── participants/
│   ├── plugins/
│   └── analysis/
├── hooks/
├── lib/
│   ├── auth/
│   ├── db/
│   ├── trpc/
│   ├── storage/
│   └── websocket/
├── server/
│   ├── api/
│   └── db/
└── types/
```

## Development Environment

### 1. Environment Variables

Create `.env.local`:

```env
# Database
DATABASE_URL="postgresql://postgres:postgres@localhost:5432/hristudio?sslmode=disable"

# NextAuth
NEXTAUTH_URL="http://localhost:3000"
NEXTAUTH_SECRET="generate-with-openssl-rand-base64-32"

# OAuth Providers (optional)
GOOGLE_CLIENT_ID=""
GOOGLE_CLIENT_SECRET=""
GITHUB_ID=""
GITHUB_SECRET=""

# S3/MinIO
S3_ENDPOINT="http://localhost:9000"
S3_ACCESS_KEY_ID="minioadmin"
S3_SECRET_ACCESS_KEY="minioadmin"
S3_BUCKET_NAME="hristudio"
S3_REGION="us-east-1"

# WebSocket
WS_URL="wss://your-app.vercel.app/api/ws"  # For production on Vercel

# ROS2 Bridge
ROSBRIDGE_URL="ws://localhost:9090"  # Local development
# ROSBRIDGE_URL="wss://your-ros-bridge.com:9090"  # Production

# Email (for production)
SMTP_HOST=""
SMTP_PORT=""
SMTP_USER=""
SMTP_PASS=""
SMTP_FROM=""
```

### 2. Docker Services

Ensure Docker services are running:

```bash
docker-compose up -d
```

### 3. Initialize Database

```bash
# Generate database migrations
bun db:generate

# Run migrations
bun db:migrate

# Seed database (development)
bun db:seed
```

## Database Implementation

### 1. Drizzle Configuration

Create `src/lib/db/index.ts`:

```typescript
import { drizzle } from 'drizzle-orm/postgres-js';
import postgres from 'postgres';
import * as schema from './schema';

const connectionString = process.env.DATABASE_URL!;
const sql = postgres(connectionString, { max: 1 });

export const db = drizzle(sql, { schema });
export type Database = typeof db;
```

### 2. Schema Definition

Create modular schema files:

`src/lib/db/schema/users.ts`:

```typescript
import { pgTable, uuid, varchar, timestamp, boolean } from 'drizzle-orm/pg-core';
import { createInsertSchema, createSelectSchema } from 'drizzle-zod';

export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  emailVerified: timestamp('email_verified'),
  name: varchar('name', { length: 255 }),
  image: varchar('image'),
  createdAt: timestamp('created_at').defaultNow(),
  updatedAt: timestamp('updated_at').defaultNow(),
  deletedAt: timestamp('deleted_at'),
});

export const insertUserSchema = createInsertSchema(users);
export const selectUserSchema = createSelectSchema(users);
export type User = typeof users.$inferSelect;
export type NewUser = typeof users.$inferInsert;
```

### 3. Relationships

`src/lib/db/schema/relations.ts`:

```typescript
import { relations } from 'drizzle-orm';
import { users, studies, studyMembers } from './index';

export const usersRelations = relations(users, ({ many }) => ({
  ownedStudies: many(studies),
  studyMemberships: many(studyMembers),
}));

export const studiesRelations = relations(studies, ({ one, many }) => ({
  creator: one(users, {
    fields: [studies.createdBy],
    references: [users.id],
  }),
  members: many(studyMembers),
  experiments: many(experiments),
}));
```

### 4. Database Utilities

`src/lib/db/utils.ts`:

```typescript
import { db } from './index';
import { sql } from 'drizzle-orm';

export async function withTransaction<T>(
  callback: (tx: typeof db) => Promise<T>
): Promise<T> {
  return await db.transaction(callback);
}

export async function checkUserPermission(
  userId: string,
  studyId: string,
  requiredPermission: string
): Promise<boolean> {
  const result = await db.execute(sql`
    SELECT check_user_permission(${userId}, ${studyId}, ${requiredPermission})
  `);
  return result.rows[0]?.check_user_permission ?? false;
}
```

## Authentication System

### 1. NextAuth Configuration

`src/lib/auth/config.ts`:

```typescript
import { NextAuthConfig } from 'next-auth';
import { DrizzleAdapter } from '@auth/drizzle-adapter';
import CredentialsProvider from 'next-auth/providers/credentials';
import GoogleProvider from 'next-auth/providers/google';
import { db } from '@/lib/db';
import { users, accounts, sessions } from '@/lib/db/schema';
import { z } from 'zod';
import bcrypt from 'bcryptjs';

export const authConfig: NextAuthConfig = {
  adapter: DrizzleAdapter(db, {
    usersTable: users,
    accountsTable: accounts,
    sessionsTable: sessions,
  }),
  session: {
    strategy: 'database',
    maxAge: 30 * 24 * 60 * 60, // 30 days
  },
  pages: {
    signIn: '/login',
    signUp: '/register',
    error: '/auth/error',
    verifyRequest: '/auth/verify',
  },
  providers: [
    CredentialsProvider({
      name: 'credentials',
      credentials: {
        email: { label: 'Email', type: 'email' },
        password: { label: 'Password', type: 'password' },
      },
      async authorize(credentials) {
        const parsed = z
          .object({
            email: z.string().email(),
            password: z.string().min(8),
          })
          .safeParse(credentials);

        if (!parsed.success) return null;

        const user = await db.query.users.findFirst({
          where: eq(users.email, parsed.data.email),
        });

        if (!user || !user.password) return null;

        const isValid = await bcrypt.compare(
          parsed.data.password,
          user.password
        );

        if (!isValid) return null;

        return {
          id: user.id,
          email: user.email,
          name: user.name,
          image: user.image,
        };
      },
    }),
    GoogleProvider({
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
    }),
  ],
  callbacks: {
    async session({ session, user }) {
      // Add user roles to session
      const userWithRoles = await db.query.users.findFirst({
        where: eq(users.id, user.id),
        with: {
          systemRoles: true,
        },
      });

      session.user.id = user.id;
      session.user.roles = userWithRoles?.systemRoles || [];
      
      return session;
    },
  },
};
```

### 2. Auth Utilities

`src/lib/auth/utils.ts`:

```typescript
import { auth } from './index';
import { redirect } from 'next/navigation';

export async function requireAuth() {
  const session = await auth();
  if (!session) {
    redirect('/login');
  }
  return session;
}

export async function requireRole(role: SystemRole) {
  const session = await requireAuth();
  const hasRole = session.user.roles.some(r => r.role === role);
  
  if (!hasRole) {
    throw new Error('Insufficient permissions');
  }
  
  return session;
}
```

## tRPC API Implementation

### 1. tRPC Setup

`src/lib/trpc/trpc.ts`:

```typescript
import { initTRPC, TRPCError } from '@trpc/server';
import { type Session } from 'next-auth';
import superjson from 'superjson';
import { ZodError } from 'zod';
import { db } from '@/lib/db';

export const createTRPCContext = async (opts: {
  headers: Headers;
  session: Session | null;
}) => {
  return {
    db,
    session: opts.session,
    ...opts,
  };
};

// In-memory stores for real-time state (instead of Redis)
export const trialStateStore = new Map<string, TrialState>();
export const activeConnections = new Map<string, Set<WebSocket>>();

const t = initTRPC.context<typeof createTRPCContext>().create({
  transformer: superjson,
  errorFormatter({ shape, error }) {
    return {
      ...shape,
      data: {
        ...shape.data,
        zodError:
          error.cause instanceof ZodError ? error.cause.flatten() : null,
      },
    };
  },
});

export const createTRPCRouter = t.router;
export const publicProcedure = t.procedure;

const enforceUserIsAuthed = t.middleware(({ ctx, next }) => {
  if (!ctx.session?.user) {
    throw new TRPCError({ code: 'UNAUTHORIZED' });
  }
  return next({
    ctx: {
      session: { ...ctx.session, user: ctx.session.user },
    },
  });
});

export const protectedProcedure = t.procedure.use(enforceUserIsAuthed);
```

### 2. Router Implementation

`src/server/api/routers/studies.ts`:

```typescript
import { z } from 'zod';
import { createTRPCRouter, protectedProcedure } from '@/lib/trpc/trpc';
import { studies, studyMembers } from '@/lib/db/schema';
import { TRPCError } from '@trpc/server';
import { eq, and } from 'drizzle-orm';

export const studiesRouter = createTRPCRouter({
  list: protectedProcedure
    .input(z.object({
      page: z.number().min(1).default(1),
      limit: z.number().min(1).max(100).default(20),
      status: z.enum(['draft', 'active', 'completed', 'archived']).optional(),
      myStudiesOnly: z.boolean().default(false),
    }))
    .query(async ({ ctx, input }) => {
      const offset = (input.page - 1) * input.limit;
      
      const whereConditions = [];
      if (input.status) {
        whereConditions.push(eq(studies.status, input.status));
      }
      if (input.myStudiesOnly) {
        const memberStudyIds = await ctx.db
          .select({ studyId: studyMembers.studyId })
          .from(studyMembers)
          .where(eq(studyMembers.userId, ctx.session.user.id));
        
        whereConditions.push(
          inArray(studies.id, memberStudyIds.map(m => m.studyId))
        );
      }

      const [items, totalCount] = await Promise.all([
        ctx.db.query.studies.findMany({
          where: and(...whereConditions),
          limit: input.limit,
          offset,
          with: {
            creator: true,
            members: {
              with: {
                user: true,
              },
            },
          },
          orderBy: (studies, { desc }) => [desc(studies.createdAt)],
        }),
        ctx.db.select({ count: count() }).from(studies).where(and(...whereConditions)),
      ]);

      return {
        items,
        totalCount: totalCount[0].count,
        totalPages: Math.ceil(totalCount[0].count / input.limit),
        currentPage: input.page,
      };
    }),

  create: protectedProcedure
    .input(z.object({
      name: z.string().min(1).max(255),
      description: z.string().optional(),
      institution: z.string().optional(),
      irbProtocol: z.string().optional(),
      metadata: z.record(z.any()).default({}),
    }))
    .mutation(async ({ ctx, input }) => {
      return await ctx.db.transaction(async (tx) => {
        // Create study
        const [study] = await tx.insert(studies).values({
          ...input,
          createdBy: ctx.session.user.id,
        }).returning();

        // Add creator as owner
        await tx.insert(studyMembers).values({
          studyId: study.id,
          userId: ctx.session.user.id,
          role: 'owner',
        });

        // Log activity
        await tx.insert(activityLogs).values({
          studyId: study.id,
          userId: ctx.session.user.id,
          action: 'study.created',
          description: `Created study "${study.name}"`,
        });

        return study;
      });
    }),

  // ... more procedures
});
```

### 3. Root Router

`src/server/api/root.ts`:

```typescript
import { createTRPCRouter } from '@/lib/trpc/trpc';
import { authRouter } from './routers/auth';
import { studiesRouter } from './routers/studies';
import { experimentsRouter } from './routers/experiments';
import { trialsRouter } from './routers/trials';
import { participantsRouter } from './routers/participants';
import { robotsRouter } from './routers/robots';
import { mediaRouter } from './routers/media';
import { analysisRouter } from './routers/analysis';
import { adminRouter } from './routers/admin';

export const appRouter = createTRPCRouter({
  auth: authRouter,
  studies: studiesRouter,
  experiments: experimentsRouter,
  trials: trialsRouter,
  participants: participantsRouter,
  robots: robotsRouter,
  media: mediaRouter,
  analysis: analysisRouter,
  admin: adminRouter,
});

export type AppRouter = typeof appRouter;
```

## Frontend Architecture

### 1. Component Organization

`src/components/experiment/designer/ExperimentDesigner.tsx`:

```typescript
'use client';

import { useState, useCallback } from 'react';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { Canvas } from './Canvas';
import { Toolbar } from './Toolbar';
import { PropertiesPanel } from './PropertiesPanel';
import { useExperiment } from '@/hooks/useExperiment';

interface ExperimentDesignerProps {
  experimentId: string;
}

export function ExperimentDesigner({ experimentId }: ExperimentDesignerProps) {
  const { experiment, updateExperiment } = useExperiment(experimentId);
  const [selectedStepId, setSelectedStepId] = useState<string | null>(null);

  const handleStepSelect = useCallback((stepId: string) => {
    setSelectedStepId(stepId);
  }, []);

  const handleStepUpdate = useCallback((stepId: string, updates: Partial<Step>) => {
    updateExperiment({
      steps: experiment.steps.map(step =>
        step.id === stepId ? { ...step, ...updates } : step
      ),
    });
  }, [experiment, updateExperiment]);

  return (
    <DndProvider backend={HTML5Backend}>
      <div className="flex h-full">
        <Toolbar className="w-64 border-r" />
        <div className="flex-1 flex flex-col">
          <Canvas
            steps={experiment.steps}
            onStepSelect={handleStepSelect}
            onStepUpdate={handleStepUpdate}
            selectedStepId={selectedStepId}
          />
        </div>
        {selectedStepId && (
          <PropertiesPanel
            step={experiment.steps.find(s => s.id === selectedStepId)!}
            onUpdate={(updates) => handleStepUpdate(selectedStepId, updates)}
            className="w-80 border-l"
          />
        )}
      </div>
    </DndProvider>
  );
}
```

### 2. Custom Hooks

`src/hooks/useExperiment.ts`:

```typescript
import { api } from '@/lib/trpc/react';
import { useRouter } from 'next/navigation';
import { toast } from '@/components/ui/use-toast';

export function useExperiment(experimentId: string) {
  const router = useRouter();
  const utils = api.useUtils();

  const { data: experiment, isLoading } = api.experiments.get.useQuery({
    id: experimentId,
  });

  const updateMutation = api.experiments.update.useMutation({
    onSuccess: () => {
      utils.experiments.get.invalidate({ id: experimentId });
      toast({
        title: 'Experiment updated',
        description: 'Your changes have been saved.',
      });
    },
    onError: (error) => {
      toast({
        title: 'Update failed',
        description: error.message,
        variant: 'destructive',
      });
    },
  });

  const updateExperiment = (updates: Partial<Experiment>) => {
    updateMutation.mutate({
      id: experimentId,
      ...updates,
    });
  };

  return {
    experiment,
    isLoading,
    updateExperiment,
    isUpdating: updateMutation.isLoading,
  };
}
```

### 3. Server Components

`src/app/(dashboard)/studies/[studyId]/page.tsx`:

```typescript
import { Suspense } from 'react';
import { notFound } from 'next/navigation';
import { requireAuth } from '@/lib/auth/utils';
import { api } from '@/lib/trpc/server';
import { StudyDashboard } from '@/components/study/StudyDashboard';
import { StudyDashboardSkeleton } from '@/components/study/StudyDashboardSkeleton';

interface StudyPageProps {
  params: {
    studyId: string;
  };
}

export default async function StudyPage({ params }: StudyPageProps) {
  await requireAuth();

  const study = await api.studies.get({
    id: params.studyId,
  });

  if (!study) {
    notFound();
  }

  return (
    <Suspense fallback={<StudyDashboardSkeleton />}>
      <StudyDashboard study={study} />
    </Suspense>
  );
}
```

## Real-time Features

### 1. WebSocket Server

`src/server/websocket/server.ts`:

```typescript
// For Vercel deployment, we use Edge Runtime WebSocket API
// src/app/api/ws/route.ts
import { NextRequest } from 'next/server';
import { verifySession } from '@/lib/auth/session';
import { TrialExecutionHandler } from '@/lib/websocket/handlers/trial-execution';
import { ObserverHandler } from '@/lib/websocket/handlers/observer';

export const runtime = 'edge';

export async function GET(req: NextRequest) {
  const { searchParams } = new URL(req.url);
  const token = searchParams.get('token');
  
  if (req.headers.get('upgrade') !== 'websocket') {
    return new Response('Expected websocket', { status: 400 });
  }

const handlers = {
  trial: new TrialExecutionHandler(),
  observer: new ObserverHandler(),
};

wss.on('connection', async (ws, req) => {
  const { query } = parse(req.url!, true);
  const token = query.token as string;
  
  try {
    const session = await verifySession(token);
    if (!session) {
      ws.close(1008, 'Invalid session');
      return;
    }

    const handler = handlers[query.type as keyof typeof handlers];
    if (!handler) {
      ws.close(1008, 'Invalid connection type');
      return;
    }

    await handler.handleConnection(ws, session, query);
  } catch (error) {
    console.error('WebSocket connection error:', error);
    ws.close(1011, 'Internal error');
  }
});

  const { socket, response } = Deno.upgradeWebSocket(req);
  
  socket.onopen = () => handleConnection(socket, session);
  socket.onmessage = (event) => handleMessage(socket, event.data);
  socket.onclose = () => handleDisconnect(socket);
  
  return response;
}
```

### 2. Client WebSocket Hook

`src/hooks/useWebSocket.ts`:

```typescript
import { useEffect, useRef, useState, useCallback } from 'react';
import { useSession } from 'next-auth/react';

interface UseWebSocketOptions {
  url: string;
  onMessage?: (data: any) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  reconnectAttempts?: number;
}

export function useWebSocket({
  url,
  onMessage,
  onConnect,
  onDisconnect,
  reconnectAttempts = 5,
}: UseWebSocketOptions) {
  const { data: session } = useSession();
  const ws = useRef<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<any>(null);
  const reconnectCount = useRef(0);

  const connect = useCallback(() => {
    if (!session?.user) return;

    const wsUrl = new URL(url);
    wsUrl.searchParams.set('token', session.user.sessionToken);

    ws.current = new WebSocket(wsUrl.toString());

    ws.current.onopen = () => {
      setIsConnected(true);
      reconnectCount.current = 0;
      onConnect?.();
    };

    ws.current.onmessage = (event) => {
      const data = JSON.parse(event.data);
      setLastMessage(data);
      onMessage?.(data);
    };

    ws.current.onclose = () => {
      setIsConnected(false);
      onDisconnect?.();

      if (reconnectCount.current < reconnectAttempts) {
        reconnectCount.current++;
        setTimeout(connect, 1000 * Math.pow(2, reconnectCount.current));
      }
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };
  }, [session, url, onConnect, onDisconnect, onMessage, reconnectAttempts]);

  useEffect(() => {
    connect();

    return () => {
      ws.current?.close();
    };
  }, [connect]);

  const sendMessage = useCallback((data: any) => {
    if (ws.current?.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(data));
    }
  }, []);

  return {
    isConnected,
    sendMessage,
    lastMessage,
  };
}
```

## File Storage System

### 1. S3/MinIO Client

`src/lib/storage/s3.ts`:

```typescript
import {
  S3Client,
  PutObjectCommand,
  GetObjectCommand,
  DeleteObjectCommand,
} from '@aws-sdk/client-s3';
import { getSignedUrl } from '@aws-sdk/s3-request-presigner';

const s3Client = new S3Client({
  endpoint: process.env.S3_ENDPOINT,
  region: process.env.S3_REGION || 'us-east-1',
  credentials: {
    accessKeyId: process.env.S3_ACCESS_KEY_ID!,
    secretAccessKey: process.env.S3_SECRET_ACCESS_KEY!,
  },
  forcePathStyle: true, // Required for MinIO
});

export async function uploadFile(
  key: string,
  body: Buffer | Uint8Array | string,
  contentType?: string
) {
  const command = new PutObjectCommand({
    Bucket: process.env.S3_BUCKET_NAME,
    Key: key,
    Body: body,
    ContentType: contentType,
  });

  return await s3Client.send(command);
}

export async function getPresignedUrl(
  key: string,
  expiresIn: number = 3600
): Promise<string> {
  const command = new GetObjectCommand({
    Bucket: process.env.S3_BUCKET_NAME,
    Key: key,
  });

  return await getSignedUrl(s3Client, command, { expiresIn });
}

export async function deleteFile(key: string) {
  const command = new DeleteObjectCommand({
    Bucket: process.env.S3_BUCKET_NAME,
    Key: key,
  });

  return await s3Client.send(command);
}
```

### 2. File Upload Handler

`src/features/media/upload.ts`:

```typescript
import { z } from 'zod';
import { uploadFile } from '@/lib/storage/s3';
import { db } from '@/lib/db';
import { mediaCaptures } from '@/lib/db/schema';

const uploadSchema = z.object({
  trialId: z.string().uuid(),
  file: z.instanceof(File),
  mediaType: z.enum(['video', 'audio', 'image']),
  startTimestamp: z.date(),
  endTimestamp: z.date().optional(),
});

export async function handleMediaUpload(input: z.infer<typeof uploadSchema>) {
  const { trialId, file, mediaType, startTimestamp, endTimestamp } = input;

  // Generate unique key
  const key = `trials/${trialId}/${mediaType}/${Date.now()}-${file.name}`;

  // Upload to S3
  const buffer = await file.arrayBuffer();
  await uploadFile(key, Buffer.from(buffer), file.type);

  // Save metadata to database
  const [mediaRecord] = await db.insert(mediaCaptures).values({
    trialId,
    mediaType,
    storagePath: key,
    fileSize: file.size,
    format: file.type,
    startTimestamp,
    endTimestamp,
    metadata: {
      originalName: file.name,
      uploadedAt: new Date(),
    },
  }).returning();

  return mediaRecord;
}
```

## Plugin System

### 1. Plugin Interface

`src/lib/plugins/types.ts`:

```typescript
import { z } from 'zod';

export interface RobotPlugin {
  id: string;
  name: string;
  version: string;
  robotId: string;
  
  // Configuration
  configSchema: z.ZodSchema;
  defaultConfig: Record<string, any>;
  
  // Capabilities
  actions: ActionDefinition[];
  
  // Lifecycle
  initialize(config: any): Promise<void>;
  connect(): Promise<boolean>;
  disconnect(): Promise<void>;
  
  // Execution
  executeAction(action: Action, params: any): Promise<ActionResult>;
  
  // State
  getState(): Promise<RobotState>;
}

export interface ActionDefinition {
  id: string;
  name: string;
  description: string;
  category: string;
  icon?: string;
  parameterSchema: z.ZodSchema;
  timeout?: number;
  retryable?: boolean;
}

export interface ActionResult {
  success: boolean;
  data?: any;
  error?: string;
  duration: number;
}

export interface RobotState {
  connected: boolean;
  battery?: number;
  position?: { x: number; y: number; z: number };
  sensors?: Record<string, any>;
}
```

### 2. Plugin Manager

`src/lib/plugins/manager.ts`:

```typescript
import { RobotPlugin } from './types';
import { db } from '@/lib/db';
import { plugins, studyPlugins } from '@/lib/db/schema';

export class PluginManager {
  private plugins = new Map<string, RobotPlugin>();
  private instances = new Map<string, RobotPlugin>();

  async loadPlugin(pluginId: string): Promise<RobotPlugin> {
    if (this.plugins.has(pluginId)) {
      return this.plugins.get(plug