# HRIStudio Implementation Details

## 🏗️ **Architecture Overview**

HRIStudio is built on a modern, scalable architecture designed for research teams conducting Human-Robot Interaction studies. The platform follows a three-layer architecture with clear separation of concerns.

### **Technology Stack**

**Frontend**
- **Next.js 15**: App Router with React 19 RC for modern SSR/SSG
- **TypeScript**: Strict mode for complete type safety
- **Tailwind CSS**: Utility-first styling with custom design system
- **shadcn/ui**: Professional UI components built on Radix UI
- **tRPC**: Type-safe client-server communication
- **React Hook Form**: Form handling with Zod validation

**Backend**
- **Next.js API Routes**: Serverless functions on Vercel Edge Runtime
- **tRPC**: End-to-end type-safe API with Zod validation
- **Drizzle ORM**: Type-safe database operations with PostgreSQL
- **NextAuth.js v5**: Authentication with database sessions
- **Bun**: Exclusive package manager and runtime

**Infrastructure**
- **Vercel**: Serverless deployment with global CDN
- **PostgreSQL**: Primary database (Vercel Postgres or external)
- **Cloudflare R2**: S3-compatible object storage for media files
- **WebSockets**: Real-time communication (Edge Runtime compatible)

---

## 🎯 **Key Architecture Decisions**

### **1. Vercel Deployment Strategy**

**Decision**: Deploy exclusively on Vercel's serverless platform

**Rationale**:
- Automatic scaling without infrastructure management
- Built-in CI/CD with GitHub integration
- Global CDN for optimal performance
- Edge Runtime support for real-time features
- Cost-effective for research projects

**Implementation**:
- Use Vercel KV instead of Redis for caching
- Edge-compatible WebSocket implementation
- Serverless function optimization
- Environment variable management via Vercel

### **2. No Redis - Edge Runtime Compatibility**

**Decision**: Use Vercel KV and in-memory caching instead of Redis

**Rationale**:
- Vercel Edge Runtime doesn't support Redis connections
- Vercel KV provides Redis-compatible API with edge distribution
- Simplified deployment without additional infrastructure
- Better performance for globally distributed users

**Implementation**:
```typescript
// Use Vercel KV for session storage
import { kv } from '@vercel/kv';

// Edge-compatible caching
export const cache = {
  get: (key: string) => kv.get(key),
  set: (key: string, value: any, ttl?: number) => kv.set(key, value, { ex: ttl }),
  del: (key: string) => kv.del(key)
};
```

### **3. Next.js 15 with React 19 RC**

**Decision**: Use cutting-edge Next.js 15 with React 19 Release Candidate

**Rationale**:
- Latest performance improvements and features
- Better Server Components support
- Enhanced TypeScript integration
- Future-proof for upcoming React features
- Improved caching and optimization

**Configuration**:
```json
{
  "dependencies": {
    "next": "^15.0.0",
    "react": "rc",
    "react-dom": "rc"
  }
}
```

### **4. Bun Exclusive Package Management**

**Decision**: Use Bun exclusively for all package management and runtime operations

**Rationale**:
- Significantly faster than npm/yarn (2-10x speed improvement)
- Built-in TypeScript support
- Compatible with Node.js ecosystem
- Unified toolchain for development
- Better developer experience

**Usage**:
```bash
# All package operations use Bun
bun install
bun add package-name
bun run script-name
bun build
bun test
```

---

## 🎨 **Unified Editor Experiences**

### **Problem Solved**
Prior to unification, each entity (Studies, Experiments, Participants, Trials) had separate form implementations with duplicated code, inconsistent patterns, and scattered validation logic.

### **EntityForm Component Architecture**

**Central Component**: `src/components/ui/entity-form.tsx`

```typescript
interface EntityFormProps {
  mode: 'create' | 'edit';
  entityName: string;
  entityNamePlural: string;
  backUrl: string;
  listUrl: string;
  title: string;
  description: string;
  icon: React.ComponentType;
  form: UseFormReturn<any>;
  onSubmit: (data: any) => Promise<void>;
  isSubmitting: boolean;
  error: string | null;
  onDelete?: () => Promise<void>;
  isDeleting?: boolean;
  sidebar: React.ReactNode;
  children: React.ReactNode;
}
```

### **Standardized Patterns**

**Layout Structure**:
```typescript
// Consistent 2/3 main + 1/3 sidebar layout
<div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
  <div className="lg:col-span-2">
    {/* Main form content */}
  </div>
  <div className="space-y-6">
    {/* Sidebar with next steps and tips */}
  </div>
</div>
```

**Form Implementation Pattern**:
```typescript
export function EntityForm({ mode, entityId }: EntityFormProps) {
  const router = useRouter();
  const form = useForm<EntityFormData>({
    resolver: zodResolver(entitySchema),
    defaultValues: { /* ... */ },
  });

  // Unified submission logic
  const onSubmit = async (data: EntityFormData) => {
    try {
      if (mode === "create") {
        const result = await createEntity.mutateAsync(data);
        router.push(`/entities/${result.id}`);
      } else {
        await updateEntity.mutateAsync({ id: entityId!, data });
        router.push(`/entities/${entityId}`);
      }
    } catch (error) {
      setError(`Failed to ${mode} entity: ${error.message}`);
    }
  };

  return (
    <EntityForm
      mode={mode}
      entityName="Entity"
      form={form}
      onSubmit={onSubmit}
      // ... other props
    >
      {/* Form fields */}
    </EntityForm>
  );
}
```

### **Achievement Metrics**
- **Significant Code Reduction**: Eliminated form duplication across entities
- **Complete Consistency**: Uniform experience across all entity types
- **Developer Velocity**: Much faster implementation of new forms
- **Maintainability**: Single component for all form improvements

---

## 📊 **DataTable Migration**

### **Enterprise-Grade Data Management**

**Unified Component**: `src/components/ui/data-table.tsx`

```typescript
interface DataTableProps<TData, TValue> {
  columns: ColumnDef<TData, TValue>[];
  data: TData[];
  searchKey?: string;
  searchPlaceholder?: string;
  isLoading?: boolean;
  onExport?: () => void;
  showColumnToggle?: boolean;
  showPagination?: boolean;
  pageSize?: number;
}
```

### **Advanced Features**

**Server-Side Operations**:
```typescript
// Efficient pagination and filtering
const { data: studies, isLoading } = api.studies.getUserStudies.useQuery({
  search: searchTerm,
  page: currentPage,
  limit: pageSize,
  sortBy: sortColumn,
  sortOrder: sortDirection
});
```

**Column Management**:
```typescript
// Dynamic column visibility
const [columnVisibility, setColumnVisibility] = useState({
  createdAt: false,
  updatedAt: false,
  // Show/hide columns based on user preferences
});
```

**Export Functionality**:
```typescript
// Role-based export permissions
const handleExport = async () => {
  if (!hasPermission("export")) return;
  
  const exportData = await api.studies.export.mutate({
    format: "csv",
    filters: currentFilters
  });
  
  downloadFile(exportData, "studies.csv");
};
```

### **Performance Improvements**
- **Much Faster**: Initial page load times
- **Significant Reduction**: Unnecessary API calls
- **Lower**: Client-side memory usage
- **Much Better**: Mobile responsiveness

### **Critical Fixes Applied**

**Horizontal Overflow Solution**:
```css
/* Two-level overflow control */
.page-container {
  overflow-x: hidden; /* Prevent page-wide scrolling */
  overflow-y: auto;   /* Allow vertical scrolling */
}

.table-container {
  overflow-x: auto;   /* Allow table scrolling */
  overflow-y: hidden; /* Prevent vertical table overflow */
}
```

**Responsive Column Management**:
```typescript
// Optimized column display for mobile
const mobileColumns = useMemo(() => {
  return columns.filter(col => 
    isMobile ? col.meta?.essential : true
  );
}, [columns, isMobile]);
```

---

## 🧪 **Development Database & Seed System**

### **Comprehensive Test Environment**

**Seed Script**: `scripts/seed-dev.ts`

```typescript
// Realistic research scenarios
const seedData = {
  studies: [
    {
      name: "Robot-Assisted Learning in Elementary Education",
      institution: "University of Technology",
      irbProtocol: "IRB-2024-001",
      focus: "Mathematics learning for elementary students"
    },
    {
      name: "Elderly Care Robot Acceptance Study", 
      institution: "Research Institute for Aging",
      irbProtocol: "IRB-2024-002",
      focus: "Companion robots in assisted living"
    }
  ],
  participants: [
    {
      code: "CHILD_001",
      demographics: { age: 8, gender: "male", grade: 3 }
    },
    {
      code: "ELDERLY_001", 
      demographics: { age: 78, gender: "female", background: "retired teacher" }
    }
  ]
};
```

### **Research Scenarios Included**

**Elementary Education Study**:
- Math tutoring with NAO robot
- Reading comprehension support
- Child-appropriate interaction protocols
- Learning outcome tracking

**Elderly Care Research**:
- Companion robot acceptance study
- Medication reminder protocols
- Social interaction analysis
- Health monitoring integration

**Navigation Trust Study**:
- Autonomous robot guidance
- Trust measurement in public spaces
- Safety protocol validation

### **Default Access Credentials**
```
Administrator: sean@soconnor.dev / password123
Researcher: alice.rodriguez@university.edu / password123
Wizard: emily.watson@lab.edu / password123
```

### **Instant Setup**
```bash
# Complete environment in under 2 minutes
bun db:push    # Set up schema
bun db:seed    # Load test data
bun dev        # Start development
```

---

## 🔐 **Authentication & Security Architecture**

### **NextAuth.js v5 Implementation**

**Configuration**: `src/server/auth/config.ts`

```typescript
export const authConfig = {
  providers: [
    Credentials({
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" }
      },
      authorize: async (credentials) => {
        const user = await verifyCredentials(credentials);
        return user ? { 
          id: user.id, 
          email: user.email, 
          role: user.role 
        } : null;
      }
    })
  ],
  session: { strategy: "jwt" },
  callbacks: {
    jwt: ({ token, user }) => {
      if (user) token.role = user.role;
      return token;
    },
    session: ({ session, token }) => ({
      ...session,
      user: {
        ...session.user,
        role: token.role as UserRole
      }
    })
  }
} satisfies NextAuthConfig;
```

### **Role-Based Access Control**

**Middleware Protection**: `middleware.ts`

```typescript
export default withAuth(
  function middleware(request) {
    const { pathname } = request.nextUrl;
    const userRole = request.nextauth.token?.role;

    // Admin-only routes
    if (pathname.startsWith('/admin')) {
      return userRole === 'administrator' 
        ? NextResponse.next()
        : NextResponse.redirect('/unauthorized');
    }

    // Researcher routes
    if (pathname.startsWith('/studies/new')) {
      return ['administrator', 'researcher'].includes(userRole!)
        ? NextResponse.next()
        : NextResponse.redirect('/unauthorized');
    }

    return NextResponse.next();
  },
  {
    callbacks: {
      authorized: ({ token }) => !!token
    }
  }
);
```

**API Protection**:
```typescript
// tRPC procedure protection
export const protectedProcedure = publicProcedure.use(({ ctx, next }) => {
  if (!ctx.session?.user) {
    throw new TRPCError({ code: "UNAUTHORIZED" });
  }
  return next({ ctx: { ...ctx, session: ctx.session } });
});

export const adminProcedure = protectedProcedure.use(({ ctx, next }) => {
  if (ctx.session.user.role !== "administrator") {
    throw new TRPCError({ code: "FORBIDDEN" });
  }
  return next();
});
```

---

## 🤖 **Robot Integration Architecture**

### **Plugin System Design**

**Unified Plugin Architecture**: HRIStudio uses a consistent plugin system for both core blocks and robot actions, providing complete architectural consistency.

#### **Core Blocks System**

**Repository Structure**:
```
public/hristudio-core/
├── repository.json           # Repository metadata
├── plugins/
│   ├── index.json           # Plugin index (26 total blocks)
│   ├── events.json          # Event trigger blocks (4 blocks)
│   ├── wizard-actions.json  # Wizard action blocks (6 blocks)
│   ├── control-flow.json    # Control flow blocks (8 blocks)
│   └── observation.json     # Observation blocks (8 blocks)
└── assets/                  # Repository assets
```

**Block Loading Architecture**:
```typescript
class BlockRegistry {
  async loadCoreBlocks() {
    // Fetch from /hristudio-core/plugins/*.json
    // Parse and validate JSON structures
    // Convert to PluginBlockDefinition format
    // Register with unified block system
    // Fallback to minimal blocks if loading fails
  }
}
```

**Core Block Categories**:
- **Events (4)**: `when_trial_starts`, `when_participant_speaks`, `when_timer_expires`, `when_key_pressed`
- **Wizard Actions (6)**: `wizard_say`, `wizard_gesture`, `wizard_show_object`, `wizard_record_note`, `wizard_wait_for_response`, `wizard_rate_interaction`
- **Control Flow (8)**: `wait`, `repeat`, `if_condition`, `parallel`, `sequence`, `random_choice`, `try_catch`, `break`
- **Observation (8)**: `observe_behavior`, `measure_response_time`, `count_events`, `record_audio`, `capture_video`, `log_event`, `survey_question`, `physiological_measure`

#### **Robot Plugin Interface**

**Plugin Interface**:
```typescript
interface RobotPlugin {
  id: string;
  name: string;
  version: string;
  manufacturer: string;
  capabilities: RobotCapability[];
  actions: RobotAction[];
  communicate: (action: RobotAction, params: any) => Promise<ActionResult>;
  connect: () => Promise<ConnectionStatus>;
  disconnect: () => Promise<void>;
}
```

**Action Definition**:
```typescript
interface RobotAction {
  id: string;
  name: string;
  description: string;
  category: 'movement' | 'speech' | 'gesture' | 'led' | 'sensor';
  parameters: ActionParameter[];
  validation: ValidationSchema;
  example: ActionExample;
}
```

### **Communication Protocols**

**RESTful API Support**:
```typescript
class RestApiPlugin implements RobotPlugin {
  async communicate(action: RobotAction, params: any) {
    const response = await fetch(`${this.baseUrl}/api/${action.endpoint}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params)
    });
    return response.json();
  }
}
```

**ROS2 via WebSocket**:
```typescript
class ROS2Plugin implements RobotPlugin {
  private ros: ROSLIB.Ros;

  async connect() {
    this.ros = new ROSLIB.Ros({
      url: `ws://${this.robotHost}:9090`
    });
    
    return new Promise((resolve) => {
      this.ros.on('connection', () => resolve('connected'));
      this.ros.on('error', () => resolve('error'));
    });
  }

  async communicate(action: RobotAction, params: any) {
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: action.topicName,
      messageType: action.messageType
    });
    
    topic.publish(new ROSLIB.Message(params));
  }
}
```

---

## ⚡ **Performance Optimization**

### **Database Optimization**

**Strategic Indexing**:
```sql
-- Performance-critical indexes
CREATE INDEX idx_studies_owner_id ON studies(owner_id);
CREATE INDEX idx_trials_study_id ON trials(study_id);
CREATE INDEX idx_trial_events_trial_id ON trial_events(trial_id);
CREATE INDEX idx_participants_study_id ON participants(study_id);

-- Compound indexes for common queries
CREATE INDEX idx_trials_study_status ON trials(study_id, status);
CREATE INDEX idx_trial_events_trial_timestamp ON trial_events(trial_id, timestamp);
```

**Query Optimization**:
```typescript
// Efficient queries with proper joins and filtering
const getStudyTrials = async (studyId: string, userId: string) => {
  return db
    .select({
      id: trials.id,
      name: trials.name,
      status: trials.status,
      participantName: participants.name,
      experimentName: experiments.name
    })
    .from(trials)
    .innerJoin(experiments, eq(trials.experimentId, experiments.id))
    .innerJoin(participants, eq(trials.participantId, participants.id))
    .innerJoin(studies, eq(experiments.studyId, studies.id))
    .innerJoin(studyMembers, eq(studies.id, studyMembers.studyId))
    .where(
      and(
        eq(studies.id, studyId),
        eq(studyMembers.userId, userId),
        isNull(trials.deletedAt)
      )
    )
    .orderBy(desc(trials.createdAt));
};
```

### **Frontend Optimization**

**Server Components First**:
```typescript
// Prefer Server Components for data fetching
async function StudiesPage() {
  const studies = await api.studies.getUserStudies.query();
  
  return (
    <PageLayout title="Studies">
      <StudiesTable data={studies} />
    </PageLayout>
  );
}
```

**Optimistic Updates**:
```typescript
// Immediate UI feedback with rollback on error
const utils = api.useUtils();
const updateStudy = api.studies.update.useMutation({
  onMutate: async (variables) => {
    await utils.studies.getUserStudies.cancel();
    const previousStudies = utils.studies.getUserStudies.getData();
    
    utils.studies.getUserStudies.setData(undefined, (old) =>
      old?.map(study => 
        study.id === variables.id 
          ? { ...study, ...variables.data }
          : study
      )
    );
    
    return { previousStudies };
  },
  onError: (error, variables, context) => {
    utils.studies.getUserStudies.setData(undefined, context?.previousStudies);
  }
});
```

---

## 🔒 **Security Implementation**

### **Input Validation**

**Comprehensive Zod Schemas**:
```typescript
export const createStudySchema = z.object({
  name: z.string()
    .min(1, "Name is required")
    .max(255, "Name too long")
    .regex(/^[a-zA-Z0-9\s\-_]+$/, "Invalid characters"),
  description: z.string()
    .min(10, "Description must be at least 10 characters")
    .max(2000, "Description too long"),
  irbProtocol: z.string()
    .regex(/^IRB-\d{4}-\d{3}$/, "Invalid IRB protocol format")
    .optional(),
  institution: z.string().max(255).optional()
});
```

**API Validation**:
```typescript
export const studiesRouter = createTRPCRouter({
  create: protectedProcedure
    .input(createStudySchema)
    .mutation(async ({ ctx, input }) => {
      // Role-based authorization
      if (!["administrator", "researcher"].includes(ctx.session.user.role)) {
        throw new TRPCError({ code: "FORBIDDEN" });
      }
      
      // Input sanitization
      const sanitizedInput = {
        ...input,
        name: input.name.trim(),
        description: input.description.trim()
      };
      
      // Create study with audit log
      const study = await ctx.db.transaction(async (tx) => {
        const newStudy = await tx.insert(studies).values({
          ...sanitizedInput,
          ownerId: ctx.session.user.id
        }).returning();
        
        await tx.insert(auditLogs).values({
          userId: ctx.session.user.id,
          action: "create",
          entityType: "study",
          entityId: newStudy[0]!.id,
          changes: sanitizedInput
        });
        
        return newStudy[0];
      });
      
      return study;
    })
});
```

### **Data Protection**

**Audit Logging**:
```typescript
// Comprehensive audit trail
const createAuditLog = async (
  userId: string,
  action: string,
  entityType: string,
  entityId: string,
  changes: Record<string, any>
) => {
  await db.insert(auditLogs).values({
    userId,
    action,
    entityType,
    entityId,
    changes: JSON.stringify(changes),
    timestamp: new Date(),
    ipAddress: getClientIP(),
    userAgent: getUserAgent()
  });
};
```

**Sensitive Data Handling**:
```typescript
// Encrypt sensitive participant data
const encryptSensitiveData = (data: ParticipantData) => {
  return {
    ...data,
    personalInfo: encrypt(JSON.stringify(data.personalInfo)),
    contactInfo: encrypt(JSON.stringify(data.contactInfo))
  };
};
```

---

## 🚀 **Deployment Strategy**

### **Vercel Configuration**

**Project Settings**:
```json
{
  "framework": "nextjs",
  "buildCommand": "bun run build",
  "outputDirectory": ".next",
  "installCommand": "bun install",
  "devCommand": "bun dev"
}
```

**Environment Variables**:
```bash
# Required for production
DATABASE_URL=postgresql://user:pass@host:5432/db
NEXTAUTH_URL=https://your-domain.com
NEXTAUTH_SECRET=your-long-random-secret

# Storage configuration
CLOUDFLARE_R2_ACCOUNT_ID=your-account-id
CLOUDFLARE_R2_ACCESS_KEY_ID=your-access-key
CLOUDFLARE_R2_SECRET_ACCESS_KEY=your-secret-key
CLOUDFLARE_R2_BUCKET_NAME=hristudio-files

# Optional integrations
SENTRY_DSN=your-sentry-dsn
ANALYTICS_ID=your-analytics-id
```

### **Production Optimizations**

**Build Configuration**: `next.config.js`
```javascript
/** @type {import('next').NextConfig} */
const nextConfig = {
  experimental: {
    serverComponentsExternalPackages: ["@node-rs/argon2"]
  },
  images: {
    remotePatterns: [
      {
        protocol: 'https',
        hostname: 'your-r2-domain.com'
      }
    ]
  },
  headers: async () => [
    {
      source: '/:path*',
      headers: [
        { key: 'X-Frame-Options', value: 'DENY' },
        { key: 'X-Content-Type-Options', value: 'nosniff' },
        { key: 'Referrer-Policy', value: 'strict-origin-when-cross-origin' }
      ]
    }
  ]
};
```

**Database Migration**:
```typescript
// Production-safe migration strategy
export const deploymentMigration = {
  // 1. Deploy new code (backward compatible)
  // 2. Run migrations
  // 3. Update environment variables
  // 4. Verify functionality
  // 5. Clean up old code
};
```

---

## 📈 **Monitoring & Observability**

### **Error Tracking**

**Comprehensive Error Handling**:
```typescript
// Global error boundary
export function GlobalErrorBoundary({ children }: { children: React.ReactNode }) {
  return (
    <ErrorBoundary
      FallbackComponent={ErrorFallback}
      onError={(error, errorInfo) => {
        console.error('Application error:', error);
        // Send to monitoring service
        sendToSentry(error, errorInfo);
      }}
    >
      {children}
    </ErrorBoundary>
  );
}
```

**API Error Handling**:
```typescript
// Structured error responses
export const handleTRPCError = (error: unknown) => {
  if (error instanceof TRPCError) {
    return {
      code: error.code,
      message: error.message,
      timestamp: new Date().toISOString()
    };
  }
  
  // Log unexpected errors
  console.error('Unexpected error:', error);
  
  return {
    code: 'INTERNAL_SERVER_ERROR',
    message: 'An unexpected error occurred',
    timestamp: new Date().toISOString()
  };
};
```

### **Performance Monitoring**

**Core Web Vitals Tracking**:
```typescript
// Performance monitoring
export function reportWebVitals(metric: NextWebVitalsMetric) {
  if (metric.label === 'web-vital') {
    console.log(metric);
    
    // Send to analytics
    analytics.track('Web Vital', {
      name: metric.name,
      value: metric.value,
      rating: metric.rating
    });
  }
}
```

---

*This document consolidates all implementation details, architecture decisions, and technical achievements for HRIStudio. It serves as the comprehensive technical reference for the platform's design and implementation.*