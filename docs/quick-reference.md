# HRIStudio Quick Reference Guide

## 🚀 **Getting Started (5 Minutes)**

### Prerequisites
- [Bun](https://bun.sh) (package manager)
- [PostgreSQL](https://postgresql.org) 14+
- [Docker](https://docker.com) (optional)

### Quick Setup
```bash
# Clone and install
git clone <repo-url> hristudio
cd hristudio
bun install

# Start database
bun run docker:up

# Setup database
bun db:push
bun db:seed

# Single command now syncs all repositories:
# - Core blocks from localhost:3000/hristudio-core
# - Robot plugins from https://repo.hristudio.com

# Start development
bun dev
```

### Default Login
- **Admin**: `sean@soconnor.dev` / `password123`
- **Researcher**: `alice.rodriguez@university.edu` / `password123`
- **Wizard**: `emily.watson@lab.edu` / `password123`

---

## 📁 **Project Structure**

```
src/
├── app/                    # Next.js App Router pages
│   ├── (auth)/            # Authentication pages
│   ├── (dashboard)/       # Main application
│   └── api/               # API routes
├── components/            # UI components
│   ├── ui/                # shadcn/ui components
│   ├── experiments/       # Feature components
│   ├── studies/
│   ├── participants/
│   └── trials/
├── server/               # Backend code
│   ├── api/routers/      # tRPC routers
│   ├── auth/             # NextAuth config
│   └── db/               # Database schema
└── lib/                  # Utilities
```

---

## 🎯 **Key Concepts**

### Hierarchical Structure
```
Study → Experiment → Trial → Step → Action
```

### User Roles
- **Administrator**: Full system access
- **Researcher**: Create studies, design experiments
- **Wizard**: Execute trials, control robots
- **Observer**: Read-only access

### Core Workflows
1. **Study Creation** → Team setup → Participant recruitment
2. **Experiment Design** → Visual designer → Protocol validation
3. **Trial Execution** → Wizard interface → Data capture
4. **Data Analysis** → Export → Insights

---

## 🛠 **Development Commands**

| Command | Purpose |
|---------|---------|
| `bun dev` | Start development server |
| `bun build` | Build for production |
| `bun typecheck` | TypeScript validation |
| `bun lint` | Code quality checks |
| `bun db:push` | Push schema changes |
| `bun db:seed` | Seed data & sync repositories |
| `bun db:studio` | Open database GUI |

---

## 🌐 **API Reference**

### Base URL
```
http://localhost:3000/api/trpc/
```

### Key Routers
- **`auth`**: Login, logout, registration
- **`studies`**: CRUD operations, team management
- **`experiments`**: Design, configuration, validation
- **`participants`**: Registration, consent, demographics
- **`trials`**: Execution, monitoring, data capture, real-time control
- **`robots`**: Integration, communication, actions, plugins
- **`dashboard`**: Overview stats, recent activity, study progress
- **`admin`**: Repository management, system settings

### Example Usage
```typescript
// Get user's studies
const studies = api.studies.getUserStudies.useQuery();

// Create new experiment
const createExperiment = api.experiments.create.useMutation();
```

---

## 🗄️ **Database Quick Reference**

### Core Tables
```sql
users              -- Authentication & profiles
studies            -- Research projects
experiments        -- Protocol templates
participants       -- Study participants
trials             -- Experiment instances
steps              -- Experiment phases
trial_events       -- Execution logs
robots             -- Available platforms
```

### Key Relationships
```
studies → experiments → trials
studies → participants
trials → trial_events
experiments → steps
```

---

## 🎨 **UI Components**

---

## 🎯 **Trial System Quick Reference**

### Trial Workflow
```
1. Create Study → 2. Design Experiment → 3. Add Participants → 4. Schedule Trial → 5. Execute with Wizard Interface → 6. Analyze Results
```

### Key Trial Pages
- **`/studies/[id]/trials`**: List trials for specific study
- **`/trials/[id]`**: Individual trial details and management
- **`/trials/[id]/wizard`**: Panel-based real-time execution interface
- **`/trials/[id]/analysis`**: Post-trial data analysis

### Trial Status Flow
```
scheduled → in_progress → completed
           ↘ aborted
           ↘ failed
```

### Wizard Interface Architecture (Panel-Based)
The wizard interface uses the same proven panel system as the experiment designer:

#### **Layout Components**
- **PageHeader**: Consistent navigation with breadcrumbs
- **PanelsContainer**: Three-panel resizable layout
- **Proper Navigation**: Dashboard → Studies → [Study] → Trials → [Trial] → Wizard Control

#### **Panel Organization**
```
┌─────────────────────────────────────────────────────────┐
│ PageHeader: Wizard Control                             │
├──────────┬─────────────────────────┬────────────────────┤
│ Left     │ Center                  │ Right              │
│ Panel    │ Panel                   │ Panel              │
│          │                         │                    │
│ Trial    │ Current Step            │ Robot Status       │
│ Controls │ & Wizard Actions        │ Participant Info   │
│ Step     │                         │ Live Events        │
│ List     │                         │ Connection Status  │
└──────────┴─────────────────────────┴────────────────────┘
```

#### **Panel Features**
- **Left Panel**: Trial controls, status, step navigation
- **Center Panel**: Main execution area with current step and wizard actions
- **Right Panel**: Real-time monitoring and context information
- **Resizable**: Drag separators to adjust panel sizes
- **Overflow Contained**: No page-level scrolling, internal panel scrolling

### Technical Features
- **Real-time Control**: Step-by-step protocol execution
- **WebSocket Integration**: Live updates with polling fallback
- **Component Reuse**: 90% code sharing with experiment designer
- **Type Safety**: Complete TypeScript compatibility
- **Mock Robot System**: TurtleBot3 simulation ready for development

---

### Layout Components
```typescript
// Page wrapper with navigation
<PageLayout title="Studies" description="Manage research studies">
  <StudiesTable />
</PageLayout>

// Entity forms (unified pattern)
<EntityForm 
  mode="create"
  entityName="Study"
  form={form}
  onSubmit={handleSubmit}
/>

// Data tables (consistent across entities)
<DataTable 
  columns={studiesColumns}
  data={studies}
  searchKey="name"
/>
```

### Form Patterns
```typescript
// Standard form setup
const form = useForm<StudyFormData>({
  resolver: zodResolver(studySchema),
  defaultValues: { /* ... */ }
});

// Unified submission
const onSubmit = async (data: StudyFormData) => {
  await createStudy.mutateAsync(data);
  router.push(`/studies/${result.id}`);
};
```

---

## 🎯 **Route Structure**

### Study-Scoped Architecture
All study-dependent functionality flows through studies for complete organizational consistency:

```
Platform Routes (Global):
/dashboard                    # Global overview with study filtering
/studies                     # Study management hub
/profile                     # User account management
/admin                      # System administration

Study-Scoped Routes (All Study-Dependent):
/studies/[id]               # Study details and overview
/studies/[id]/participants  # Study participants
/studies/[id]/trials       # Study trials  
/studies/[id]/experiments  # Study experiment protocols
/studies/[id]/plugins      # Study robot plugins
/studies/[id]/analytics    # Study analytics

Individual Entity Routes (Cross-Study):
/trials/[id]              # Individual trial details
/trials/[id]/wizard       # Trial execution interface (TO BE BUILT)
/experiments/[id]         # Individual experiment details
/experiments/[id]/designer # Visual experiment designer

Helpful Redirects (User Guidance):
/participants             # → Study selection guidance
/trials                  # → Study selection guidance
/experiments             # → Study selection guidance
/plugins                 # → Study selection guidance
/analytics               # → Study selection guidance
```

### Architecture Benefits
- **Complete Consistency**: All study-dependent functionality properly scoped
- **Clear Mental Model**: Platform-level vs study-level separation
- **No Duplication**: Single source of truth for each functionality
- **User-Friendly**: Helpful guidance for moved functionality

## 🔐 **Authentication**

### Protecting Routes
```typescript
// Middleware protection
export default withAuth(
  function middleware(request) {
    // Route logic
  },
  {
    callbacks: {
      authorized: ({ token }) => !!token,
    },
  }
);

// Component protection
const { data: session, status } = useSession();
if (status === "loading") return <Loading />;
if (!session) return <SignIn />;
```

### Role Checking
```typescript
// Server-side
ctx.session.user.role === "administrator"

// Client-side
import { useSession } from "next-auth/react";
const hasRole = (role: string) => session?.user.role === role;
```

---

## 🤖 **Robot Integration**

### Core Block System
```typescript
// Core blocks loaded from local repository during development
// Repository sync: localhost:3000/hristudio-core → database

// Block categories (27 total blocks in 4 groups):
// - Events (4): when_trial_starts, when_participant_speaks, etc.
// - Wizard Actions (6): wizard_say, wizard_gesture, etc.  
// - Control Flow (8): wait, repeat, if_condition, etc.
// - Observation (9): observe_behavior, record_audio, etc.
```

### Plugin Repository System
```typescript
// Repository sync (admin only)
await api.admin.repositories.sync.mutate({ id: repoId });

// Plugin installation
await api.robots.plugins.install.mutate({
  studyId: 'study-id',
  pluginId: 'plugin-id'
});

// Get study plugins
const plugins = api.robots.plugins.getStudyPlugins.useQuery({
  studyId: selectedStudyId
});
```

### Plugin Structure
```typescript
interface Plugin {
  id: string;
  name: string;
  version: string;
  trustLevel: 'official' | 'verified' | 'community';
  actionDefinitions: RobotAction[];
  metadata: {
    platform: string;
    category: string;
    repositoryId: string;
  };
}
```

### Repository Integration
- **Robot Plugins**: `https://repo.hristudio.com` (live)
- **Core Blocks**: `localhost:3000/hristudio-core` (development)
- **Auto-sync**: Integrated into `bun db:seed` command
- **Plugin Store**: Browse → Install → Use in experiments

---

## 📊 **Common Patterns**

### Error Handling
```typescript
try {
  await mutation.mutateAsync(data);
  toast.success("Success!");
  router.push("/success-page");
} catch (error) {
  setError(error.message);
  toast.error("Failed to save");
}
```

### Loading States
```typescript
const { data, isLoading, error } = api.studies.getAll.useQuery();

if (isLoading) return <Skeleton />;
if (error) return <ErrorMessage error={error} />;
return <DataTable data={data} />;
```

### Form Validation
```typescript
const schema = z.object({
  name: z.string().min(1, "Name required"),
  description: z.string().min(10, "Description too short"),
  duration: z.number().min(5, "Minimum 5 minutes")
});
```

---

## 🚀 **Deployment**

### Vercel Deployment
```bash
# Install Vercel CLI
bun add -g vercel

# Deploy
vercel --prod

# Environment variables
vercel env add DATABASE_URL
vercel env add NEXTAUTH_SECRET
vercel env add CLOUDFLARE_R2_*
```

### Environment Variables
```bash
# Required
DATABASE_URL=postgresql://...
NEXTAUTH_URL=https://your-domain.com
NEXTAUTH_SECRET=your-secret

# Storage
CLOUDFLARE_R2_ACCOUNT_ID=...
CLOUDFLARE_R2_ACCESS_KEY_ID=...
CLOUDFLARE_R2_SECRET_ACCESS_KEY=...
CLOUDFLARE_R2_BUCKET_NAME=hristudio-files
```

---

## Experiment Designer — Quick Tips

- Panels layout
  - Uses Tailwind-first grid via `PanelsContainer` with fraction-based columns (no hardcoded px).
  - Left/Center/Right panels are minmax(0, …) columns to prevent horizontal overflow.
  - Status bar lives inside the bordered container; no gap below the panels.

- Resizing (no persistence)
  - Drag separators between Left↔Center and Center↔Right to resize panels.
  - Fractions are clamped (min/max) to keep panels usable and avoid page overflow.
  - Keyboard on handles: Arrow keys to resize; Shift+Arrow for larger steps.

- Overflow rules (no page-level X scroll)
  - Root containers: `overflow-hidden`, `min-h-0`.
  - Each panel wrapper: `min-w-0 overflow-hidden`.
  - Each panel content: `overflow-y-auto overflow-x-hidden` (scroll inside the panel).
  - If X scroll appears, clamp the offending child (truncate, `break-words`, `overflow-x-hidden`).

- Action Library scroll
  - Search/categories header and footer are fixed; the list uses internal scroll (`ScrollArea` with `flex-1`).
  - Long lists never scroll the page — only the panel.

- Inspector tabs (shadcn/ui)
  - Single Tabs root controls both header and content.
  - TabsList uses simple grid or inline-flex; triggers are plain `TabsTrigger`.
  - Active state is styled globally (via `globals.css`) using Radix `data-state="active"`.

## 🔧 **Troubleshooting**

### Common Issues

**Build Errors**
```bash
# Clear cache and rebuild
rm -rf .next
bun run build
```

**Database Issues**
```bash
# Reset database
bun db:push --force
bun db:seed
```

**TypeScript Errors**
```bash
# Check types
bun typecheck

# Common fixes
# - Check imports
# - Verify API return types
# - Update schema types
```

### Performance Tips
- Use React Server Components where possible
- Implement proper pagination for large datasets
- Add database indexes for frequently queried fields
- Use optimistic updates for better UX

---

## 📚 **Further Reading**

### Documentation Files
- **[Project Overview](./project-overview.md)**: Complete feature overview
- **[Implementation Details](./implementation-details.md)**: Architecture decisions and patterns
- **[Database Schema](./database-schema.md)**: Complete database documentation
- **[API Routes](./api-routes.md)**: Comprehensive API reference
- **[Core Blocks System](./core-blocks-system.md)**: Repository-based block architecture
- **[Plugin System Guide](./plugin-system-implementation-guide.md)**: Robot integration guide
- **[Project Status](./project-status.md)**: Current development status
- **[Work in Progress](./work_in_progress.md)**: Recent changes and active development

### External Resources
- [Next.js Documentation](https://nextjs.org/docs)
- [tRPC Documentation](https://trpc.io/docs)
- [Drizzle ORM Guide](https://orm.drizzle.team/docs)
- [shadcn/ui Components](https://ui.shadcn.com)

---

## 🎯 **Quick Tips**
### Quick Tips

### Development Workflow
1. Always run `bun typecheck` before commits
2. Use the unified `EntityForm` for all CRUD operations
3. Follow the established component patterns
4. Add proper error boundaries for new features
5. Test with multiple user roles
6. Use single `bun db:seed` for complete setup

### Code Standards
- Use TypeScript strict mode
- Prefer Server Components over Client Components
- Implement proper error handling
- Add loading states for all async operations
- Use Zod for input validation

### Best Practices
- Keep components focused and composable
- Use the established file naming conventions
- Implement proper RBAC for new features
- Add comprehensive logging for debugging
- Follow accessibility guidelines (WCAG 2.1 AA)
- Use repository-based plugins instead of hardcoded robot actions
- Test plugin installation/uninstallation in different studies

### Route Architecture
- **Study-Scoped**: All entity management flows through studies
- **Individual Entities**: Trial/experiment details maintain separate routes
- **Helpful Redirects**: Old routes guide users to new locations
- **Consistent Navigation**: Breadcrumbs reflect the study → entity hierarchy

---

*This quick reference covers the most commonly needed information for HRIStudio development. For detailed implementation guidance, refer to the comprehensive documentation files.*