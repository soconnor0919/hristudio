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
- **`trials`**: Execution, monitoring, data capture
- **`robots`**: Integration, communication, actions, plugins
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
### Further Reading

### Documentation Files
- **[Project Overview](./project-overview.md)**: Complete feature overview
- **[Implementation Details](./implementation-details.md)**: Architecture decisions and patterns
- **[Database Schema](./database-schema.md)**: Complete database documentation
- **[API Routes](./api-routes.md)**: Comprehensive API reference
- **[Core Blocks System](./core-blocks-system.md)**: Repository-based block architecture
- **[Plugin System Guide](./plugin-system-implementation-guide.md)**: Robot integration guide
- **[Project Status](./project-status.md)**: Current development status

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

---

*This quick reference covers the most commonly needed information for HRIStudio development. For detailed implementation guidance, refer to the comprehensive documentation files.*