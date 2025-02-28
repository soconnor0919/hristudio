# Development Guidelines

## Overview

This document outlines the development practices, coding standards, and workflow guidelines for contributing to HRIStudio. Following these guidelines ensures consistency and maintainability across the codebase.

## Development Environment

### Prerequisites

- Node.js (v18+)
- PostgreSQL (v14+)
- Docker (for local development)
- VS Code (recommended)

### Setup

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/hristudio.git
   cd hristudio
   ```

2. **Install Dependencies:**
   ```bash
   npm install
   ```

3. **Environment Configuration:**
   ```bash
   cp .env.example .env
   # Edit .env with your local settings
   ```

4. **Database Setup:**
   ```bash
   npm run docker:up     # Start PostgreSQL container
   npm run db:push      # Apply database schema
   npm run db:seed      # Seed initial data
   ```

5. **Start Development Server:**
   ```bash
   npm run dev
   ```

## Code Organization

### Directory Structure

```
src/
├── app/                    # Next.js pages and layouts
├── components/            # React components
│   ├── auth/             # Authentication components
│   ├── experiments/      # Experiment-related components
│   ├── layout/           # Layout components
│   ├── navigation/       # Navigation components
│   ├── studies/          # Study-related components
│   └── ui/              # Shared UI components
├── lib/                  # Utility functions and shared logic
│   ├── experiments/     # Experiment-related utilities
│   ├── permissions/     # Permission checking utilities
│   └── plugin-store/    # Plugin store implementation
├── server/              # Server-side code
│   ├── api/            # tRPC routers
│   ├── auth/           # Authentication configuration
│   └── db/             # Database schemas and utilities
└── styles/             # Global styles and Tailwind config
```

### Naming Conventions

1. **Files and Directories:**
   ```typescript
   // Components
   components/auth/sign-in-form.tsx
   components/studies/study-card.tsx

   // Pages
   app/dashboard/studies/[id]/page.tsx
   app/dashboard/experiments/new/page.tsx
   ```

2. **Component Names:**
   ```typescript
   // PascalCase for component names
   export function SignInForm() { ... }
   export function StudyCard() { ... }
   ```

3. **Variables and Functions:**
   ```typescript
   // camelCase for variables and functions
   const userSession = useSession();
   function handleSubmit() { ... }
   ```

## Coding Standards

### TypeScript

1. **Type Definitions:**
   ```typescript
   // Use interfaces for object definitions
   interface StudyProps {
     id: number;
     title: string;
     description?: string;
     createdAt: Date;
   }

   // Use type for unions and intersections
   type Status = "draft" | "active" | "archived";
   ```

2. **Type Safety:**
   ```typescript
   // Use proper type annotations
   function getStudy(id: number): Promise<Study> {
     return db.query.studies.findFirst({
       where: eq(studies.id, id),
     });
   }
   ```

### React Components

1. **Functional Components:**
   ```typescript
   interface ButtonProps {
     variant?: "default" | "outline" | "ghost";
     children: React.ReactNode;
   }

   export function Button({ variant = "default", children }: ButtonProps) {
     return (
       <button className={cn(buttonVariants({ variant }))}>
         {children}
       </button>
     );
   }
   ```

2. **Hooks:**
   ```typescript
   function useStudy(studyId: number) {
     const { data, isLoading } = api.study.getById.useQuery({ id: studyId });
     
     return {
       study: data,
       isLoading,
     };
   }
   ```

### Styling

1. **Tailwind CSS:**
   ```typescript
   // Use Tailwind classes
   <div className="flex items-center justify-between p-4 bg-card">
     <h2 className="text-lg font-semibold">Title</h2>
     <Button className="hover:bg-primary/90">Action</Button>
   </div>
   ```

2. **CSS Variables:**
   ```css
   :root {
     --primary: 217 91% 60%;
     --primary-foreground: 0 0% 100%;
   }

   .custom-element {
     background: hsl(var(--primary));
     color: hsl(var(--primary-foreground));
   }
   ```

## Testing

### Unit Tests

```typescript
describe("StudyCard", () => {
  it("renders study information correctly", () => {
    const study = {
      id: 1,
      title: "Test Study",
      description: "Test Description",
    };

    render(<StudyCard study={study} />);
    
    expect(screen.getByText("Test Study")).toBeInTheDocument();
    expect(screen.getByText("Test Description")).toBeInTheDocument();
  });
});
```

### Integration Tests

```typescript
describe("Study Creation", () => {
  it("creates a new study", async () => {
    const user = userEvent.setup();
    
    render(<CreateStudyForm />);
    
    await user.type(screen.getByLabelText("Title"), "New Study");
    await user.click(screen.getByText("Create Study"));
    
    expect(await screen.findByText("Study created successfully")).toBeInTheDocument();
  });
});
```

## Error Handling

### API Errors

```typescript
try {
  const result = await api.study.create.mutate(data);
  toast({
    title: "Success",
    description: "Study created successfully",
  });
} catch (error) {
  toast({
    title: "Error",
    description: error.message,
    variant: "destructive",
  });
}
```

### Form Validation

```typescript
const form = useForm<FormData>({
  resolver: zodResolver(schema),
  defaultValues: {
    title: "",
    description: "",
  },
});

function onSubmit(data: FormData) {
  try {
    // Form submission logic
  } catch (error) {
    form.setError("root", {
      type: "submit",
      message: "Something went wrong",
    });
  }
}
```

## Performance Optimization

### React Optimization

1. **Memoization:**
   ```typescript
   const MemoizedComponent = memo(function Component({ data }: Props) {
     return <div>{data}</div>;
   });
   ```

2. **Code Splitting:**
   ```typescript
   const DynamicComponent = dynamic(() => import("./HeavyComponent"), {
     loading: () => <Skeleton />,
   });
   ```

### Database Optimization

1. **Efficient Queries:**
   ```typescript
   // Use select to only fetch needed fields
   const study = await db.query.studies.findFirst({
     select: {
       id: true,
       title: true,
     },
     where: eq(studies.id, studyId),
   });
   ```

2. **Batch Operations:**
   ```typescript
   await db.transaction(async (tx) => {
     await Promise.all(
       participants.map(p => 
         tx.insert(participants).values(p)
       )
     );
   });
   ```

## Git Workflow

### Branching Strategy

1. `main` - Production-ready code
2. `develop` - Development branch
3. Feature branches: `feature/feature-name`
4. Bug fixes: `fix/bug-description`

### Commit Messages

```bash
# Format
<type>(<scope>): <description>

# Examples
feat(studies): add study creation workflow
fix(auth): resolve sign-in validation issue
docs(api): update API documentation
```

## Deployment

### Environment Configuration

```bash
# Development
DATABASE_URL="postgresql://postgres:postgres@localhost:5432/hristudio"
NEXTAUTH_URL="http://localhost:3000"

# Production
DATABASE_URL="postgresql://user:pass@production-db/hristudio"
NEXTAUTH_URL="https://hristudio.com"
```

### Build Process

```bash
# Build application
npm run build

# Type check
npm run typecheck

# Run tests
npm run test

# Start production server
npm run start
```

## Best Practices

1. **Code Quality:**
   - Write self-documenting code
   - Add comments for complex logic
   - Follow TypeScript best practices

2. **Security:**
   - Validate all inputs
   - Implement proper authentication
   - Use HTTPS in production

3. **Performance:**
   - Optimize bundle size
   - Implement caching strategies
   - Monitor performance metrics

4. **Maintenance:**
   - Keep dependencies updated
   - Document breaking changes
   - Maintain test coverage 