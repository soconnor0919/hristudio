# HRIStudio

A modern web application for managing human-robot interaction studies, built with Next.js 15, TypeScript, and the App Router.

## Tech Stack

- **Framework**: Next.js 15 with App Router
- **Language**: TypeScript
- **Authentication**: NextAuth.js
- **Database**: PostgreSQL with Drizzle ORM
- **UI Components**: Shadcn UI + Radix UI
- **Styling**: Tailwind CSS
- **API Layer**: tRPC
- **File Storage**: MinIO (S3-compatible)

## Key Principles

### TypeScript Usage
- Use TypeScript for all code files
- Prefer interfaces over types
- Avoid enums; use const objects with `as const` instead
- Use proper type inference with `zod` schemas

### Component Structure
- Use functional components with TypeScript interfaces
- Structure files in this order:
  1. Exported component
  2. Subcomponents
  3. Helper functions
  4. Static content
  5. Types/interfaces

### Naming Conventions
- Use lowercase with dashes for directories (e.g., `components/auth-wizard`)
- Use PascalCase for components
- Use camelCase for functions and variables
- Prefix boolean variables with auxiliary verbs (e.g., `isLoading`, `hasError`)

### Data Management
- Use Drizzle ORM for database operations
- Split names into `firstName` and `lastName` fields
- Use tRPC for type-safe API calls
- Implement proper error handling and loading states

### Authentication
- Use NextAuth.js for authentication
- Handle user sessions with JWT strategy
- Store passwords with bcrypt hashing
- Implement proper CSRF protection

### File Structure
```
src/
├── app/                    # Next.js App Router pages
├── components/            
│   ├── ui/                # Reusable UI components
│   └── layout/            # Layout components
├── server/
│   ├── api/               # tRPC routers
│   ├── auth/              # Authentication config
│   └── db/                # Database schema and config
└── lib/                   # Utility functions
```

### Best Practices

#### Forms
```typescript
// Form Schema
const formSchema = z.object({
  firstName: z.string().min(1, "First name is required"),
  lastName: z.string().min(1, "Last name is required"),
  email: z.string().email(),
  // ...
});

// Form Component
export function MyForm() {
  const form = useForm<z.infer<typeof formSchema>>({
    resolver: zodResolver(formSchema),
    defaultValues: {
      firstName: "",
      lastName: "",
      // ...
    },
  });
}
```

#### Server Components
- Use Server Components by default
- Add 'use client' only when needed for:
  - Event listeners
  - Browser APIs
  - React hooks
  - Client-side state

#### Image Handling
```typescript
// Image Upload
const handleFileUpload = async (file: File) => {
  const formData = new FormData();
  formData.append("file", file);
  const response = await fetch("/api/upload", {
    method: "POST",
    body: formData,
  });
  return response.json();
};

// Image Display
<Image
  src={imageUrl}
  alt="Description"
  width={size}
  height={size}
  className="object-cover"
  priority={isAboveFold}
/>
```

#### Database Schema
```typescript
// User Table
export const users = createTable("user", {
  id: varchar("id", { length: 255 })
    .notNull()
    .primaryKey()
    .$defaultFn(() => crypto.randomUUID()),
  firstName: varchar("first_name", { length: 255 }),
  lastName: varchar("last_name", { length: 255 }),
  email: varchar("email", { length: 255 }).notNull(),
  // ...
});
```

### Performance Optimization
- Use React Server Components where possible
- Implement proper image optimization
- Use dynamic imports for large client components
- Implement proper caching strategies

### Security
- Implement proper CSRF protection
- Use environment variables for sensitive data
- Implement proper input validation
- Use proper content security policies

## Development

```bash
# Install dependencies
pnpm install

# Set up environment variables
cp .env.example .env.local

# Start development server
pnpm dev

# Run type checking
pnpm type-check

# Run linting
pnpm lint
```

## Database Migrations

```bash
# Generate migration
pnpm drizzle-kit generate:pg

# Push migration
pnpm db:push
```

## Deployment

The application is designed to be deployed on any platform that supports Node.js. We recommend using Vercel for the best Next.js deployment experience.

## Contributing

1. Follow the TypeScript guidelines
2. Use the provided component patterns
3. Implement proper error handling
4. Add appropriate tests
5. Follow the commit message convention