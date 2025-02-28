# System Architecture

## Overview

HRIStudio is built on a modern tech stack centered around Next.js 15's App Router, emphasizing server-side rendering and type safety throughout the application. This document outlines the core architectural decisions and system design.

## Tech Stack

- **Framework:** Next.js 15 (App Router)
- **Language:** TypeScript
- **Database ORM:** Drizzle
- **Authentication:** NextAuth.js (Auth.js)
- **API Layer:** tRPC
- **UI Components:** 
  - Shadcn UI
  - Radix UI
  - Tailwind CSS
- **State Management:** React Context + Hooks
- **Form Handling:** React Hook Form + Zod

## Core Architecture Components

### Next.js App Router Structure

```
src/
├── app/                    # Next.js 15 App Router pages
│   ├── api/               # API routes
│   ├── auth/              # Authentication pages
│   ├── dashboard/         # Dashboard and main application
│   └── layout.tsx         # Root layout
├── components/            # Shared React components
├── lib/                   # Utility functions and shared logic
├── server/               # Server-side code
│   ├── api/              # tRPC routers
│   ├── auth/             # Authentication configuration
│   └── db/               # Database schemas and utilities
└── styles/               # Global styles and Tailwind config
```

### Global Providers

The application is wrapped in several context providers that manage global state:

```typescript
export function Providers({ children }: { children: React.ReactNode }) {
  return (
    <ThemeProvider>
      <PluginStoreProvider>
        <StudyProvider>
          {children}
          <Toaster />
        </StudyProvider>
      </PluginStoreProvider>
    </ThemeProvider>
  );
}
```

### Server Components vs Client Components

We prioritize Server Components for better performance and SEO:

- **Server Components (Default):**
  - Data fetching
  - Static content rendering
  - Layout components
  - Database operations

- **Client Components (Marked with "use client"):**
  - Interactive UI elements
  - Components requiring browser APIs
  - Real-time updates
  - Form handling

## API Layer

### tRPC Integration

Type-safe API routes are implemented using tRPC:

```typescript
export const appRouter = createTRPCRouter({
  study: studyRouter,
  participant: participantRouter,
  experiment: experimentRouter,
});
```

Each router provides strongly-typed procedures:

```typescript
export const studyRouter = createTRPCRouter({
  create: protectedProcedure
    .input(studySchema)
    .mutation(async ({ ctx, input }) => {
      // Implementation
    }),
  // Other procedures...
});
```

### Error Handling

Centralized error handling through tRPC:

```typescript
export const createTRPCContext = async (opts: CreateNextContextOptions) => {
  const session = await getServerAuthSession();
  return {
    session,
    db,
    // Additional context...
  };
};
```

## Database Architecture

### Drizzle ORM Integration

Custom table creation utility to prevent naming conflicts:

```typescript
export const createTable = pgTableCreator((name) => `hs_${name}`);
```

Example schema definition:

```typescript
export const studies = createTable("study", {
  id: integer("id").primaryKey().notNull().generatedAlwaysAsIdentity(),
  title: varchar("title", { length: 256 }).notNull(),
  description: text("description"),
  // Additional fields...
});
```

### Relations

Explicit relation definitions using Drizzle's relations API:

```typescript
export const studiesRelations = relations(studies, ({ one, many }) => ({
  creator: one(users, {
    fields: [studies.createdById],
    references: [users.id],
  }),
  members: many(studyMembers),
  // Additional relations...
}));
```

## Performance Considerations

### Server-Side Rendering

- Leveraging Next.js App Router for optimal server-side rendering
- Minimizing client-side JavaScript
- Implementing proper caching strategies

### Data Fetching

- Using React Suspense for loading states
- Implementing stale-while-revalidate patterns
- Optimizing database queries

### Caching Strategy

- Browser-level caching for static assets
- Server-side caching for API responses
- Plugin store metadata caching (5-minute TTL)

## Security

### Authentication Flow

1. User credentials validation
2. Password hashing with bcrypt
3. Session management with NextAuth
4. Role-based access control

### Data Protection

- Input validation using Zod schemas
- SQL injection prevention through Drizzle ORM
- XSS prevention through proper React escaping
- CSRF protection via Auth.js tokens

## Monitoring and Debugging

### Error Tracking

- Custom error classes for specific scenarios
- Detailed error logging
- Error boundary implementation

### Performance Monitoring

- Web Vitals tracking
- Server-side metrics collection
- Client-side performance monitoring

## Development Workflow

### Type Safety

- Strict TypeScript configuration
- Zod schema validation
- tRPC for end-to-end type safety

### Code Organization

- Feature-based directory structure
- Clear separation of concerns
- Consistent naming conventions

### Testing Strategy

- Unit tests for utility functions
- Integration tests for API routes
- End-to-end tests for critical flows

## Deployment

### Environment Configuration

- Development environment
- Staging environment
- Production environment

### CI/CD Pipeline

- Automated testing
- Type checking
- Linting
- Build verification

## Conclusion

This architecture provides a solid foundation for HRIStudio, emphasizing:

- Type safety
- Performance
- Scalability
- Maintainability
- Security

Future architectural decisions should align with these principles while considering the evolving needs of the platform. 