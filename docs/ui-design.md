# UI Design & User Experience

## Design System

### Color System

Our color system is defined in CSS variables with both light and dark mode variants:

```css
:root {
  /* Core colors */
  --background: 0 0% 100%;
  --foreground: 222 47% 11%;
  
  /* Primary colors */
  --primary: 217 91% 60%;
  --primary-foreground: 0 0% 100%;
  
  /* Card colors */
  --card: 0 0% 100%;
  --card-foreground: 222 47% 11%;
  
  /* Additional semantic colors */
  --muted: 210 40% 96%;
  --muted-foreground: 215 16% 47%;
  --accent: 210 40% 96%;
  --accent-foreground: 222 47% 11%;
  
  /* ... additional color definitions ... */
}

.dark {
  --background: 222 47% 11%;
  --foreground: 210 40% 98%;
  /* ... dark mode variants ... */
}
```

### Typography

We use the Geist font family for its clean, modern appearance:

```typescript
import { GeistSans } from 'geist/font/sans';

<body className={cn(
  "min-h-screen bg-background font-sans antialiased",
  GeistSans.className
)}>
```

### Spacing System

Consistent spacing using Tailwind's scale:

- `space-1`: 0.25rem (4px)
- `space-2`: 0.5rem (8px)
- `space-4`: 1rem (16px)
- `space-6`: 1.5rem (24px)
- `space-8`: 2rem (32px)

## Component Architecture

### Base Components

All base components are built on Radix UI primitives and styled with Tailwind:

```typescript
// Example Button Component
const Button = React.forwardRef<
  HTMLButtonElement,
  ButtonProps
>(({ className, variant, size, ...props }, ref) => {
  return (
    <button
      className={cn(
        "inline-flex items-center justify-center rounded-md text-sm font-medium transition-colors",
        "focus-visible:outline-none focus-visible:ring-2",
        "disabled:opacity-50 disabled:pointer-events-none",
        buttonVariants({ variant, size, className })
      )}
      ref={ref}
      {...props}
    />
  );
});
```

### Layout Components

#### Page Layout

```typescript
export function PageLayout({ children }: { children: React.ReactNode }) {
  return (
    <div className="flex h-full min-h-screen w-full">
      <AppSidebar />
      <div className="flex w-0 flex-1 flex-col">
        <Header />
        <main className="flex-1 overflow-auto p-4">
          <PageTransition>
            {children}
          </PageTransition>
        </main>
      </div>
    </div>
  );
}
```

#### Sidebar Navigation

The sidebar uses a floating design with dynamic content based on context:

```typescript
export function AppSidebar({ ...props }: SidebarProps) {
  return (
    <Sidebar 
      collapsible="icon" 
      variant="floating"
      className="border-none"
      {...props}
    >
      <SidebarHeader>
        <StudySwitcher />
      </SidebarHeader>
      <SidebarContent>
        <NavMain items={navItems} />
      </SidebarContent>
      <SidebarFooter>
        <NavUser />
      </SidebarFooter>
      <SidebarRail />
    </Sidebar>
  );
}
```

### Form Components

Forms use React Hook Form with Zod validation:

```typescript
const form = useForm<FormData>({
  resolver: zodResolver(schema),
  defaultValues: {
    title: "",
    description: "",
  },
});

return (
  <Form {...form}>
    <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
      <FormField
        control={form.control}
        name="title"
        render={({ field }) => (
          <FormItem>
            <FormLabel>Title</FormLabel>
            <FormControl>
              <Input {...field} />
            </FormControl>
            <FormMessage />
          </FormItem>
        )}
      />
      {/* Additional form fields */}
    </form>
  </Form>
);
```

## Responsive Design

### Breakpoints

We follow Tailwind's default breakpoints:

- `sm`: 640px
- `md`: 768px
- `lg`: 1024px
- `xl`: 1280px
- `2xl`: 1536px

### Mobile-First Approach

```typescript
export function StudyCard({ study }: StudyCardProps) {
  return (
    <Card className="
      w-full
      p-4
      sm:p-6
      md:hover:shadow-lg
      transition-all
      duration-200
    ">
      {/* Card content */}
    </Card>
  );
}
```

## Animation System

### Transition Utilities

Common transitions are defined in Tailwind config:

```javascript
theme: {
  extend: {
    transitionTimingFunction: {
      'bounce-ease': 'cubic-bezier(0.68, -0.55, 0.265, 1.55)',
    },
  },
}
```

### Page Transitions

Using Framer Motion for smooth page transitions:

```typescript
export function PageTransition({ children }: { children: React.ReactNode }) {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: 20 }}
      transition={{ duration: 0.2 }}
    >
      {children}
    </motion.div>
  );
}
```

## Loading States

### Skeleton Components

```typescript
export function CardSkeleton() {
  return (
    <div className="p-6 space-y-4">
      <Skeleton className="h-7 w-[40%]" />
      <Skeleton className="h-4 w-[60%]" />
      <div className="pt-4">
        <Skeleton className="h-4 w-[25%]" />
      </div>
    </div>
  );
}
```

### Loading Indicators

```typescript
export function LoadingSpinner({ size = "default" }: { size?: "sm" | "default" | "lg" }) {
  return (
    <div
      className={cn(
        "animate-spin rounded-full border-2",
        "border-background border-t-foreground",
        {
          "h-4 w-4": size === "sm",
          "h-6 w-6": size === "default",
          "h-8 w-8": size === "lg",
        }
      )}
    />
  );
}
```

## Accessibility

### ARIA Labels

All interactive components include proper ARIA labels:

```typescript
export function IconButton({ label, icon: Icon, ...props }: IconButtonProps) {
  return (
    <Button
      {...props}
      aria-label={label}
      className="p-2 hover:bg-muted/50 rounded-full"
    >
      <Icon className="h-4 w-4" />
      <span className="sr-only">{label}</span>
    </Button>
  );
}
```

### Keyboard Navigation

Support for keyboard navigation in all interactive components:

```typescript
export function NavigationMenu() {
  return (
    <nav
      role="navigation"
      className="focus-within:outline-none"
      onKeyDown={(e) => {
        if (e.key === "Escape") {
          // Handle escape key
        }
      }}
    >
      {/* Navigation items */}
    </nav>
  );
}
```

## Best Practices

1. **Component Organization:**
   - One component per file
   - Clear prop interfaces
   - Consistent file naming

2. **Style Organization:**
   - Use Tailwind utility classes
   - Extract common patterns to components
   - Maintain consistent spacing

3. **Performance:**
   - Lazy load non-critical components
   - Use React.memo for expensive renders
   - Implement proper loading states

4. **Accessibility:**
   - Include ARIA labels
   - Support keyboard navigation
   - Maintain proper contrast ratios

5. **Testing:**
   - Component unit tests
   - Integration tests for flows
   - Visual regression testing 