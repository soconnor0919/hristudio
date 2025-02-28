# Authentication & Permissions System

## Overview

HRIStudio implements a robust authentication and role-based access control (RBAC) system using NextAuth.js (Auth.js) and a custom permissions framework. This system ensures secure access to resources and proper isolation of sensitive data.

## Authentication

### NextAuth Configuration

```typescript
export const authConfig = {
  adapter: DrizzleAdapter(db),
  providers: [
    CredentialsProvider({
      name: "credentials",
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" }
      },
      async authorize(credentials) {
        // Credential validation logic
      }
    })
  ],
  callbacks: {
    session: ({ session, user }) => ({
      ...session,
      user: {
        ...session.user,
        id: user.id,
        name: user.firstName && user.lastName 
          ? `${user.firstName} ${user.lastName}` 
          : null,
        firstName: user.firstName,
        lastName: user.lastName,
      },
    }),
  },
  pages: {
    signIn: '/auth/signin',
  },
};
```

### Session Management

```typescript
interface Session {
  user: {
    id: string;
    email: string;
    firstName: string | null;
    lastName: string | null;
  } & DefaultSession["user"];
}
```

## Role-Based Access Control

### Role Hierarchy

1. **Owner**
   - Single owner per study
   - Full control over all aspects
   - Can delete study or transfer ownership
   - Can manage all other roles

2. **Admin**
   - Multiple admins allowed
   - Can manage participants and experiments
   - Cannot delete study or transfer ownership
   - Can invite and manage other users (except Owner)

3. **Principal Investigator (PI)**
   - Scientific oversight role
   - Full access to participant data
   - Can manage experiment protocols
   - Cannot modify core study settings

4. **Wizard**
   - Operates robots during experiments
   - Can control live experiment sessions
   - Limited view of participant data
   - Cannot modify study design

5. **Researcher**
   - Can view and analyze data
   - Access to anonymized information
   - Cannot modify study or participant data
   - Cannot run experiment trials

6. **Observer**
   - Can view live experiments
   - Access to anonymized data
   - Can add annotations
   - Cannot modify any study aspects

### Permission Categories

```typescript
export const PERMISSIONS = {
  // Study Management
  CREATE_STUDY: "create_study",
  DELETE_STUDY: "delete_study",
  EDIT_STUDY: "edit_study",
  TRANSFER_OWNERSHIP: "transfer_ownership",
  
  // Participant Management
  VIEW_PARTICIPANTS: "view_participants",
  ADD_PARTICIPANT: "add_participant",
  EDIT_PARTICIPANT: "edit_participant",
  DELETE_PARTICIPANT: "delete_participant",
  VIEW_PARTICIPANT_NAMES: "view_participant_names",
  
  // Experiment Management
  CREATE_EXPERIMENT: "create_experiment",
  EDIT_EXPERIMENT: "edit_experiment",
  DELETE_EXPERIMENT: "delete_experiment",
  RUN_EXPERIMENT: "run_experiment",
  
  // Data Access
  EXPORT_DATA: "export_data",
  VIEW_ANALYTICS: "view_analytics",
  
  // User Management
  INVITE_USERS: "invite_users",
  MANAGE_ROLES: "manage_roles",
} as const;
```

### Role-Permission Matrix

```typescript
export const ROLE_PERMISSIONS: Record<Role, Permission[]> = {
  OWNER: Object.values(PERMISSIONS),
  ADMIN: [
    PERMISSIONS.EDIT_STUDY,
    PERMISSIONS.VIEW_PARTICIPANTS,
    PERMISSIONS.ADD_PARTICIPANT,
    PERMISSIONS.EDIT_PARTICIPANT,
    PERMISSIONS.DELETE_PARTICIPANT,
    PERMISSIONS.VIEW_PARTICIPANT_NAMES,
    PERMISSIONS.CREATE_EXPERIMENT,
    PERMISSIONS.EDIT_EXPERIMENT,
    PERMISSIONS.DELETE_EXPERIMENT,
    PERMISSIONS.RUN_EXPERIMENT,
    PERMISSIONS.EXPORT_DATA,
    PERMISSIONS.VIEW_ANALYTICS,
    PERMISSIONS.INVITE_USERS,
    PERMISSIONS.MANAGE_ROLES,
  ],
  PRINCIPAL_INVESTIGATOR: [
    PERMISSIONS.VIEW_PARTICIPANTS,
    PERMISSIONS.ADD_PARTICIPANT,
    PERMISSIONS.EDIT_PARTICIPANT,
    PERMISSIONS.VIEW_PARTICIPANT_NAMES,
    PERMISSIONS.CREATE_EXPERIMENT,
    PERMISSIONS.EDIT_EXPERIMENT,
    PERMISSIONS.RUN_EXPERIMENT,
    PERMISSIONS.EXPORT_DATA,
    PERMISSIONS.VIEW_ANALYTICS,
  ],
  // ... additional role permissions
};
```

## Implementation

### Permission Checking

```typescript
export async function checkPermissions({
  studyId,
  permission,
  session,
}: {
  studyId?: number;
  permission: Permission;
  session: Session | null;
}): Promise<void> {
  if (!session?.user) {
    throw new TRPCError({
      code: "UNAUTHORIZED",
      message: "You must be logged in to perform this action",
    });
  }

  // Anyone who is logged in can create a study
  if (!studyId) {
    if (permission === "CREATE_STUDY") {
      return;
    }
    throw new TRPCError({
      code: "BAD_REQUEST",
      message: "Study ID is required for this action",
    });
  }

  const membership = await db.query.studyMembers.findFirst({
    where: and(
      eq(studyMembers.studyId, studyId),
      eq(studyMembers.userId, session.user.id),
    ),
  });

  if (!membership) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You do not have permission to perform this action",
    });
  }

  const normalizedRole = membership.role.toUpperCase() as keyof typeof ROLE_PERMISSIONS;
  const permittedActions = ROLE_PERMISSIONS[normalizedRole] ?? [];

  if (normalizedRole === "OWNER") {
    return;
  }

  if (!permittedActions.includes(permission)) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You do not have permission to perform this action",
    });
  }
}
```

### Database Schema

```typescript
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

## UI Integration

### Protected Routes

```typescript
export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode
}) {
  const { data: session, status } = useSession()
  const router = useRouter()

  useEffect(() => {
    if (status === "unauthenticated") {
      router.replace("/auth/signin")
    }
  }, [status, router])

  if (!session) {
    return null
  }

  return (
    // Layout content
  );
}
```

### Conditional Rendering

```typescript
export function ParticipantDetails({ participant, role }: Props) {
  const canViewIdentifiableInfo = [
    ROLES.OWNER,
    ROLES.ADMIN,
    ROLES.PRINCIPAL_INVESTIGATOR
  ].includes(role);

  return (
    <div>
      {canViewIdentifiableInfo ? (
        <div>
          <h3>{participant.name}</h3>
          <p>{participant.email}</p>
        </div>
      ) : (
        <div>
          <h3>Participant {participant.id}</h3>
          <p>[Redacted]</p>
        </div>
      )}
    </div>
  );
}
```

## Security Considerations

### Password Handling

- Passwords are hashed using bcrypt before storage
- Minimum password requirements enforced
- Rate limiting on authentication attempts

### Session Security

- CSRF protection enabled
- Secure session cookies
- Session expiration and renewal

### Data Access

- Row-level security through role checks
- Audit logging of sensitive operations
- Data encryption at rest

## Best Practices

1. **Permission Checking:**
   - Always check permissions before sensitive operations
   - Use the checkPermissions utility consistently
   - Include proper error messages

2. **Role Assignment:**
   - Validate role assignments
   - Maintain role hierarchy
   - Prevent privilege escalation

3. **UI Security:**
   - Hide sensitive UI elements based on permissions
   - Clear error messages without exposing internals
   - Proper loading states during authentication

4. **Audit Trail:**
   - Log authentication attempts
   - Track permission changes
   - Monitor sensitive data access

## Future Enhancements

1. **Advanced Authentication:**
   - Multi-factor authentication
   - OAuth provider integration
   - SSO support

2. **Enhanced Permissions:**
   - Custom role creation
   - Temporary permissions
   - Permission inheritance

3. **Audit System:**
   - Detailed activity logging
   - Security alerts
   - Compliance reporting

4. **UI Improvements:**
   - Role management interface
   - Permission visualization
   - Audit log viewer 