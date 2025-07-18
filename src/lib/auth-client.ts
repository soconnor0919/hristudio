// Client-side role utilities without database imports
import type { Session } from "next-auth";

// Role types from schema
export type SystemRole = "administrator" | "researcher" | "wizard" | "observer";
export type StudyRole = "owner" | "researcher" | "wizard" | "observer";

/**
 * Check if the current user has a specific system role
 */
export function hasRole(session: Session | null, role: SystemRole): boolean {
  if (!session?.user?.roles) return false;
  return session.user.roles.some((userRole) => userRole.role === role);
}

/**
 * Check if the current user is an administrator
 */
export function isAdmin(session: Session | null): boolean {
  return hasRole(session, "administrator");
}

/**
 * Check if the current user is a researcher or admin
 */
export function isResearcher(session: Session | null): boolean {
  return hasRole(session, "researcher") || isAdmin(session);
}

/**
 * Check if the current user is a wizard or admin
 */
export function isWizard(session: Session | null): boolean {
  return hasRole(session, "wizard") || isAdmin(session);
}

/**
 * Check if the current user has any of the specified roles
 */
export function hasAnyRole(session: Session | null, roles: SystemRole[]): boolean {
  if (!session?.user?.roles) return false;
  return session.user.roles.some((userRole) =>
    roles.includes(userRole.role)
  );
}

/**
 * Check if a user owns or has admin access to a resource
 */
export function canAccessResource(
  session: Session | null,
  resourceOwnerId: string
): boolean {
  if (!session?.user) return false;

  // Admin can access anything
  if (isAdmin(session)) return true;

  // Owner can access their own resources
  if (session.user.id === resourceOwnerId) return true;

  return false;
}

/**
 * Format role for display
 */
export function formatRole(role: SystemRole): string {
  const roleMap: Record<SystemRole, string> = {
    administrator: "Administrator",
    researcher: "Researcher",
    wizard: "Wizard",
    observer: "Observer",
  };

  return roleMap[role] || role;
}

/**
 * Get role description
 */
export function getRoleDescription(role: SystemRole): string {
  const descriptions: Record<SystemRole, string> = {
    administrator: "Full system access and user management",
    researcher: "Can create and manage studies and experiments",
    wizard: "Can control robots during trials and experiments",
    observer: "Read-only access to studies and trial data",
  };

  return descriptions[role] || "Unknown role";
}

/**
 * Get available roles for assignment
 */
export function getAvailableRoles(): Array<{
  value: SystemRole;
  label: string;
  description: string;
}> {
  const roles: SystemRole[] = ["administrator", "researcher", "wizard", "observer"];

  return roles.map((role) => ({
    value: role,
    label: formatRole(role),
    description: getRoleDescription(role),
  }));
}

/**
 * Get role badge color classes
 */
export function getRoleColor(role: SystemRole): string {
  switch (role) {
    case "administrator":
      return "bg-red-100 text-red-800 border-red-200";
    case "researcher":
      return "bg-blue-100 text-blue-800 border-blue-200";
    case "wizard":
      return "bg-purple-100 text-purple-800 border-purple-200";
    case "observer":
      return "bg-green-100 text-green-800 border-green-200";
    default:
      return "bg-gray-100 text-gray-800 border-gray-200";
  }
}

/**
 * Get role permissions list (client-side mock data)
 */
export function getRolePermissions(role: SystemRole): string[] {
  const rolePermissions: Record<SystemRole, string[]> = {
    administrator: [
      "Full system access",
      "User management",
      "System configuration",
      "Audit logs access",
      "Data export/import",
    ],
    researcher: [
      "Create/manage studies",
      "Design experiments",
      "Analyze trial data",
      "Manage participants",
      "Export research data",
    ],
    wizard: [
      "Control robots during trials",
      "Execute experiment steps",
      "Monitor trial progress",
      "Record interventions",
      "Access wizard interface",
    ],
    observer: [
      "View studies and experiments",
      "Watch trial executions",
      "Access analysis reports",
      "View participant data",
      "Read-only system access",
    ],
  };

  return rolePermissions[role] || [];
}
