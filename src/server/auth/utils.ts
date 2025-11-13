import { and, eq } from "drizzle-orm";
import type { Session } from "next-auth";
import { redirect } from "next/navigation";
import { db } from "~/server/db";
import { users, userSystemRoles } from "~/server/db/schema";
import { auth } from "./index";

// Role types from schema
export type SystemRole = "administrator" | "researcher" | "wizard" | "observer";
export type StudyRole = "owner" | "researcher" | "wizard" | "observer";

/**
 * Get the current session or redirect to login
 */
export async function requireAuth() {
  const session = await auth();
  if (!session?.user) {
    redirect("/auth/signin");
  }
  return session;
}

/**
 * Get the current session without redirecting
 */
export async function getSession() {
  return await auth();
}

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
 * Require admin role or redirect
 */
export async function requireAdmin() {
  const session = await requireAuth();
  if (!isAdmin(session)) {
    redirect("/unauthorized");
  }
  return session;
}

/**
 * Require researcher role or redirect
 */
export async function requireResearcher() {
  const session = await requireAuth();
  if (!isResearcher(session)) {
    redirect("/unauthorized");
  }
  return session;
}

/**
 * Get user roles from database
 */
export async function getUserRoles(userId: string) {
  const userWithRoles = await db.query.users.findFirst({
    where: eq(users.id, userId),
    with: {
      systemRoles: {
        with: {
          grantedByUser: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
        },
      },
    },
  });

  return userWithRoles?.systemRoles ?? [];
}

/**
 * Grant a system role to a user
 */
export async function grantRole(
  userId: string,
  role: SystemRole,
  grantedBy: string
) {
  // Check if user already has this role
  const existingRole = await db.query.userSystemRoles.findFirst({
    where: and(
      eq(userSystemRoles.userId, userId),
      eq(userSystemRoles.role, role)
    ),
  });

  if (existingRole) {
    throw new Error(`User already has role: ${role}`);
  }

  // Grant the role
  const newRole = await db
    .insert(userSystemRoles)
    .values({
      userId,
      role,
      grantedBy,
    })
    .returning();

  return newRole[0];
}

/**
 * Revoke a system role from a user
 */
export async function revokeRole(userId: string, role: SystemRole) {
  const deletedRole = await db
    .delete(userSystemRoles)
    .where(
      and(
        eq(userSystemRoles.userId, userId),
        eq(userSystemRoles.role, role)
      )
    )
    .returning();

  if (deletedRole.length === 0) {
    throw new Error(`User does not have role: ${role}`);
  }

  return deletedRole[0];
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
