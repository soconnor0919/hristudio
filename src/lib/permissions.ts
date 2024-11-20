import { db } from "~/db";
import { permissions, rolePermissions, userRoles } from "~/db/schema";
import { eq } from "drizzle-orm";

// Define permission codes
export const PERMISSIONS = {
  VIEW_PARTICIPANT_NAMES: 'view_participant_names',
  CREATE_PARTICIPANT: 'create_participant',
  DELETE_PARTICIPANT: 'delete_participant',
  CREATE_STUDY: 'create_study',
  DELETE_STUDY: 'delete_study',
  MANAGE_ROLES: 'manage_roles',
} as const;

export type PermissionCode = keyof typeof PERMISSIONS;

// Cache user permissions
const userPermissionsCache = new Map<string, Set<string>>();

export async function getUserPermissions(userId: string): Promise<Set<string>> {
  // Check cache first
  const cached = userPermissionsCache.get(userId);
  if (cached) return cached;

  // Query permissions from database
  const userPerms = await db
    .select({
      permissionCode: permissions.code,
    })
    .from(userRoles)
    .leftJoin(rolePermissions, eq(userRoles.roleId, rolePermissions.roleId))
    .leftJoin(permissions, eq(rolePermissions.permissionId, permissions.id))
    .where(eq(userRoles.userId, userId));
  const permSet = new Set<string>(userPerms.map(p => p.permissionCode).filter((code): code is string => code !== null));
  userPermissionsCache.set(userId, permSet);
  
  return permSet;
}

export async function hasPermission(userId: string, permissionCode: string): Promise<boolean> {
  const userPerms = await getUserPermissions(userId);
  return userPerms.has(permissionCode);
}

// Clear cache for user
export function clearUserPermissionsCache(userId: string) {
  userPermissionsCache.delete(userId);
}
