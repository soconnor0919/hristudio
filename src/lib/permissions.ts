import { eq, and } from "drizzle-orm";
import { db } from "~/db";
import { 
  permissionsTable, 
  userRolesTable, 
  rolePermissionsTable 
} from "~/db/schema";

export const PERMISSIONS = {
  VIEW_PARTICIPANT_NAMES: "view_participant_names",
  CREATE_PARTICIPANT: "create_participant",
  DELETE_PARTICIPANT: "delete_participant",
  CREATE_STUDY: "create_study",
  DELETE_STUDY: "delete_study",
  MANAGE_ROLES: "manage_roles",
} as const;

export type PermissionCode = keyof typeof PERMISSIONS;

export async function getUserPermissions(userId: string) {
  // Get all permissions for the user through their roles
  const userPermissions = await db
    .select({
      permissionCode: permissionsTable.code,
    })
    .from(userRolesTable)
    .innerJoin(
      rolePermissionsTable,
      eq(userRolesTable.roleId, rolePermissionsTable.roleId)
    )
    .innerJoin(
      permissionsTable,
      eq(rolePermissionsTable.permissionId, permissionsTable.id)
    )
    .where(eq(userRolesTable.userId, userId));

  return userPermissions.map(p => p.permissionCode);
}

export async function hasPermission(userId: string, permissionCode: string) {
  const result = await db
    .select({
      id: permissionsTable.id,
    })
    .from(userRolesTable)
    .innerJoin(
      rolePermissionsTable,
      eq(userRolesTable.roleId, rolePermissionsTable.roleId)
    )
    .innerJoin(
      permissionsTable,
      eq(rolePermissionsTable.permissionId, permissionsTable.id)
    )
    .where(
      and(
        eq(userRolesTable.userId, userId),
        eq(permissionsTable.code, permissionCode)
      )
    )
    .limit(1);

  return result.length > 0;
}
