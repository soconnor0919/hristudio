import { eq, and, or } from "drizzle-orm";
import { db } from "~/db";
import { userRolesTable, rolePermissionsTable, permissionsTable } from "~/db/schema";
import { ApiError } from "./api-utils";
import { auth } from "@clerk/nextjs/server";
import { PERMISSIONS } from "./permissions-client";

export { PERMISSIONS };

export async function hasStudyAccess(userId: string, studyId: number): Promise<boolean> {
  const userRoles = await db
    .select()
    .from(userRolesTable)
    .where(
      and(
        eq(userRolesTable.userId, userId),
        eq(userRolesTable.studyId, studyId)
      )
    );

  return userRoles.length > 0;
}

export async function hasPermission(
  userId: string,
  permissionCode: string,
  studyId: number
): Promise<boolean> {
  const permissions = await db
    .selectDistinct({
      permissionCode: permissionsTable.code,
    })
    .from(userRolesTable)
    .innerJoin(rolePermissionsTable, eq(rolePermissionsTable.roleId, userRolesTable.roleId))
    .innerJoin(permissionsTable, eq(permissionsTable.id, rolePermissionsTable.permissionId))
    .where(
      and(
        eq(userRolesTable.userId, userId),
        eq(userRolesTable.studyId, studyId)
      )
    );

  return permissions.some(p => p.permissionCode === permissionCode);
}

export type PermissionCheck = {
  studyId: number;
  permission?: string;
  requireStudyAccess?: boolean;
};

export async function checkPermissions(check: PermissionCheck) {
  const { userId } = await auth();
  if (!userId) {
    return { error: ApiError.Unauthorized() };
  }

  const { studyId, permission, requireStudyAccess = true } = check;

  if (requireStudyAccess) {
    const hasAccess = await hasStudyAccess(userId, studyId);
    if (!hasAccess) {
      return { error: ApiError.NotFound("Study") };
    }
  }

  if (permission) {
    const hasRequiredPermission = await hasPermission(userId, permission, studyId);
    if (!hasRequiredPermission) {
      return { error: ApiError.Forbidden() };
    }
  }

  return { userId };
} 