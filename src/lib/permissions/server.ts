import { and, eq } from "drizzle-orm";
import { db } from "~/server/db";
import {
  permissions,
  rolePermissions,
  roles,
  userRoles,
} from "~/server/db/schema/permissions";
import { type Permission, type PermissionValue } from "./constants";
import { auth } from "~/server/auth";
import { TRPCError } from "@trpc/server";

export async function getUserPermissions(userId: string, studyId?: number) {
  const conditions = [eq(userRoles.userId, userId)];
  
  if (studyId) {
    conditions.push(eq(userRoles.studyId, studyId));
  }
  
  const userPermissions = await db
    .select({
      permissionCode: permissions.code,
    })
    .from(userRoles)
    .innerJoin(
      rolePermissions,
      eq(userRoles.roleId, rolePermissions.roleId)
    )
    .innerJoin(
      permissions,
      eq(rolePermissions.permissionId, permissions.id)
    )
    .where(and(...conditions));
  
  return userPermissions.map((p) => p.permissionCode);
}

export async function hasPermission(
  userId: string,
  permissionCode: PermissionValue,
  studyId?: number
) {
  const conditions = [
    eq(userRoles.userId, userId),
    eq(permissions.code, permissionCode),
  ];
  
  if (studyId) {
    conditions.push(eq(userRoles.studyId, studyId));
  }
  
  const result = await db
    .select({
      id: permissions.id,
    })
    .from(userRoles)
    .innerJoin(
      rolePermissions,
      eq(userRoles.roleId, rolePermissions.roleId)
    )
    .innerJoin(
      permissions,
      eq(rolePermissions.permissionId, permissions.id)
    )
    .where(and(...conditions))
    .limit(1);
  
  return result.length > 0;
}

export async function hasStudyAccess(userId: string, studyId: number) {
  const result = await db
    .select()
    .from(userRoles)
    .where(
      and(
        eq(userRoles.userId, userId),
        eq(userRoles.studyId, studyId)
      )
    )
    .limit(1);
  
  return result.length > 0;
}

export async function getUserStudies(userId: string) {
  return db
    .selectDistinct({
      studyId: userRoles.studyId,
    })
    .from(userRoles)
    .where(eq(userRoles.userId, userId));
}

export async function getUserStudyRoles(userId: string, studyId: number) {
  return db
    .select({
      roleId: userRoles.roleId,
      roleName: roles.name,
      roleCode: roles.code,
      createdAt: userRoles.createdAt,
    })
    .from(userRoles)
    .innerJoin(roles, eq(userRoles.roleId, roles.id))
    .where(
      and(
        eq(userRoles.userId, userId),
        eq(userRoles.studyId, studyId)
      )
    );
}

interface PermissionCheck {
  studyId: number;
  permission?: PermissionValue;
  requireStudyAccess?: boolean;
}

export async function checkPermissions(check: PermissionCheck) {
  const session = await auth();
  if (!session?.user?.id) {
    throw new TRPCError({
      code: "UNAUTHORIZED",
      message: "You must be logged in to perform this action",
    });
  }

  const { studyId, permission, requireStudyAccess = true } = check;

  if (requireStudyAccess) {
    const hasAccess = await hasStudyAccess(session.user.id, studyId);
    if (!hasAccess) {
      throw new TRPCError({
        code: "NOT_FOUND",
        message: "Study not found",
      });
    }
  }

  if (permission) {
    const hasRequiredPermission = await hasPermission(session.user.id, permission, studyId);
    if (!hasRequiredPermission) {
      throw new TRPCError({
        code: "FORBIDDEN",
        message: "You don't have permission to perform this action",
      });
    }
  }

  return { userId: session.user.id };
} 