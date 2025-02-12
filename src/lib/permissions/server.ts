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
import { getServerSession } from "next-auth";
import { studyMembers } from "~/server/db/schema";
import { ROLE_PERMISSIONS, ROLES, PERMISSIONS } from "./constants";
import type { Session } from "next-auth";
import { studies } from "~/server/db/schema";

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

interface CheckPermissionsOptions {
  studyId?: number;
  permission: Permission;
  session: Session | null;
}

export async function checkPermissions({
  studyId,
  permission,
  session,
}: CheckPermissionsOptions): Promise<void> {
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

  // Normalize role (convert membership.role to uppercase) so that it matches the keys in ROLE_PERMISSIONS
  const normalizedRole = membership.role.toUpperCase() as keyof typeof ROLE_PERMISSIONS;
  const permittedActions = ROLE_PERMISSIONS[normalizedRole] ?? [];

  // For owners, they have all permissions
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