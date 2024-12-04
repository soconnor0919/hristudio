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
  EDIT_STUDY: "edit_study",
  VIEW_STUDY: "view_study",
  EDIT_PARTICIPANT: "edit_participant",
  CONTROL_ROBOT: "control_robot",
  VIEW_ROBOT_STATUS: "view_robot_status",
  RECORD_EXPERIMENT: "record_experiment",
  VIEW_EXPERIMENT: "view_experiment",
  VIEW_EXPERIMENT_DATA: "view_experiment_data",
  EXPORT_EXPERIMENT_DATA: "export_experiment_data",
  ANNOTATE_EXPERIMENT: "annotate_experiment",
  MANAGE_USERS: "manage_users",
  MANAGE_SYSTEM_SETTINGS: "manage_system_settings",
} as const;

export type PermissionCode = keyof typeof PERMISSIONS;

export async function getUserPermissions(userId: string, studyId?: number) {
  // Build the base query conditions
  const conditions = [eq(userRolesTable.userId, userId)];
  
  // If studyId is provided, add it to conditions
  if (studyId) {
    conditions.push(eq(userRolesTable.studyId, studyId));
  }

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
    .where(and(...conditions));

  return userPermissions.map(p => p.permissionCode);
}

export async function hasPermission(userId: string, permissionCode: string, studyId?: number) {
  console.log("Checking permission:", {
    userId,
    permissionCode,
    studyId,
    permissionConstant: PERMISSIONS.MANAGE_ROLES
  });

  const result = await db
    .select({
      id: permissionsTable.id,
      code: permissionsTable.code,
      roleId: userRolesTable.roleId,
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
        eq(permissionsTable.code, permissionCode),
        studyId ? eq(userRolesTable.studyId, studyId) : undefined
      )
    );

  console.log("Permission check details:", {
    query: "Executed",
    foundPermissions: result.map(r => ({ roleId: r.roleId, code: r.code }))
  });
  
  return result.length > 0;
}

// Helper function to check if user has any role in a study
export async function hasStudyAccess(userId: string, studyId: number) {
  const result = await db
    .select()
    .from(userRolesTable)
    .where(
      and(
        eq(userRolesTable.userId, userId),
        eq(userRolesTable.studyId, studyId)
      )
    )
    .limit(1);

  return result.length > 0;
}

// Helper function to get all studies a user has access to
export async function getUserStudies(userId: string) {
  return db
    .selectDistinct({ studyId: userRolesTable.studyId })
    .from(userRolesTable)
    .where(eq(userRolesTable.userId, userId));
}

// Helper function to get all roles a user has in a study
export async function getUserStudyRoles(userId: string, studyId: number) {
  return db
    .select({
      roleId: userRolesTable.roleId,
      createdAt: userRolesTable.createdAt,
    })
    .from(userRolesTable)
    .where(
      and(
        eq(userRolesTable.userId, userId),
        eq(userRolesTable.studyId, studyId)
      )
    );
}
