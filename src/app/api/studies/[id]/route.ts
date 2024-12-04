import { eq, and } from "drizzle-orm";
import { db } from "~/db";
import { studyTable, userRolesTable, rolePermissionsTable, permissionsTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";

export async function GET(request: Request, { params }: { params: { id: string } }) {
  try {
    const { id } = params;
    const studyId = parseInt(id);

    if (isNaN(studyId)) {
      return ApiError.BadRequest("Invalid study ID");
    }

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.VIEW_STUDY
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    // Get study with permissions
    const studyWithPermissions = await db
      .selectDistinct({
        id: studyTable.id,
        title: studyTable.title,
        description: studyTable.description,
        createdAt: studyTable.createdAt,
        updatedAt: studyTable.updatedAt,
        userId: studyTable.userId,
        permissionCode: permissionsTable.code,
      })
      .from(studyTable)
      .leftJoin(userRolesTable, eq(userRolesTable.studyId, studyTable.id))
      .leftJoin(rolePermissionsTable, eq(rolePermissionsTable.roleId, userRolesTable.roleId))
      .leftJoin(permissionsTable, eq(permissionsTable.id, rolePermissionsTable.permissionId))
      .where(eq(studyTable.id, studyId));

    if (!studyWithPermissions.length) {
      return ApiError.NotFound("Study");
    }

    // Group permissions
    const study = {
      ...studyWithPermissions[0],
      permissions: studyWithPermissions
        .map(s => s.permissionCode)
        .filter((code): code is string => code !== null)
    };

    return createApiResponse(study);
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 