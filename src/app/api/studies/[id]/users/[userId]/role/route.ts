import { eq, and } from "drizzle-orm";
import { db } from "~/db";
import { userRolesTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";

export async function PUT(
  request: Request,
  context: { params: { id: string; userId: string } }
) {
  try {
    const { id, userId } = await context.params;
    const studyId = parseInt(id);

    if (isNaN(studyId)) {
      return ApiError.BadRequest("Invalid study ID");
    }

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.MANAGE_ROLES
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    const { roleId } = await request.json();

    if (!roleId || typeof roleId !== "number") {
      return ApiError.BadRequest("Role ID is required");
    }

    // Update user's role in the study
    await db.transaction(async (tx) => {
      // Delete existing roles
      await tx
        .delete(userRolesTable)
        .where(
          and(
            eq(userRolesTable.userId, userId),
            eq(userRolesTable.studyId, studyId)
          )
        );

      // Assign new role
      await tx
        .insert(userRolesTable)
        .values({
          userId,
          roleId,
          studyId,
        });
    });

    return createApiResponse({ message: "Role updated successfully" });
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 