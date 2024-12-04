import { eq, and } from "drizzle-orm";
import { db } from "~/db";
import { userRolesTable, usersTable, rolesTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";

export async function GET(
  request: Request,
  context: { params: { id: string } }
) {
  try {
    const { id } = await context.params;
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

    // Get all users in the study with their roles
    const studyUsers = await db
      .select({
        id: usersTable.id,
        email: usersTable.email,
        name: usersTable.name,
        roleId: rolesTable.id,
        roleName: rolesTable.name,
      })
      .from(userRolesTable)
      .innerJoin(usersTable, eq(usersTable.id, userRolesTable.userId))
      .innerJoin(rolesTable, eq(rolesTable.id, userRolesTable.roleId))
      .where(eq(userRolesTable.studyId, studyId));

    // Group roles by user
    const users = studyUsers.reduce((acc, curr) => {
      const existingUser = acc.find(u => u.id === curr.id);
      if (!existingUser) {
        acc.push({
          id: curr.id,
          email: curr.email,
          name: curr.name,
          roles: [{
            id: curr.roleId,
            name: curr.roleName,
          }]
        });
      } else if (curr.roleName && !existingUser.roles.some(r => r.id === curr.roleId)) {
        existingUser.roles.push({
          id: curr.roleId,
          name: curr.roleName,
        });
      }
      return acc;
    }, [] as Array<{
      id: string;
      email: string;
      name: string | null;
      roles: Array<{ id: number; name: string }>;
    }>);

    return createApiResponse(users);
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 