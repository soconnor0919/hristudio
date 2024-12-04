import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { ApiError, createApiResponse } from "~/lib/api-utils";
import { db } from "~/db";
import { userRolesTable, rolePermissionsTable, permissionsTable } from "~/db/schema";
import { eq, and } from "drizzle-orm";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const permissions = await db
      .selectDistinct({
        code: permissionsTable.code,
      })
      .from(userRolesTable)
      .innerJoin(rolePermissionsTable, eq(rolePermissionsTable.roleId, userRolesTable.roleId))
      .innerJoin(permissionsTable, eq(permissionsTable.id, rolePermissionsTable.permissionId))
      .where(eq(userRolesTable.userId, userId));

    return createApiResponse(permissions.map(p => p.code));
  } catch (error) {
    return ApiError.ServerError(error);
  }
}
