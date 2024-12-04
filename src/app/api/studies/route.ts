import { eq, and, or } from "drizzle-orm";
import { db } from "~/db";
import { studyTable, userRolesTable, rolePermissionsTable, permissionsTable, rolesTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse, getEnvironment } from "~/lib/api-utils";
import { auth } from "@clerk/nextjs/server";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const currentEnvironment = getEnvironment();

    // Get all studies where user has any role
    const studiesWithPermissions = await db
      .selectDistinct({
        id: studyTable.id,
        title: studyTable.title,
        description: studyTable.description,
        createdAt: studyTable.createdAt,
        updatedAt: studyTable.updatedAt,
        userId: studyTable.userId,
        permissionCode: permissionsTable.code,
        roleName: rolesTable.name,
      })
      .from(studyTable)
      .innerJoin(
        userRolesTable,
        and(
          eq(userRolesTable.studyId, studyTable.id),
          eq(userRolesTable.userId, userId)
        )
      )
      .innerJoin(rolesTable, eq(rolesTable.id, userRolesTable.roleId))
      .leftJoin(rolePermissionsTable, eq(rolePermissionsTable.roleId, userRolesTable.roleId))
      .leftJoin(permissionsTable, eq(permissionsTable.id, rolePermissionsTable.permissionId))
      .where(eq(studyTable.environment, currentEnvironment));

    // Group permissions and roles by study
    const studies = studiesWithPermissions.reduce((acc, curr) => {
      const existingStudy = acc.find(s => s.id === curr.id);
      if (!existingStudy) {
        acc.push({
          id: curr.id,
          title: curr.title,
          description: curr.description,
          createdAt: curr.createdAt,
          updatedAt: curr.updatedAt,
          userId: curr.userId,
          permissions: curr.permissionCode ? [curr.permissionCode] : [],
          roles: curr.roleName ? [curr.roleName] : []
        });
      } else {
        if (curr.permissionCode && !existingStudy.permissions.includes(curr.permissionCode)) {
          existingStudy.permissions.push(curr.permissionCode);
        }
        if (curr.roleName && !existingStudy.roles.includes(curr.roleName)) {
          existingStudy.roles.push(curr.roleName);
        }
      }
      return acc;
    }, [] as Array<{
      id: number;
      title: string;
      description: string | null;
      createdAt: Date;
      updatedAt: Date | null;
      userId: string;
      permissions: string[];
      roles: string[];
    }>);

    return createApiResponse(studies);
  } catch (error) {
    return ApiError.ServerError(error);
  }
}

export async function POST(request: Request) {
  const { userId } = await auth();
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const { title, description } = await request.json();
    const currentEnvironment = getEnvironment();

    // Create study and assign admin role in a transaction
    const result = await db.transaction(async (tx) => {
      // Create the study
      const [study] = await tx
        .insert(studyTable)
        .values({
          title,
          description,
          userId: userId,
          environment: currentEnvironment,
        })
        .returning();

      // Look up the ADMIN role
      const [adminRole] = await tx
        .select()
        .from(rolesTable)
        .where(eq(rolesTable.name, 'admin'))
        .limit(1);

      if (!adminRole) {
        throw new Error('Admin role not found');
      }

      // Assign admin role
      await tx
        .insert(userRolesTable)
        .values({
          userId: userId,
          roleId: adminRole.id,
          studyId: study.id,
        });

      // Get all permissions for this role
      const permissions = await tx
        .select({
          permissionCode: permissionsTable.code
        })
        .from(rolePermissionsTable)
        .innerJoin(permissionsTable, eq(permissionsTable.id, rolePermissionsTable.permissionId))
        .where(eq(rolePermissionsTable.roleId, adminRole.id));

      return {
        ...study,
        permissions: permissions.map(p => p.permissionCode)
      };
    });

    return createApiResponse(result);
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 
