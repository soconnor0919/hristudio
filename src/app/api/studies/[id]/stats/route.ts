import { eq } from "drizzle-orm";
import { sql } from "drizzle-orm";
import { db } from "~/db";
import { participantsTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";
import { auth } from "@clerk/nextjs/server";

export async function GET(
  request: Request,
  context: { params: { id: string } }
) {
  const { userId } = await auth();
  const { id } = context.params;
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const studyId = parseInt(id);

    if (isNaN(studyId)) {
      return ApiError.BadRequest("Invalid study ID");
    }

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.VIEW_STUDY,
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    // Get participant count using SQL count
    const [{ count }] = await db
      .select({
        count: sql<number>`count(*)::int`,
      })
      .from(participantsTable)
      .where(eq(participantsTable.studyId, studyId));

    // TODO: Add actual trial and form counts when those tables are added
    const stats = {
      participantCount: count,
      completedTrialsCount: 0,
      pendingFormsCount: 0,
    };

    return createApiResponse(stats);
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 