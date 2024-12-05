import { eq } from "drizzle-orm";
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
  const { id } = await Promise.resolve(context.params);
  
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
      permission: PERMISSIONS.VIEW_PARTICIPANT_NAMES,
    });

    const participants = await db
      .select()
      .from(participantsTable)
      .where(eq(participantsTable.studyId, studyId));

    if (permissionCheck.error) {
      const anonymizedParticipants = participants.map((participant, index) => ({
        ...participant,
        name: `Participant ${String.fromCharCode(65 + index)}`,
      }));
      return createApiResponse(anonymizedParticipants);
    }

    return createApiResponse(participants);
  } catch (error) {
    return ApiError.ServerError(error);
  }
}

export async function POST(
  request: Request,
  context: { params: { id: string } }
) {
  const { userId } = await auth();
  const { id } = await Promise.resolve(context.params);
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const studyId = parseInt(id);
    const { name } = await request.json();

    if (isNaN(studyId)) {
      return ApiError.BadRequest("Invalid study ID");
    }

    if (!name || typeof name !== "string") {
      return ApiError.BadRequest("Name is required");
    }

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.CREATE_PARTICIPANT,
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    const participant = await db
      .insert(participantsTable)
      .values({
        name,
        studyId,
      })
      .returning();

    return createApiResponse(participant[0]);
  } catch (error) {
    return ApiError.ServerError(error);
  }
}

export async function DELETE(
  request: Request,
  context: { params: { id: string } }
) {
  const { userId } = await auth();
  const { id } = await Promise.resolve(context.params);
  
  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    const studyId = parseInt(id);
    const { participantId } = await request.json();

    if (isNaN(studyId)) {
      return ApiError.BadRequest("Invalid study ID");
    }

    if (!participantId || typeof participantId !== "number") {
      return ApiError.BadRequest("Participant ID is required");
    }

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.DELETE_PARTICIPANT,
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    await db
      .delete(participantsTable)
      .where(eq(participantsTable.id, participantId));

    return createApiResponse({ success: true });
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 