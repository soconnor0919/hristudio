// @ts-nocheck
/* eslint-disable */
/* tslint:disable */

import { eq } from "drizzle-orm";
import { NextRequest } from "next/server";
import { db } from "~/db";
import { invitationsTable } from "~/db/schema";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";

export async function DELETE(req: NextRequest, { params }: { params: { id: string } }) {
  try {
    const { id } = params;
    const invitationId = parseInt(id, 10);

    if (isNaN(invitationId)) {
      return ApiError.BadRequest("Invalid invitation ID");
    }

    // Get the invitation to check the study ID
    const invitation = await db
      .select()
      .from(invitationsTable)
      .where(eq(invitationsTable.id, invitationId))
      .limit(1);

    if (!invitation[0]) {
      return ApiError.NotFound("Invitation");
    }

    const permissionCheck = await checkPermissions({
      studyId: invitation[0].studyId,
      permission: PERMISSIONS.MANAGE_ROLES
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    await db
      .delete(invitationsTable)
      .where(eq(invitationsTable.id, invitationId));

    return createApiResponse({ message: "Invitation deleted successfully" });
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 