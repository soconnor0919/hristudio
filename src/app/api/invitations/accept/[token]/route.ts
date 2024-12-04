import { eq, and } from "drizzle-orm";
import { NextRequest } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable, userRolesTable } from "~/db/schema";
import { ApiError, createApiResponse } from "~/lib/api-utils";

export async function POST(req: NextRequest, { params }: { params: { token: string } }) {
  const { userId } = await auth();
  const { token } = params;

  if (!userId) {
    return ApiError.Unauthorized();
  }

  try {
    // Find the invitation
    const [invitation] = await db
      .select()
      .from(invitationsTable)
      .where(
        and(
          eq(invitationsTable.token, token),
          eq(invitationsTable.accepted, false)
        )
      )
      .limit(1);

    if (!invitation) {
      return ApiError.NotFound("Invitation");
    }

    // Check if invitation has expired
    if (new Date() > invitation.expiresAt) {
      return ApiError.BadRequest("Invitation has expired");
    }

    // Assign role and mark invitation as accepted in a transaction
    await db.transaction(async (tx) => {
      // Assign role
      await tx
        .insert(userRolesTable)
        .values({
          userId: userId,
          roleId: invitation.roleId,
          studyId: invitation.studyId,
        });

      // Mark invitation as accepted
      await tx
        .update(invitationsTable)
        .set({
          accepted: true,
          acceptedByUserId: userId,
        })
        .where(eq(invitationsTable.id, invitation.id));
    });

    return createApiResponse({ message: "Invitation accepted successfully" });
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 