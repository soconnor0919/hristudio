import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable } from "~/db/schema";
import { hasPermission, PERMISSIONS } from "~/lib/permissions";

export async function DELETE(
  request: Request,
  context: { params: { id: string } }
) {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  try {
    // Properly await and destructure params
    const { id } = await context.params;
    const invitationId = parseInt(id);

    // Get the invitation to check study access
    const [invitation] = await db
      .select()
      .from(invitationsTable)
      .where(eq(invitationsTable.id, invitationId))
      .limit(1);

    if (!invitation) {
      return new NextResponse("Invitation not found", { status: 404 });
    }

    // Check if user has permission to manage roles for this study
    const canManageRoles = await hasPermission(userId, PERMISSIONS.MANAGE_ROLES, invitation.studyId);
    if (!canManageRoles) {
      return new NextResponse("Forbidden", { status: 403 });
    }

    // Delete the invitation
    await db
      .delete(invitationsTable)
      .where(eq(invitationsTable.id, invitationId));

    return new NextResponse(null, { status: 204 });
  } catch (error) {
    console.error("Error deleting invitation:", error);
    return new NextResponse("Internal Server Error", { status: 500 });
  }
} 