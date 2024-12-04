import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable, studyTable, rolesTable } from "~/db/schema";
import { eq } from "drizzle-orm";
import { randomBytes } from "crypto";
import { sendInvitationEmail } from "~/lib/email";
import { PERMISSIONS, checkPermissions } from "~/lib/permissions-server";
import { ApiError, createApiResponse } from "~/lib/api-utils";

// Helper to generate a secure random token
function generateToken(): string {
  return randomBytes(32).toString('hex');
}

export async function GET(request: Request) {
  try {
    const url = new URL(request.url);
    const studyId = url.searchParams.get("studyId");

    if (!studyId) {
      return ApiError.BadRequest("Study ID is required");
    }

    const permissionCheck = await checkPermissions({
      studyId: parseInt(studyId),
      permission: PERMISSIONS.MANAGE_ROLES
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    // Get all invitations for the study, including role names
    const invitations = await db
      .select({
        id: invitationsTable.id,
        email: invitationsTable.email,
        accepted: invitationsTable.accepted,
        expiresAt: invitationsTable.expiresAt,
        createdAt: invitationsTable.createdAt,
        roleName: rolesTable.name,
      })
      .from(invitationsTable)
      .innerJoin(rolesTable, eq(invitationsTable.roleId, rolesTable.id))
      .where(eq(invitationsTable.studyId, parseInt(studyId)));

    return createApiResponse(invitations);
  } catch (error) {
    return ApiError.ServerError(error);
  }
}

export async function POST(request: Request) {
  try {
    const { email, studyId, roleId } = await request.json();

    const permissionCheck = await checkPermissions({
      studyId,
      permission: PERMISSIONS.MANAGE_ROLES
    });

    if (permissionCheck.error) {
      return permissionCheck.error;
    }

    const { userId } = permissionCheck;

    // Get study details
    const study = await db
      .select()
      .from(studyTable)
      .where(eq(studyTable.id, studyId))
      .limit(1);

    if (!study[0]) {
      return ApiError.NotFound("Study");
    }

    // Verify the role exists
    const role = await db
      .select()
      .from(rolesTable)
      .where(eq(rolesTable.id, roleId))
      .limit(1);

    if (!role[0]) {
      return ApiError.BadRequest("Invalid role");
    }

    // Generate invitation token
    const token = generateToken();
    const expiresAt = new Date();
    expiresAt.setDate(expiresAt.getDate() + 7); // 7 days expiration

    // Create invitation
    const [invitation] = await db
      .insert(invitationsTable)
      .values({
        email,
        studyId,
        roleId,
        token,
        invitedById: userId,
        expiresAt,
      })
      .returning();

    // Send invitation email
    await sendInvitationEmail({
      to: email,
      inviterName: "A researcher", // TODO: Get inviter name
      studyTitle: study[0].title,
      role: role[0].name,
      token,
    });

    return createApiResponse(invitation);
  } catch (error) {
    return ApiError.ServerError(error);
  }
} 