import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable, studyTable, rolesTable } from "~/db/schema";
import { eq, and } from "drizzle-orm";
import { randomBytes } from "crypto";
import { sendInvitationEmail } from "~/lib/email";
import { hasPermission, hasStudyAccess, PERMISSIONS } from "~/lib/permissions";

// Helper to generate a secure random token
function generateToken(): string {
  return randomBytes(32).toString('hex');
}

export async function GET(request: Request) {
  const { userId } = await auth();
  
  if (!userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  try {
    const url = new URL(request.url);
    const studyId = url.searchParams.get("studyId");

    if (!studyId) {
      return NextResponse.json({ error: "Study ID is required" }, { status: 400 });
    }

    // First check if user has access to the study
    const hasAccess = await hasStudyAccess(userId, parseInt(studyId));
    if (!hasAccess) {
      return NextResponse.json({ error: "Study not found" }, { status: 404 });
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

    return NextResponse.json(invitations);
  } catch (error) {
    console.error("Error fetching invitations:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
}

export async function POST(request: Request) {
  const { userId } = await auth();
  
  if (!userId) {
    return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
  }

  try {
    const { email, studyId, roleId } = await request.json();

    // First check if user has access to the study
    const hasAccess = await hasStudyAccess(userId, studyId);
    if (!hasAccess) {
      return NextResponse.json({ error: "Study not found" }, { status: 404 });
    }

    // Then check if user has permission to invite users
    const canInvite = await hasPermission(userId, PERMISSIONS.MANAGE_ROLES, studyId);
    if (!canInvite) {
      return NextResponse.json({ error: "Forbidden" }, { status: 403 });
    }

    // Get study details
    const study = await db
      .select()
      .from(studyTable)
      .where(eq(studyTable.id, studyId))
      .limit(1);

    if (!study[0]) {
      return NextResponse.json({ error: "Study not found" }, { status: 404 });
    }

    // Verify the role exists
    const role = await db
      .select()
      .from(rolesTable)
      .where(eq(rolesTable.id, roleId))
      .limit(1);

    if (!role[0]) {
      return NextResponse.json({ error: "Invalid role" }, { status: 400 });
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

    return NextResponse.json(invitation);
  } catch (error) {
    console.error("Error creating invitation:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
} 