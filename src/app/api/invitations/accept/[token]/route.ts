import { eq, and, gt } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable, userRolesTable } from "~/db/schema";

export async function POST(
  request: Request,
  { params }: { params: { token: string } }
) {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  try {
    const { token } = params;

    // Find the invitation
    const [invitation] = await db
      .select()
      .from(invitationsTable)
      .where(
        and(
          eq(invitationsTable.token, token),
          eq(invitationsTable.accepted, false),
          gt(invitationsTable.expiresAt, new Date())
        )
      )
      .limit(1);

    if (!invitation) {
      return new NextResponse(
        "Invitation not found or has expired",
        { status: 404 }
      );
    }

    // Start a transaction
    await db.transaction(async (tx) => {
      // Mark invitation as accepted
      await tx
        .update(invitationsTable)
        .set({ accepted: true })
        .where(eq(invitationsTable.id, invitation.id));

      // Assign role to user for this specific study
      await tx
        .insert(userRolesTable)
        .values({
          userId,
          roleId: invitation.roleId,
          studyId: invitation.studyId,
        })
        .onConflictDoNothing();
    });

    return new NextResponse("Invitation accepted", { status: 200 });
  } catch (error) {
    console.error("Error accepting invitation:", error);
    return new NextResponse("Internal Server Error", { status: 500 });
  }
} 