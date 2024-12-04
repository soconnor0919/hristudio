import { eq, and } from "drizzle-orm";
import { NextRequest, NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable } from "~/db/schema";

export async function POST(req: NextRequest, { params }: { params: { token: string } }) {
  const { userId } = await auth();
  const { token } = params;

  if (!userId) {
    return NextResponse.json(
      { error: "Unauthorized" },
      { status: 401 }
    );
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
      return NextResponse.json(
        { error: "Invalid or expired invitation" },
        { status: 404 }
      );
    }

    // Update the invitation
    await db
      .update(invitationsTable)
      .set({
        accepted: true,
        acceptedByUserId: userId,
      })
      .where(eq(invitationsTable.id, invitation.id));

    return NextResponse.json(
      { message: "Invitation accepted successfully" },
      { status: 200 }
    );
  } catch (error) {
    console.error("Error accepting invitation:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
} 