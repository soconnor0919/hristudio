// @ts-nocheck
/* eslint-disable */
/* tslint:disable */

import { eq } from "drizzle-orm";
import { NextRequest, NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { invitationsTable } from "~/db/schema";

// @ts-ignore
export async function DELETE(req: NextRequest, { params }: { params: { id: string } }) {
  const { userId } = await auth();
  const { id } = params;

  if (!userId) {
    return NextResponse.json(
      { error: "Unauthorized" },
      { status: 401 }
    );
  }

  try {
    const invitationId = parseInt(id, 10);

    if (isNaN(invitationId)) {
      return NextResponse.json(
        { error: "Invalid invitation ID" },
        { status: 400 }
      );
    }

    await db
      .delete(invitationsTable)
      .where(eq(invitationsTable.id, invitationId));

    return NextResponse.json(
      { message: "Invitation deleted successfully" },
      { status: 200 }
    );
  } catch (error) {
    console.error("Error deleting invitation:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
} 