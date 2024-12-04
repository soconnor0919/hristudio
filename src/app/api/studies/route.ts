import { eq, or } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { studyTable, usersTable, userRolesTable } from "~/db/schema";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  // Get all studies where user is either the owner or has a role
  const studies = await db
    .select({
      id: studyTable.id,
      title: studyTable.title,
      description: studyTable.description,
      createdAt: studyTable.createdAt,
      updatedAt: studyTable.updatedAt,
      userId: studyTable.userId,
    })
    .from(studyTable)
    .leftJoin(userRolesTable, eq(userRolesTable.studyId, studyTable.id))
    .where(
      or(
        eq(studyTable.userId, userId),
        eq(userRolesTable.userId, userId)
      )
    )
    .groupBy(studyTable.id);

  return NextResponse.json(studies);
}

export async function POST(request: Request) {
  const { userId } = await auth();
  
  if (!userId) {
    return new Response(JSON.stringify({ error: "Unauthorized" }), { 
      status: 401,
      headers: { 'Content-Type': 'application/json' }
    });
  }

  try {
    // Verify user exists first
    const existingUser = await db
      .select()
      .from(usersTable)
      .where(eq(usersTable.id, userId));

    const { title, description } = await request.json();

    const study = await db
      .insert(studyTable)
      .values({
        title,
        description,
        userId: userId,
      })
      .returning();

    return new Response(JSON.stringify(study[0]), {
      status: 200,
      headers: { 'Content-Type': 'application/json' }
    });
  } catch (error) {
    return new Response(JSON.stringify({ 
      error: "Failed to create study",
      details: error instanceof Error ? error.message : 'Unknown error'
    }), {
      status: 500,
      headers: { 'Content-Type': 'application/json' }
    });
  }
} 
