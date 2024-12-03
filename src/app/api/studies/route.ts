import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { studyTable, usersTable } from "~/db/schema";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  const studies = await db
    .select()
    .from(studyTable)
    .where(eq(studyTable.userId, userId));
    // TODO: Open up to multiple users
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
    // Debug log
    console.log("Creating study for user:", userId);

    // Verify user exists first
    const existingUser = await db
      .select()
      .from(usersTable)
      .where(eq(usersTable.id, userId));

    console.log("Found user:", existingUser[0]); // Debug log

    const { title, description } = await request.json();
    
    // Debug log
    console.log("Study data:", { title, description, userId });

    const study = await db
      .insert(studyTable)
      .values({
        title,
        description,
        userId: userId, // Explicitly use the userId from auth
      })
      .returning();

    console.log("Created study:", study[0]); // Debug log

    return new Response(JSON.stringify(study[0]), {
      status: 200,
      headers: { 'Content-Type': 'application/json' }
    });
  } catch (error) {
    // Enhanced error logging
    console.error("Error details:", {
      error,
      userId,
      errorMessage: error instanceof Error ? error.message : 'Unknown error',
      errorName: error instanceof Error ? error.name : 'Unknown error type'
    });

    return new Response(JSON.stringify({ 
      error: "Failed to create study",
      details: error instanceof Error ? error.message : 'Unknown error'
    }), {
      status: 500,
      headers: { 'Content-Type': 'application/json' }
    });
  }
} 
