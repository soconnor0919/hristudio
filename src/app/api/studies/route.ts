import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { studyTable } from "~/db/schema";

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
    return new NextResponse("Unauthorized", { status: 401 });
  }

  const { title, description } = await request.json();

  const study = await db
    .insert(studyTable)
    .values({
      title,
      description,
      userId,
    })
    .returning();

  return NextResponse.json(study[0]);
} 
