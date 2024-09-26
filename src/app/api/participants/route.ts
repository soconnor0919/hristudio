import { db } from "~/server/db";
import { participants } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq } from "drizzle-orm";

export async function GET(request: Request) {
  const { searchParams } = new URL(request.url);
  const studyId = searchParams.get('studyId');
  
  if (!studyId) {
    return NextResponse.json({ error: 'Study ID is required' }, { status: 400 });
  }

  const allParticipants = await db.select().from(participants).where(eq(participants.studyId, parseInt(studyId)));
  return NextResponse.json(allParticipants);
}

export async function POST(request: Request) {
  const { name, studyId } = await request.json();
  if (!name || !studyId) {
    return NextResponse.json({ error: 'Name and Study ID are required' }, { status: 400 });
  }
  const newParticipant = await db.insert(participants).values({ name, studyId }).returning();
  return NextResponse.json(newParticipant[0]);
}