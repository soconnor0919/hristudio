import { db } from "~/server/db";
import { participants, trialParticipants, trials } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq, sql } from "drizzle-orm";
import { auth } from "@clerk/nextjs/server"; // Import auth to get userId
import { anonymizeParticipants } from "~/lib/permissions"; // Import the anonymize function

export async function GET(request: Request) {
  const { userId } = auth(); // Get the userId from auth
  try {
    const { searchParams } = new URL(request.url);
    const studyId = searchParams.get('studyId');
    
    if (!studyId) {
      return NextResponse.json({ error: 'Study ID is required' }, { status: 400 });
    }

    const participantsWithLatestTrial = await db
      .select({
        id: participants.id,
        name: participants.name,
        studyId: participants.studyId,
        createdAt: participants.createdAt,
        latestTrialTimestamp: sql<Date | null>`MAX(${trials.createdAt})`.as('latestTrialTimestamp')
      })
      .from(participants)
      .leftJoin(trialParticipants, eq(participants.id, trialParticipants.participantId))
      .leftJoin(trials, eq(trialParticipants.trialId, trials.id))
      .where(eq(participants.studyId, parseInt(studyId)))
      .groupBy(participants.id)
      .orderBy(sql`COALESCE(MAX(${trials.createdAt}), ${participants.createdAt}) DESC`);

    // Anonymize participant names
    const anonymizedParticipants = anonymizeParticipants(participantsWithLatestTrial, userId);

    return NextResponse.json(anonymizedParticipants);
  } catch (error) {
    console.error('Error in GET /api/participants:', error);
    return NextResponse.json({ error: 'Internal Server Error' }, { status: 500 });
  }
}

export async function POST(request: Request) {
  const { name, studyId } = await request.json();
  if (!name || !studyId) {
    return NextResponse.json({ error: 'Name and Study ID are required' }, { status: 400 });
  }
  const newParticipant = await db.insert(participants).values({ name, studyId }).returning();
  return NextResponse.json(newParticipant[0]);
}