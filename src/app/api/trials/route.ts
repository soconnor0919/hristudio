import { db } from "~/server/db";
import { trials, trialParticipants } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq, sql } from "drizzle-orm";

export async function GET() {
  const allTrials = await db
    .select({
      id: trials.id,
      title: trials.title,
      participantIds: sql`ARRAY_AGG(${trialParticipants.participantId})`.as('participantIds'),
    })
    .from(trials)
    .leftJoin(trialParticipants, eq(trials.id, trialParticipants.trialId))
    .groupBy(trials.id);

  return NextResponse.json(allTrials);
}

export async function POST(request: Request) {
  const { title, participantIds } = await request.json();
  
  if (!title || !Array.isArray(participantIds) || participantIds.some(id => typeof id !== 'number')) {
    return NextResponse.json({ error: 'Title and valid Participant IDs are required' }, { status: 400 });
  }

  // Insert the new trial into the trials table
  const newTrial = await db.insert(trials).values({ title }).returning();
  // Check if newTrial is defined and has at least one element
  if (!newTrial || newTrial.length === 0) {
    throw new Error('Failed to create a new trial');
  }
  // Insert the participant associations into the trial_participants table
  const trialId = newTrial[0]?.id; // Use optional chaining to safely get the ID of the newly created trial
  if (trialId === undefined) {
    throw new Error('Trial ID is undefined');
  }
  const trialParticipantEntries = participantIds.map(participantId => ({
    trialId,
    participantId,
  }));

  await db.insert(trialParticipants).values(trialParticipantEntries);

  return NextResponse.json(newTrial[0]);
}

export async function DELETE(request: Request) {
  const { id } = await request.json();
  await db.delete(trials).where(eq(trials.id, id));
  return NextResponse.json({ message: "Trial deleted successfully" });
}
