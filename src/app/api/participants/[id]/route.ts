import { db } from "~/server/db";
import { participants } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq } from "drizzle-orm";

export async function DELETE(
  request: Request,
  { params }: { params: { id: string } }
) {
  console.log('DELETE route hit, params:', params);
  const id = parseInt(params.id);
  
  if (isNaN(id)) {
    console.log('Invalid ID:', id);
    return NextResponse.json({ error: 'Invalid ID' }, { status: 400 });
  }

  try {
    console.log('Attempting to delete participant with ID:', id);
    const deletedParticipant = await db.delete(participants)
      .where(eq(participants.id, id))
      .returning();

    console.log('Deleted participant:', deletedParticipant);

    if (deletedParticipant.length === 0) {
      console.log('Participant not found');
      return NextResponse.json({ error: 'Participant not found' }, { status: 404 });
    }

    console.log('Participant deleted successfully');
    return NextResponse.json({ message: "Participant deleted successfully" });
  } catch (error) {
    console.error('Error deleting participant:', error);
    return NextResponse.json({ error: 'Failed to delete participant', details: String(error) }, { status: 500 });
  }
}