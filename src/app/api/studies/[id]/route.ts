import { db } from "~/server/db";
import { studies } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq, and } from "drizzle-orm";
import { auth } from "@clerk/nextjs/server";

export async function GET(
  request: Request,
  { params }: { params: { id: string } }
) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const id = parseInt(params.id);
  if (isNaN(id)) {
    return NextResponse.json({ error: 'Invalid ID' }, { status: 400 });
  }

  const study = await db.select().from(studies).where(and(eq(studies.id, id), eq(studies.userId, userId))).limit(1);

  if (study.length === 0) {
    return NextResponse.json({ error: 'Study not found' }, { status: 404 });
  }

  return NextResponse.json(study[0]);
}

export async function PUT(
  request: Request,
  { params }: { params: { id: string } }
) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const id = parseInt(params.id);
  if (isNaN(id)) {
    return NextResponse.json({ error: 'Invalid ID' }, { status: 400 });
  }

  const { title, description } = await request.json();
  if (!title) {
    return NextResponse.json({ error: 'Title is required' }, { status: 400 });
  }

  const updatedStudy = await db
    .update(studies)
    .set({ title, description })
    .where(and(eq(studies.id, id), eq(studies.userId, userId)))
    .returning();

  if (updatedStudy.length === 0) {
    return NextResponse.json({ error: 'Study not found or unauthorized' }, { status: 404 });
  }

  return NextResponse.json(updatedStudy[0]);
}

export async function DELETE(
  request: Request,
  { params }: { params: { id: string } }
) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const id = parseInt(params.id);
  if (isNaN(id)) {
    return NextResponse.json({ error: 'Invalid ID' }, { status: 400 });
  }

  const deletedStudy = await db
    .delete(studies)
    .where(and(eq(studies.id, id), eq(studies.userId, userId)))
    .returning();

  if (deletedStudy.length === 0) {
    return NextResponse.json({ error: 'Study not found or unauthorized' }, { status: 404 });
  }

  return NextResponse.json({ message: "Study deleted successfully" });
}
