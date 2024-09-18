import { db } from "~/server/db";
import { studies } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq } from "drizzle-orm";

export async function GET() {
  const allStudies = await db.select().from(studies);
  return NextResponse.json(allStudies);
}

export async function POST(request: Request) {
  const { title, description } = await request.json();
  const newStudy = await db.insert(studies).values({ title, description }).returning();
  return NextResponse.json(newStudy[0]);
}

export async function PUT(request: Request) {
  const { id, title, description } = await request.json();
  const updatedStudy = await db
    .update(studies)
    .set({ title, description })
    .where(eq(studies.id, id))
    .returning();
  return NextResponse.json(updatedStudy[0]);
}

export async function DELETE(request: Request) {
  const { id } = await request.json();
  await db.delete(studies).where(eq(studies.id, id));
  return NextResponse.json({ message: "Study deleted" });
}