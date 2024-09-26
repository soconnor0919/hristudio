import { db } from "~/server/db";
import { studies } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq } from "drizzle-orm";
import { auth } from "@clerk/nextjs/server";

export async function GET(request: Request) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const allStudies = await db.select().from(studies).where(eq(studies.userId, userId));
  return NextResponse.json(allStudies);
}

export async function POST(request: Request) {
  const { userId } = auth();
  if (!userId) {
    return NextResponse.json({ error: 'Unauthorized' }, { status: 401 });
  }

  const { title, description } = await request.json();
  if (!title) {
    return NextResponse.json({ error: 'Title is required' }, { status: 400 });
  }

  const newStudy = await db.insert(studies).values({ title, description, userId }).returning();
  return NextResponse.json(newStudy[0]);
}