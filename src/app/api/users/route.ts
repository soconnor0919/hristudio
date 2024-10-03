import { db } from "~/server/db";
import { users } from "~/server/db/schema";
import { NextResponse } from "next/server";
import { eq } from "drizzle-orm";
export async function POST(request: Request) {
  try {
    const { email } = await request.json();
    
    // Check if email is provided
    if (!email) {
      return NextResponse.json({ error: "Email is required" }, { status: 400 });
    }

    // Check if the user already exists
    const existingUser = await db.select().from(users).where(eq(users.email, email)).limit(1);
    if (existingUser) {
      return NextResponse.json({ error: "User already exists" }, { status: 409 });
    }

    // Insert the new user into the database
    const newUser = await db.insert(users).values({ email }).returning();
    return NextResponse.json(newUser[0]);
  } catch (error) {
    console.error("Error creating user:", error);
    return NextResponse.json({ error: "Failed to create user" }, { status: 500 });
  }
}