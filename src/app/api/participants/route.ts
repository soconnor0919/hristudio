import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { participants } from "~/db/schema";

export async function GET(request: Request) {
    const { userId } = await auth();

    if (!userId) {
        return new NextResponse("Unauthorized", { status: 401 });
    }

    const url = new URL(request.url);
    const studyId = url.searchParams.get("studyId");

    if (!studyId) {
        return new NextResponse("Study ID is required", { status: 400 });
    }

    const participantList = await db
        .select()
        .from(participants)
        .where(eq(participants.studyId, parseInt(studyId)));

    return NextResponse.json(participantList);
}

export async function POST(request: Request) {
    const { userId } = await auth();

    if (!userId) {
        return new NextResponse("Unauthorized", { status: 401 });
    }

    const { name, studyId } = await request.json();

    try {
        const participant = await db
            .insert(participants)
            .values({
                name,
                studyId,
            })
            .returning();

        return NextResponse.json(participant[0]);
    } catch (error) {
        console.error("Error adding participant:", error);
        return new NextResponse("Internal Server Error", { status: 500 });
    }
}