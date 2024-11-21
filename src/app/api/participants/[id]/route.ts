import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { participants } from "~/db/schema";


export async function DELETE(request: Request, { params }: any) {
    const { userId } = await auth();

    if (!userId) {
        return new NextResponse("Unauthorized", { status: 401 });
    }

    const participantId = parseInt(params.id);

    try {
        const result = await db
            .delete(participants)
            .where(eq(participants.id, participantId))
            .execute();

        if (result.rowCount === 0) {
            return new NextResponse("Not Found", { status: 404 });
        }

        return new NextResponse(null, { status: 204 }); // No content for successful deletion
    } catch (error) {
        console.error("Error deleting participant:", error);
        return new NextResponse("Internal Server Error", { status: 500 });
    }
}