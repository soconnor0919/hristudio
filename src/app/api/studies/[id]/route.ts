import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { studyTable } from "~/db/schema";

export async function DELETE(request: Request, { params }: { params: { id: string } }) {
    const { userId } = await auth();
  
    if (!userId) {
      return new NextResponse("Unauthorized", { status: 401 });
    }

    const { id } = await params;
    const studyId = parseInt(id);
  
    try {
        const result = await db
            .delete(studyTable)
            .where(eq(studyTable.id, studyId))
            .execute();
  
        if (result.rowCount === 0) {
            return new NextResponse("Not Found", { status: 404 });
        }
  
        return new NextResponse(null, { status: 204 });
    } catch (error) {
        console.error("Error deleting study:", error);
        return new NextResponse("Internal Server Error", { status: 500 });
    }
  }