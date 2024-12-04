import { eq } from "drizzle-orm";
import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { studyTable } from "~/db/schema";
import { hasStudyAccess } from "~/lib/permissions";

export async function GET(
  request: Request,
  context: { params: { id: string } }
) {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  try {
    // Properly await and destructure params
    const { id } = await context.params;
    const studyId = parseInt(id);
    
    // Check if user has access to this study
    const hasAccess = await hasStudyAccess(userId, studyId);
    if (!hasAccess) {
      return new NextResponse("Forbidden", { status: 403 });
    }

    // Get study details
    const study = await db
      .select()
      .from(studyTable)
      .where(eq(studyTable.id, studyId))
      .limit(1);

    if (!study[0]) {
      return new NextResponse("Study not found", { status: 404 });
    }

    return NextResponse.json(study[0]);
  } catch (error) {
    console.error("Error fetching study:", error);
    return new NextResponse("Internal Server Error", { status: 500 });
  }
} 