import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { db } from "~/db";
import { rolesTable } from "~/db/schema";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  try {
    const roles = await db
      .select({
        id: rolesTable.id,
        name: rolesTable.name,
        description: rolesTable.description,
      })
      .from(rolesTable);

    return NextResponse.json(roles);
  } catch (error) {
    console.error("Error fetching roles:", error);
    return new NextResponse("Internal Server Error", { status: 500 });
  }
} 