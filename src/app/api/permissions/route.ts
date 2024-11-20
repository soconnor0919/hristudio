import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { getUserPermissions } from "~/lib/permissions";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  const permissions = await getUserPermissions(userId);
  return NextResponse.json(Array.from(permissions));
}
