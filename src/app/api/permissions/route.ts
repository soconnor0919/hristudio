import { NextResponse } from "next/server";
import { auth } from "@clerk/nextjs/server";
import { getUserPermissions } from "~/lib/permissions";

export async function GET() {
  const { userId } = await auth();
  
  if (!userId) {
    return new NextResponse("Unauthorized", { status: 401 });
  }

  try {
    const permissions = await getUserPermissions(userId);
    return NextResponse.json(permissions);
  } catch (error) {
    console.error("Error fetching permissions:", error);
    return new NextResponse("Internal Server Error", { status: 500 });
  }
}
