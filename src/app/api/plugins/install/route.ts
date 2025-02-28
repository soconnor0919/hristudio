import { z } from "zod";
import { NextResponse } from "next/server";
import { getPlugin } from "~/lib/plugin-store/service";
import { db } from "~/server/db";
import { installedPlugins } from "~/server/db/schema/store";
import { eq } from "drizzle-orm";

// POST /api/plugins/install - Install a plugin
export async function POST(req: Request) {
  try {
    const body = await req.json();
    const schema = z.object({
      robotId: z.string(),
      repositoryId: z.string(),
    });
    
    const { robotId, repositoryId } = schema.parse(body);
    
    // Get plugin details
    const plugin = await getPlugin(robotId);
    if (!plugin) {
      return NextResponse.json(
        { error: "Plugin not found" },
        { status: 404 }
      );
    }
    
    // Check if already installed
    const existing = await db.query.installedPlugins.findFirst({
      where: eq(installedPlugins.robotId, robotId),
    });
    
    if (existing) {
      return NextResponse.json(
        { error: "Plugin already installed" },
        { status: 400 }
      );
    }
    
    // Install plugin
    const installed = await db.insert(installedPlugins).values({
      robotId,
      repositoryId,
      name: plugin.name,
      version: plugin.version,
      enabled: true,
      config: {},
    }).returning();
    
    return NextResponse.json(installed[0]);
  } catch (error) {
    console.error("Failed to install plugin:", error);
    if (error instanceof z.ZodError) {
      return NextResponse.json(
        { error: "Invalid request body", details: error.errors },
        { status: 400 }
      );
    }
    return NextResponse.json(
      { error: "Failed to install plugin" },
      { status: 500 }
    );
  }
}

// DELETE /api/plugins/install - Uninstall a plugin
export async function DELETE(req: Request) {
  try {
    const url = new URL(req.url);
    const robotId = url.searchParams.get("robotId");
    
    if (!robotId) {
      return NextResponse.json(
        { error: "Robot ID is required" },
        { status: 400 }
      );
    }
    
    await db.delete(installedPlugins).where(eq(installedPlugins.robotId, robotId));
    return NextResponse.json({ success: true });
  } catch (error) {
    console.error("Failed to uninstall plugin:", error);
    return NextResponse.json(
      { error: "Failed to uninstall plugin" },
      { status: 500 }
    );
  }
} 