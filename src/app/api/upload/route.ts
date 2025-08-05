import { eq } from "drizzle-orm";
import { NextResponse, type NextRequest } from "next/server";
import { z } from "zod";
import {
    generateFileKey,
    getMimeType, uploadFile, validateFile
} from "~/lib/storage/minio";
import { auth } from "~/server/auth";
import { db } from "~/server/db";
import { mediaCaptures, trials } from "~/server/db/schema";

const uploadSchema = z.object({
  trialId: z.string().optional(),
  category: z
    .enum(["video", "audio", "image", "document", "sensor_data"])
    .default("document"),
  filename: z.string(),
  contentType: z.string().optional(),
});

export async function POST(request: NextRequest) {
  try {
    // Check authentication
    const session = await auth();
    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Parse form data
    const formData = await request.formData();
    const file = formData.get("file") as File;
    const trialId = formData.get("trialId") as string | null;
    const category = (formData.get("category") as string) || "document";

    if (!file) {
      return NextResponse.json({ error: "No file provided" }, { status: 400 });
    }

    // Validate input
    const validationResult = uploadSchema.safeParse({
      trialId: trialId || undefined,
      category,
      filename: file.name,
      contentType: file.type,
    });

    if (!validationResult.success) {
      return NextResponse.json(
        {
          error: "Invalid request parameters",
          details: validationResult.error.flatten(),
        },
        { status: 400 },
      );
    }

    const { trialId: validatedTrialId, category: validatedCategory } =
      validationResult.data;

    // Validate file type and size based on category
    const fileValidation = validateFileByCategory(file, validatedCategory);
    if (!fileValidation.valid) {
      return NextResponse.json(
        { error: fileValidation.error },
        { status: 400 },
      );
    }

    // Check trial access if trialId is provided
    if (validatedTrialId) {
      const trial = await db
        .select()
        .from(trials)
        .where(eq(trials.id, validatedTrialId))
        .limit(1);

      if (!trial.length) {
        return NextResponse.json({ error: "Trial not found" }, { status: 404 });
      }

      // TODO: Check if user has access to this trial through study membership
    }

    // Generate unique file key
    const fileKey = generateFileKey(
      validatedCategory,
      file.name,
      session.user.id,
      validatedTrialId,
    );

    // Convert file to buffer
    const arrayBuffer = await file.arrayBuffer();
    const buffer = Buffer.from(arrayBuffer);

    // Upload to MinIO
    const uploadResult = await uploadFile({
      key: fileKey,
      body: buffer,
      contentType: file.type || getMimeType(file.name),
      metadata: {
        originalName: file.name,
        uploadedBy: session.user.id,
        uploadedAt: new Date().toISOString(),
        category: validatedCategory,
        ...(validatedTrialId && { trialId: validatedTrialId }),
      },
    });

    // Save media capture record to database
    const mediaCapture = await db
      .insert(mediaCaptures)
      .values({
        trialId: validatedTrialId!,  // Non-null assertion since it's validated above
        format: file.type || getMimeType(file.name),
        fileSize: file.size,
        storagePath: fileKey,
        mediaType: getCaptureType(validatedCategory),
        metadata: {
          uploadedBy: session.user.id,
          category: validatedCategory,
          etag: uploadResult.etag,
          originalName: file.name,
        },
      })
      .returning();

    return NextResponse.json({
      success: true,
      data: {
        id: mediaCapture[0]?.id,
        filename: file.name,
        size: file.size,
        contentType: file.type,
        key: fileKey,
        url: uploadResult.url,
        category: validatedCategory,
        uploadedAt: new Date().toISOString(),
      },
    });
  } catch (error) {
    console.error("Upload error:", error);
    return NextResponse.json(
      {
        error: "Upload failed",
        details: error instanceof Error ? error.message : "Unknown error",
      },
      { status: 500 },
    );
  }
}

// Generate presigned upload URL for direct client uploads
export async function GET(request: NextRequest) {
  try {
    const session = await auth();
    if (!session?.user) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    const { searchParams } = new URL(request.url);
    const filename = searchParams.get("filename");
    const contentType = searchParams.get("contentType");
    const category = searchParams.get("category") || "document";
    const trialId = searchParams.get("trialId");

    if (!filename) {
      return NextResponse.json(
        { error: "Filename is required" },
        { status: 400 },
      );
    }

    // Validate category
    const validCategories = [
      "video",
      "audio",
      "image",
      "document",
      "sensor_data",
    ];
    if (!validCategories.includes(category)) {
      return NextResponse.json({ error: "Invalid category" }, { status: 400 });
    }

    // Generate unique file key
    const fileKey = generateFileKey(
      category,
      filename,
      session.user.id,
      trialId || undefined,
    );

    // Generate presigned URL for upload
    const { getUploadUrl } = await import("~/lib/storage/minio");
    const uploadUrl = await getUploadUrl(fileKey, contentType || undefined);

    return NextResponse.json({
      success: true,
      data: {
        uploadUrl,
        fileKey,
        expiresIn: 3600, // 1 hour
      },
    });
  } catch (error) {
    console.error("Presigned URL generation error:", error);
    return NextResponse.json(
      {
        error: "Failed to generate upload URL",
        details: error instanceof Error ? error.message : "Unknown error",
      },
      { status: 500 },
    );
  }
}

function validateFileByCategory(
  file: File,
  category: string,
): { valid: boolean; error?: string } {
  const maxSizes = {
    video: 500 * 1024 * 1024, // 500MB
    audio: 100 * 1024 * 1024, // 100MB
    image: 10 * 1024 * 1024, // 10MB
    document: 50 * 1024 * 1024, // 50MB
    sensor_data: 100 * 1024 * 1024, // 100MB
  };

  const allowedTypes = {
    video: ["mp4", "avi", "mov", "wmv", "flv", "webm"],
    audio: ["mp3", "wav", "ogg", "m4a"],
    image: ["jpg", "jpeg", "png", "gif", "webp", "svg"],
    document: ["pdf", "doc", "docx", "txt", "csv", "json", "xml"],
    sensor_data: ["csv", "json", "txt", "xml"],
  };

  const maxSize =
    maxSizes[category as keyof typeof maxSizes] || 50 * 1024 * 1024;
  const types = allowedTypes[category as keyof typeof allowedTypes] || [];

  return validateFile(file.name, file.size, types, maxSize);
}

function getCaptureType(
  category: string,
): "video" | "audio" | "image" {
  switch (category) {
    case "video":
      return "video";
    case "audio":
      return "audio";
    case "image":
      return "image";
    case "sensor_data":
      return "image"; // Map sensor data to image for now
    default:
      return "image"; // Default to image
  }
}
