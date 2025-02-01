import { NextResponse } from "next/server";
import { getObjectFromS3 } from "~/server/storage/s3";
import { Readable } from "stream";

export async function GET(
  req: Request,
  { params }: { params: { key: string } }
) {
  try {
    // Ensure params.key is awaited
    const { key } = params;
    const decodedKey = decodeURIComponent(key);
    console.log("Fetching image with key:", decodedKey);
    
    const response = await getObjectFromS3(decodedKey);
    console.log("S3 response received:", {
      contentType: response.ContentType,
      contentLength: response.ContentLength,
    });
    
    if (!response.Body) {
      console.error("No image data in response body");
      return NextResponse.json(
        { error: "Image data not found" },
        { status: 404 }
      );
    }

    const stream = response.Body as Readable;
    const chunks: Buffer[] = [];
    for await (const chunk of stream) {
      chunks.push(Buffer.from(chunk));
    }
    const buffer = Buffer.concat(chunks);
    
    console.log("Image buffer created, size:", buffer.length);

    // Ensure we set the correct image content type
    const contentType = response.ContentType ?? 'image/jpeg';
    
    // Create response headers
    const headers = {
      "Content-Type": contentType,
      "Content-Length": buffer.length.toString(),
      "Cache-Control": "public, max-age=31536000, immutable",
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "GET, OPTIONS",
      "Access-Control-Allow-Headers": "Content-Type",
    };

    console.log("Sending response with headers:", headers);

    // Return the response with explicit headers object
    return new Response(buffer, { 
      status: 200,
      headers,
    });
  } catch (error) {
    console.error("Error serving image:", error);
    if ((error as any)?.name === "NoSuchKey") {
      return NextResponse.json(
        { error: "Image not found" },
        { status: 404 }
      );
    }
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
} 