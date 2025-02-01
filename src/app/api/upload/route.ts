import { NextResponse } from "next/server";
import { getServerAuthSession } from "~/server/auth";
import { uploadToS3 } from "~/server/storage/s3";
import { nanoid } from "nanoid";

export async function POST(req: Request) {
  try {
    const session = await getServerAuthSession();
    if (!session) {
      return NextResponse.json({ error: "Unauthorized" }, { status: 401 });
    }

    // Get the form data
    const formData = await req.formData();
    const file = formData.get("file") as File;
    
    if (!file) {
      return NextResponse.json(
        { error: "No file provided" },
        { status: 400 }
      );
    }

    // Generate a unique key for the file
    const key = `${session.user.id}/${nanoid()}.${file.type.split("/")[1]}`;

    // Convert file to buffer and upload
    const arrayBuffer = await file.arrayBuffer();
    const buffer = Buffer.from(arrayBuffer);
    const url = await uploadToS3(buffer, key, file.type);

    console.log("File uploaded successfully:", key);

    return NextResponse.json({ url });
  } catch (error) {
    console.error("Error handling upload:", error);
    return NextResponse.json(
      { error: "Internal server error" },
      { status: 500 }
    );
  }
} 