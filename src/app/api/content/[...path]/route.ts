import { NextRequest, NextResponse } from 'next/server';
import path from 'path';
import fs from 'fs/promises';
import { auth } from "@clerk/nextjs/server";

export async function GET(
  request: NextRequest,
  { params }: { params: { path: string[] } }
) {
  const { userId } = auth();
  if (!userId) {
    return new NextResponse('Unauthorized', { status: 401 });
  }

  // Construct the file path relative to the project root
  const filePath = path.join(process.cwd(), 'content', ...params.path);

  console.log('Attempting to read file:', filePath); // Add this log

  try {
    const file = await fs.readFile(filePath);
    const response = new NextResponse(file);
    
    // Determine content type based on file extension
    const ext = path.extname(filePath).toLowerCase();
    switch (ext) {
      case '.pdf':
        response.headers.set('Content-Type', 'application/pdf');
        break;
      case '.png':
        response.headers.set('Content-Type', 'image/png');
        break;
      case '.jpg':
      case '.jpeg':
        response.headers.set('Content-Type', 'image/jpeg');
        break;
      default:
        response.headers.set('Content-Type', 'application/octet-stream');
    }

    return response;
  } catch (error) {
    console.error('Error reading file:', error);
    return new NextResponse('File not found', { status: 404 });
  }
}