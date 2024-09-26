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

  const filePath = path.join(process.cwd(), 'content', ...params.path);

  try {
    const file = await fs.readFile(filePath);
    const response = new NextResponse(file);
    response.headers.set('Content-Type', 'application/pdf');
    return response;
  } catch (error) {
    console.error('Error reading file:', error);
    return new NextResponse('File not found', { status: 404 });
  }
}