import fs from 'fs/promises';
import path from 'path';

const CONTENT_DIR = path.join(process.cwd(), 'content');

export async function saveFile(file: File, contentId: number): Promise<string> {
  const contentFolder = path.join(CONTENT_DIR, contentId.toString());
  await fs.mkdir(contentFolder, { recursive: true });

  const buffer = Buffer.from(await file.arrayBuffer());
  const filePath = path.join(contentFolder, file.name);
  await fs.writeFile(filePath, buffer);

  return filePath;
}