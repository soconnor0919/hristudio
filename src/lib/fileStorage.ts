import fs from 'fs/promises';
import path from 'path';
import { fromBuffer } from 'pdf2pic';

const CONTENT_DIR = path.join(process.cwd(), 'content');

export async function saveFile(file: File, filePath: string, previewContentTypeId: number): Promise<{ pdfPath: string; previewPath: string }> {
  const fullPath = path.join(CONTENT_DIR, filePath);
  const dir = path.dirname(fullPath);
  await fs.mkdir(dir, { recursive: true });

  const buffer = Buffer.from(await file.arrayBuffer());
  await fs.writeFile(fullPath, buffer);

  // Generate preview image
  const previewFileName = path.basename(filePath, '.pdf') + '_preview.png';
  const previewDir = path.join(CONTENT_DIR, previewContentTypeId.toString());
  await fs.mkdir(previewDir, { recursive: true });
  const previewPath = path.join(previewDir, previewFileName);

  const options = {
    density: 100,
    saveFilename: path.basename(previewPath, '.png'),
    savePath: previewDir,
    format: "png",
    width: 600,
    height: 800
  };

  const convert = fromBuffer(buffer, options);
  const result = await convert(1);

  // Rename the file to remove the ".1" suffix
  const generatedFilePath = result.path;
  if (generatedFilePath) {
    await fs.rename(generatedFilePath, previewPath);
  }

  // Return relative paths that can be used in URLs
  return {
    pdfPath: `/content/${filePath}`,
    previewPath: `/content/${previewContentTypeId}/${previewFileName}`
  };
}