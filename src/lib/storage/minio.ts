import { DeleteObjectCommand, GetObjectCommand, HeadObjectCommand, PutObjectCommand, S3Client } from "@aws-sdk/client-s3";
import { getSignedUrl } from "@aws-sdk/s3-request-presigner";
import { env } from "~/env";

// Configure MinIO S3 client
const s3Client = new S3Client({
  endpoint: env.MINIO_ENDPOINT || "http://localhost:9000",
  region: env.MINIO_REGION || "us-east-1",
  credentials: {
    accessKeyId: env.MINIO_ACCESS_KEY || "minioadmin",
    secretAccessKey: env.MINIO_SECRET_KEY || "minioadmin",
  },
  forcePathStyle: true, // Required for MinIO
});

const BUCKET_NAME = env.MINIO_BUCKET_NAME || "hristudio";
const PRESIGNED_URL_EXPIRY = 3600; // 1 hour in seconds

export interface UploadParams {
  key: string;
  body: Buffer | Uint8Array | string;
  contentType?: string;
  metadata?: Record<string, string>;
}

export interface UploadResult {
  key: string;
  url: string;
  size: number;
  contentType: string;
  etag: string;
}

export interface PresignedUrlOptions {
  expiresIn?: number;
  responseContentType?: string;
  responseContentDisposition?: string;
}

/**
 * Upload a file to MinIO storage
 */
export async function uploadFile(params: UploadParams): Promise<UploadResult> {
  try {
    const command = new PutObjectCommand({
      Bucket: BUCKET_NAME,
      Key: params.key,
      Body: params.body,
      ContentType: params.contentType || "application/octet-stream",
      Metadata: params.metadata,
    });

    const result = await s3Client.send(command);

    return {
      key: params.key,
      url: `${env.MINIO_ENDPOINT}/${BUCKET_NAME}/${params.key}`,
      size: Buffer.isBuffer(params.body) ? params.body.length : params.body.toString().length,
      contentType: params.contentType || "application/octet-stream",
      etag: result.ETag || "",
    };
  } catch (error) {
    console.error("Error uploading file to MinIO:", error);
    throw new Error(`Failed to upload file: ${error instanceof Error ? error.message : "Unknown error"}`);
  }
}

/**
 * Generate a presigned URL for file access
 */
export async function getPresignedUrl(
  key: string,
  operation: "getObject" | "putObject" = "getObject",
  options: PresignedUrlOptions = {}
): Promise<string> {
  try {
    const { expiresIn = PRESIGNED_URL_EXPIRY, responseContentType, responseContentDisposition } = options;

    let command;
    if (operation === "getObject") {
      command = new GetObjectCommand({
        Bucket: BUCKET_NAME,
        Key: key,
        ResponseContentType: responseContentType,
        ResponseContentDisposition: responseContentDisposition,
      });
    } else {
      command = new PutObjectCommand({
        Bucket: BUCKET_NAME,
        Key: key,
        ContentType: responseContentType,
      });
    }

    const url = await getSignedUrl(s3Client, command, { expiresIn });
    return url;
  } catch (error) {
    console.error("Error generating presigned URL:", error);
    throw new Error(`Failed to generate presigned URL: ${error instanceof Error ? error.message : "Unknown error"}`);
  }
}

/**
 * Delete a file from MinIO storage
 */
export async function deleteFile(key: string): Promise<void> {
  try {
    const command = new DeleteObjectCommand({
      Bucket: BUCKET_NAME,
      Key: key,
    });

    await s3Client.send(command);
  } catch (error) {
    console.error("Error deleting file from MinIO:", error);
    throw new Error(`Failed to delete file: ${error instanceof Error ? error.message : "Unknown error"}`);
  }
}

/**
 * Check if a file exists in MinIO storage
 */
export async function fileExists(key: string): Promise<boolean> {
  try {
    const command = new HeadObjectCommand({
      Bucket: BUCKET_NAME,
      Key: key,
    });

    await s3Client.send(command);
    return true;
  } catch (error) {
    if (error instanceof Error && error.name === "NotFound") {
      return false;
    }
    console.error("Error checking file existence:", error);
    throw new Error(`Failed to check file existence: ${error instanceof Error ? error.message : "Unknown error"}`);
  }
}

/**
 * Get file metadata from MinIO storage
 */
export async function getFileMetadata(key: string): Promise<{
  size: number;
  lastModified: Date;
  contentType: string;
  etag: string;
  metadata: Record<string, string>;
}> {
  try {
    const command = new HeadObjectCommand({
      Bucket: BUCKET_NAME,
      Key: key,
    });

    const result = await s3Client.send(command);

    return {
      size: result.ContentLength || 0,
      lastModified: result.LastModified || new Date(),
      contentType: result.ContentType || "application/octet-stream",
      etag: result.ETag || "",
      metadata: result.Metadata || {},
    };
  } catch (error) {
    console.error("Error getting file metadata:", error);
    throw new Error(`Failed to get file metadata: ${error instanceof Error ? error.message : "Unknown error"}`);
  }
}

/**
 * Generate a download URL for a file
 */
export async function getDownloadUrl(key: string, filename?: string): Promise<string> {
  const contentDisposition = filename ? `attachment; filename="${filename}"` : undefined;

  return getPresignedUrl(key, "getObject", {
    responseContentDisposition: contentDisposition,
  });
}

/**
 * Generate an upload URL for direct client uploads
 */
export async function getUploadUrl(key: string, contentType?: string): Promise<string> {
  return getPresignedUrl(key, "putObject", {
    responseContentType: contentType,
  });
}

/**
 * Helper function to generate a unique file key
 */
export function generateFileKey(
  prefix: string,
  filename: string,
  userId?: string,
  trialId?: string
): string {
  const timestamp = Date.now();
  const randomSuffix = Math.random().toString(36).substring(2, 8);

  // Sanitize filename
  const sanitizedFilename = filename.replace(/[^a-zA-Z0-9.-]/g, "_");

  const parts = [prefix];

  if (userId) parts.push(`user-${userId}`);
  if (trialId) parts.push(`trial-${trialId}`);

  parts.push(`${timestamp}-${randomSuffix}-${sanitizedFilename}`);

  return parts.join("/");
}

/**
 * Helper function to get file extension from filename
 */
export function getFileExtension(filename: string): string {
  const lastDot = filename.lastIndexOf(".");
  return lastDot !== -1 ? filename.substring(lastDot + 1).toLowerCase() : "";
}

/**
 * Helper function to get MIME type from file extension
 */
export function getMimeType(filename: string): string {
  const extension = getFileExtension(filename);

  const mimeTypes: Record<string, string> = {
    // Images
    jpg: "image/jpeg",
    jpeg: "image/jpeg",
    png: "image/png",
    gif: "image/gif",
    webp: "image/webp",
    svg: "image/svg+xml",

    // Videos
    mp4: "video/mp4",
    avi: "video/x-msvideo",
    mov: "video/quicktime",
    wmv: "video/x-ms-wmv",
    flv: "video/x-flv",
    webm: "video/webm",

    // Audio
    mp3: "audio/mpeg",
    wav: "audio/wav",
    ogg: "audio/ogg",
    m4a: "audio/mp4",

    // Documents
    pdf: "application/pdf",
    doc: "application/msword",
    docx: "application/vnd.openxmlformats-officedocument.wordprocessingml.document",
    xls: "application/vnd.ms-excel",
    xlsx: "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    ppt: "application/vnd.ms-powerpoint",
    pptx: "application/vnd.openxmlformats-officedocument.presentationml.presentation",

    // Data formats
    json: "application/json",
    xml: "application/xml",
    csv: "text/csv",
    txt: "text/plain",

    // Archives
    zip: "application/zip",
    rar: "application/vnd.rar",
    "7z": "application/x-7z-compressed",
    tar: "application/x-tar",
    gz: "application/gzip",
  };

  return mimeTypes[extension] || "application/octet-stream";
}

/**
 * Validate file type and size
 */
export function validateFile(
  filename: string,
  size: number,
  allowedTypes?: string[],
  maxSize?: number
): { valid: boolean; error?: string } {
  // Check file size (default 100MB limit)
  const maxFileSize = maxSize || 100 * 1024 * 1024;
  if (size > maxFileSize) {
    return {
      valid: false,
      error: `File size exceeds maximum allowed size of ${Math.round(maxFileSize / 1024 / 1024)}MB`,
    };
  }

  // Check file type if specified
  if (allowedTypes && allowedTypes.length > 0) {
    const extension = getFileExtension(filename);
    if (!allowedTypes.includes(extension)) {
      return {
        valid: false,
        error: `File type "${extension}" is not allowed. Allowed types: ${allowedTypes.join(", ")}`,
      };
    }
  }

  return { valid: true };
}

// Export S3 client for advanced usage
export { s3Client };
// Export bucket name for reference
export { BUCKET_NAME };

