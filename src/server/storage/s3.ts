"use server";

import {
  S3Client,
  GetObjectCommand,
  PutObjectCommand,
} from "@aws-sdk/client-s3";
import { getSignedUrl } from "@aws-sdk/s3-request-presigner";

// Validate and get S3 configuration
async function validateConfig() {
  const requiredEnvVars = [
    "S3_ENDPOINT",
    "S3_REGION",
    "S3_ACCESS_KEY",
    "S3_SECRET_KEY",
    "S3_BUCKET_NAME",
  ] as const;

  const missing = requiredEnvVars.filter((key) => !process.env[key]);
  if (missing.length > 0) {
    throw new Error(
      `Missing required environment variables: ${missing.join(", ")}`,
    );
  }

  return {
    endpoint: process.env.S3_ENDPOINT!,
    region: process.env.S3_REGION!,
    accessKey: process.env.S3_ACCESS_KEY!,
    secretKey: process.env.S3_SECRET_KEY!,
    bucketName: process.env.S3_BUCKET_NAME!,
    usePathStyle: process.env.S3_USE_PATH_STYLE_ENDPOINT === "true",
  };
}

// Lazy initialization of the S3 client
let s3Client: S3Client | null = null;

export async function getS3Client() {
  if (!s3Client) {
    const config = await validateConfig();
    console.log("Creating S3 client with config:", {
      endpoint: config.endpoint,
      region: config.region,
      forcePathStyle: config.usePathStyle,
    });
    s3Client = new S3Client({
      endpoint: config.endpoint,
      region: config.region,
      credentials: {
        accessKeyId: config.accessKey,
        secretAccessKey: config.secretKey,
      },
      forcePathStyle: config.usePathStyle,
    });
  }
  return s3Client;
}

export async function uploadToS3(
  file: Buffer,
  key: string,
  contentType: string,
) {
  const config = await validateConfig();
  const client = await getS3Client();
  console.log("Uploading file:", { key, contentType, size: file.length });

  const command = new PutObjectCommand({
    Bucket: config.bucketName,
    Key: key,
    Body: file,
    ContentType: contentType,
  });

  try {
    await client.send(command);
    console.log("File uploaded successfully to S3:", { key });
    return `http://localhost:3000/api/images/${encodeURIComponent(key)}`;
  } catch (error) {
    console.error("Error uploading to S3:", error);
    throw error;
  }
}

export async function getObjectFromS3(key: string) {
  const config = await validateConfig();
  const client = await getS3Client();
  console.log("Getting object from S3:", { key, bucket: config.bucketName });

  const command = new GetObjectCommand({
    Bucket: config.bucketName,
    Key: key,
  });

  try {
    const response = await client.send(command);
    console.log("S3 get object response:", {
      contentType: response.ContentType,
      contentLength: response.ContentLength,
    });
    if (!response.Body) {
      throw new Error("No object body received");
    }
    return response;
  } catch (error) {
    console.error("Error getting object from S3:", error);
    throw error;
  }
}

export async function getUploadUrl(key: string, contentType: string) {
  const config = await validateConfig();
  const client = await getS3Client();
  const command = new PutObjectCommand({
    Bucket: config.bucketName,
    Key: key,
    ContentType: contentType,
  });

  try {
    const url = await getSignedUrl(client, command, { expiresIn: 3600 });
    return url;
  } catch (error) {
    console.error("Error generating upload URL:", error);
    throw error;
  }
}

export async function getDownloadUrl(key: string) {
  const config = await validateConfig();
  const client = await getS3Client();
  const command = new GetObjectCommand({
    Bucket: config.bucketName,
    Key: key,
  });

  try {
    const url = await getSignedUrl(client, command, { expiresIn: 3600 });
    return url;
  } catch (error) {
    console.error("Error generating download URL:", error);
    throw error;
  }
}
