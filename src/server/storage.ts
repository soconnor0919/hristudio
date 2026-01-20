import { S3Client } from "@aws-sdk/client-s3";
import { env } from "~/env";

const globalForS3 = globalThis as unknown as {
    s3Client: S3Client | undefined;
};

export const s3Client =
    globalForS3.s3Client ??
    new S3Client({
        region: env.MINIO_REGION ?? "us-east-1",
        endpoint: env.MINIO_ENDPOINT,
        credentials: {
            accessKeyId: env.MINIO_ACCESS_KEY ?? "minioadmin",
            secretAccessKey: env.MINIO_SECRET_KEY ?? "minioadmin",
        },
        forcePathStyle: true, // Needed for MinIO
    });

if (env.NODE_ENV !== "production") globalForS3.s3Client = s3Client;
