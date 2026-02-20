import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { participantDocuments } from "~/server/db/schema";
import { TRPCError } from "@trpc/server";
import { env } from "~/env";
import * as Minio from "minio";
import { uuid } from "drizzle-orm/pg-core";
import { eq, desc } from "drizzle-orm";

// Initialize MinIO client
const minioUrl = new URL(env.MINIO_ENDPOINT ?? "http://localhost:9000");

const minioClient = new Minio.Client({
    endPoint: minioUrl.hostname,
    port: parseInt(minioUrl.port) || 9000,
    useSSL: minioUrl.protocol === "https:",
    accessKey: env.MINIO_ACCESS_KEY ?? "minioadmin",
    secretKey: env.MINIO_SECRET_KEY ?? "minioadmin",
});

const BUCKET_NAME = env.MINIO_BUCKET_NAME ?? "hristudio-assets";

// Ensure bucket exists on startup (best effort)
const ensureBucket = async () => {
    try {
        const exists = await minioClient.bucketExists(BUCKET_NAME);
        if (!exists) {
            await minioClient.makeBucket(BUCKET_NAME, env.MINIO_REGION ?? "us-east-1");
            // Set public policy if needed? For now, keep private and use presigned URLs.
        }
    } catch (e) {
        console.error("Error ensuring MinIO bucket exists:", e);
    }
}
void ensureBucket(); // Fire and forget on load

export const filesRouter = createTRPCRouter({
    // Get a presigned URL for uploading a file
    getPresignedUrl: protectedProcedure
        .input(z.object({
            filename: z.string(),
            contentType: z.string(),
            participantId: z.string(),
        }))
        .mutation(async ({ input }) => {
            const fileExtension = input.filename.split(".").pop();
            const uniqueFilename = `${input.participantId}/${crypto.randomUUID()}.${fileExtension}`;

            try {
                const presignedUrl = await minioClient.presignedPutObject(
                    BUCKET_NAME,
                    uniqueFilename,
                    60 * 5 // 5 minutes expiry
                );

                return {
                    url: presignedUrl,
                    storagePath: uniqueFilename, // Pass this back to client to save in DB after upload
                };
            } catch (error) {
                console.error("Error generating presigned URL:", error);
                throw new TRPCError({
                    code: "INTERNAL_SERVER_ERROR",
                    message: "Failed to generate upload URL",
                });
            }
        }),

    // Get a presigned URL for downloading/viewing a file
    getDownloadUrl: protectedProcedure
        .input(z.object({
            storagePath: z.string(),
        }))
        .query(async ({ input }) => {
            try {
                const url = await minioClient.presignedGetObject(
                    BUCKET_NAME,
                    input.storagePath,
                    60 * 60 // 1 hour
                );
                return { url };
            } catch (error) {
                throw new TRPCError({
                    code: "NOT_FOUND",
                    message: "File not found or storage error",
                });
            }
        }),

    // Record a successful upload in the database
    registerUpload: protectedProcedure
        .input(z.object({
            participantId: z.string(),
            name: z.string(),
            type: z.string().optional(),
            storagePath: z.string(),
            fileSize: z.number().optional(),
        }))
        .mutation(async ({ ctx, input }) => {
            await ctx.db.insert(participantDocuments).values({
                participantId: input.participantId,
                name: input.name,
                type: input.type,
                storagePath: input.storagePath,
                fileSize: input.fileSize,
                uploadedBy: ctx.session.user.id,
            });
        }),

    // List documents for a participant
    listParticipantDocuments: protectedProcedure
        .input(z.object({ participantId: z.string() }))
        .query(async ({ ctx, input }) => {
            return await ctx.db.query.participantDocuments.findMany({
                where: eq(participantDocuments.participantId, input.participantId),
                orderBy: [desc(participantDocuments.createdAt)],
                with: {
                    // Optional: join with uploader info if needed
                }
            });
        }),

    // Delete a document
    deleteDocument: protectedProcedure
        .input(z.object({ id: z.string() }))
        .mutation(async ({ ctx, input }) => {
            const doc = await ctx.db.query.participantDocuments.findFirst({
                where: eq(participantDocuments.id, input.id),
            });

            if (!doc) {
                throw new TRPCError({ code: "NOT_FOUND", message: "Document not found" });
            }

            // Delete from database
            await ctx.db.delete(participantDocuments).where(eq(participantDocuments.id, input.id));

            // Delete from MinIO (fire and forget or await)
            try {
                await minioClient.removeObject(BUCKET_NAME, doc.storagePath);
            } catch (e) {
                console.error("Failed to delete object from S3:", e);
                // We still consider the operation successful for the user as the DB record is gone.
            }
        }),
});
