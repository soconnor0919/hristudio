
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { s3Client } from "~/server/storage";
import { PutObjectCommand } from "@aws-sdk/client-s3";
import { getSignedUrl } from "@aws-sdk/s3-request-presigner";
import { env } from "~/env";
import { TRPCError } from "@trpc/server";
import { db } from "~/server/db";
import { mediaCaptures } from "~/server/db/schema";

export const storageRouter = createTRPCRouter({
    getUploadPresignedUrl: protectedProcedure
        .input(
            z.object({
                filename: z.string(),
                contentType: z.string(),
            })
        )
        .mutation(async ({ input }) => {
            const bucket = env.MINIO_BUCKET_NAME ?? "hristudio-data";
            const key = input.filename;

            try {
                const command = new PutObjectCommand({
                    Bucket: bucket,
                    Key: key,
                    ContentType: input.contentType,
                });

                const url = await getSignedUrl(s3Client, command, { expiresIn: 3600 });

                return {
                    url,
                    key,
                    bucket,
                };
            } catch (error) {
                console.error("Error generating presigned URL:", error);
                throw new TRPCError({
                    code: "INTERNAL_SERVER_ERROR",
                    message: "Failed to generate upload URL",
                });
            }
        }),
    saveRecording: protectedProcedure
        .input(
            z.object({
                trialId: z.string(),
                storagePath: z.string(),
                fileSize: z.number().optional(),
                format: z.string().optional(),
                mediaType: z.enum(["video", "audio", "image"]).default("video"),
            })
        )
        .mutation(async ({ ctx, input }) => {
            const { db } = ctx;

            await db.insert(mediaCaptures).values({
                trialId: input.trialId,
                mediaType: input.mediaType,
                storagePath: input.storagePath,
                fileSize: input.fileSize,
                format: input.format,
                startTimestamp: new Date(), // Approximate
                // metadata: { uploadedBy: ctx.session.user.id }
            });

            return { success: true };
        }),
});
