import { TRPCError } from "@trpc/server";
import { and, asc, desc, eq, gte, inArray, lte, type SQL } from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import type { db } from "~/server/db";
import {
    experiments,
    mediaCaptures,
    sensorData,
    studyMembers,
    trials
} from "~/server/db/schema";

// Helper function to check if user has access to trial for media operations
async function checkTrialAccess(
  database: typeof db,
  userId: string,
  trialId: string,
  requiredRoles: string[] = ["owner", "researcher", "wizard"],
) {
  const trial = await database
    .select({
      id: trials.id,
      experimentId: trials.experimentId,
      studyId: experiments.studyId,
    })
    .from(trials)
    .innerJoin(experiments, eq(trials.experimentId, experiments.id))
    .where(eq(trials.id, trialId))
    .limit(1);

  if (!trial[0]) {
    throw new TRPCError({
      code: "NOT_FOUND",
      message: "Trial not found",
    });
  }

  const membership = await database
    .select()
    .from(studyMembers)
    .where(
      and(
        eq(studyMembers.studyId, trial[0].studyId),
        eq(studyMembers.userId, userId),
        inArray(studyMembers.role, requiredRoles as ("owner" | "researcher" | "wizard" | "observer")[]),
      ),
    )
    .limit(1);

  if (!membership[0]) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "Insufficient permissions to access this trial",
    });
  }

  return trial[0];
}

// Helper function to generate presigned upload URL for R2
async function generatePresignedUploadUrl(
  fileName: string,
  contentType: string,
  studyId: string,
): Promise<{ uploadUrl: string; fileUrl: string }> {
  // TODO: Implement actual R2 presigned URL generation
  // This would use AWS SDK or similar to generate presigned URLs for Cloudflare R2

  const key = `studies/${studyId}/media/${Date.now()}-${fileName}`;

  // Mock implementation - replace with actual R2 integration
  return {
    uploadUrl: `https://mock-r2-bucket.com/upload/${key}`,
    fileUrl: `https://mock-r2-bucket.com/files/${key}`,
  };
}

export const mediaRouter = createTRPCRouter({
  uploadVideo: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        fileName: z.string(),
        fileSize: z.number().min(1),
        contentType: z.string(),
        duration: z.number().optional(),
        metadata: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const trial = await checkTrialAccess(db, userId, input.trialId);

      // Validate content type
      if (!input.contentType.startsWith("video/")) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invalid content type for video upload",
        });
      }

      // Generate presigned upload URL
      const { uploadUrl, fileUrl } = await generatePresignedUploadUrl(
        input.fileName,
        input.contentType,
        trial.studyId,
      );

      // Create media capture record
      const mediaCaptureResults = await db
        .insert(mediaCaptures)
        .values({
          trialId: input.trialId,
          mediaType: "video",
          storagePath: fileUrl,
          fileSize: input.fileSize,
          duration: input.duration,
          metadata: input.metadata,
        })
        .returning();

      const mediaCapture = mediaCaptureResults[0];
      if (!mediaCapture) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create media capture record",
        });
      }

      return {
        mediaCapture,
        uploadUrl,
      };
    }),

  uploadAudio: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        fileName: z.string(),
        fileSize: z.number().min(1),
        contentType: z.string(),
        duration: z.number().optional(),
        metadata: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const trial = await checkTrialAccess(db, userId, input.trialId);

      // Validate content type
      if (!input.contentType.startsWith("audio/")) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invalid content type for audio upload",
        });
      }

      // Generate presigned upload URL
      const { uploadUrl, fileUrl } = await generatePresignedUploadUrl(
        input.fileName,
        input.contentType,
        trial.studyId,
      );

      // Create media capture record
      const mediaCaptureResults2 = await db
        .insert(mediaCaptures)
        .values({
          trialId: input.trialId,
          mediaType: "audio",
          storagePath: fileUrl,
          fileSize: input.fileSize,
          format: input.contentType,
          duration: input.duration,
          metadata: input.metadata,
        })
        .returning();

      const mediaCapture = mediaCaptureResults2[0];
      if (!mediaCapture) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create media capture record",
        });
      }

      return {
        mediaCapture,
        uploadUrl,
      };
    }),

  list: protectedProcedure
    .input(
      z.object({
        trialId: z.string().optional(),
        studyId: z.string().optional(),
        type: z.enum(["video", "audio", "image"]).optional(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const conditions: SQL[] = [];

      if (input.trialId) {
        await checkTrialAccess(db, userId, input.trialId);
        conditions.push(eq(mediaCaptures.trialId, input.trialId));
      }

              if (input.type) {
          conditions.push(eq(mediaCaptures.mediaType, input.type));
        }

      const whereClause = and(
        eq(studyMembers.userId, userId),
        inArray(studyMembers.role, ["owner", "researcher", "wizard"] as ("owner" | "researcher" | "wizard" | "observer")[]),
        ...conditions,
      );

      const results = await db
        .select({
          id: mediaCaptures.id,
          trialId: mediaCaptures.trialId,
          mediaType: mediaCaptures.mediaType,
          storagePath: mediaCaptures.storagePath,
          fileSize: mediaCaptures.fileSize,
          format: mediaCaptures.format,
          duration: mediaCaptures.duration,
          metadata: mediaCaptures.metadata,
          createdAt: mediaCaptures.createdAt,
          trial: {
            id: trials.id,
            experimentId: trials.experimentId,
          },
        })
        .from(mediaCaptures)
        .innerJoin(trials, eq(mediaCaptures.trialId, trials.id))
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .innerJoin(studyMembers, eq(studyMembers.studyId, experiments.studyId))
        .where(whereClause)
        .orderBy(desc(mediaCaptures.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return results;
    }),

  getUrl: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const media = await db
        .select({
          id: mediaCaptures.id,
          trialId: mediaCaptures.trialId,
          storagePath: mediaCaptures.storagePath,
          format: mediaCaptures.format,
        })
        .from(mediaCaptures)
        .where(eq(mediaCaptures.id, input.id))
        .limit(1);

      if (!media[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Media file not found",
        });
      }

      // Check access through trial
      await checkTrialAccess(db, userId, media[0].trialId);

      // TODO: Generate presigned download URL for R2
      // For now, return the stored file path
      return {
        url: media[0].storagePath,
        fileName: media[0].storagePath.split('/').pop() ?? 'unknown',
        contentType: media[0].format ?? 'application/octet-stream',
        expiresAt: new Date(Date.now() + 60 * 60 * 1000), // 1 hour
      };
    }),

  delete: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const media = await db
        .select({
          id: mediaCaptures.id,
          trialId: mediaCaptures.trialId,
          storagePath: mediaCaptures.storagePath,
        })
        .from(mediaCaptures)
        .where(eq(mediaCaptures.id, input.id))
        .limit(1);

      if (!media[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Media file not found",
        });
      }

      // Check access through trial (only researchers and owners can delete)
      await checkTrialAccess(db, userId, media[0].trialId, [
        "owner",
        "researcher",
      ]);

      // Delete from database
      await db.delete(mediaCaptures).where(eq(mediaCaptures.id, input.id));

      // TODO: Delete from R2 storage
      // await deleteFromR2(media[0].storagePath);

      return { success: true };
    }),

  // Sensor data recording and querying
  sensorData: createTRPCRouter({
    record: protectedProcedure
      .input(
        z.object({
          trialId: z.string(),
          sensorType: z.string(),
          timestamp: z.date(),
                  data: z.any(),
        metadata: z.any().optional(),
        }),
      )
      .mutation(async ({ ctx, input }) => {
        const { db } = ctx;
        const userId = ctx.session.user.id;

        await checkTrialAccess(db, userId, input.trialId);

        const sensorRecordResults = await db
          .insert(sensorData)
          .values({
            trialId: input.trialId,
            sensorType: input.sensorType,
            timestamp: input.timestamp,
            data: input.data,
          })
          .returning();

        const sensorRecord = sensorRecordResults[0];
        if (!sensorRecord) {
          throw new TRPCError({
            code: "INTERNAL_SERVER_ERROR",
            message: "Failed to create sensor data record",
          });
        }

        return sensorRecord;
      }),

    query: protectedProcedure
      .input(
        z.object({
          trialId: z.string(),
          sensorType: z.string().optional(),
          startTime: z.date().optional(),
          endTime: z.date().optional(),
          limit: z.number().min(1).max(10000).default(1000),
          offset: z.number().min(0).default(0),
        }),
      )
      .query(async ({ ctx, input }) => {
        const { db } = ctx;
        const userId = ctx.session.user.id;

        await checkTrialAccess(db, userId, input.trialId);

        const conditions = [eq(sensorData.trialId, input.trialId)];

        if (input.sensorType) {
          conditions.push(eq(sensorData.sensorType, input.sensorType));
        }
        if (input.startTime) {
          conditions.push(gte(sensorData.timestamp, input.startTime));
        }
        if (input.endTime) {
          conditions.push(lte(sensorData.timestamp, input.endTime));
        }

        const results = await db
          .select()
          .from(sensorData)
          .where(and(...conditions))
          .orderBy(asc(sensorData.timestamp))
          .limit(input.limit)
          .offset(input.offset);

        return results;
      }),
  }),
});
