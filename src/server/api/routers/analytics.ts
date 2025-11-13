import { TRPCError } from "@trpc/server";
import { and, asc, desc, eq, gte, inArray, lte, type SQL } from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import type { db } from "~/server/db";
import {
    annotations, experiments, exportJobs, exportStatusEnum, studyMembers, trials
} from "~/server/db/schema";

// Helper function to check if user has access to trial for analytics operations
async function checkTrialAccess(
  database: typeof db,
  userId: string,
  trialId: string,
  requiredRoles: ("owner" | "researcher" | "wizard" | "observer")[] = [
    "owner",
    "researcher",
    "wizard",
  ],
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
        inArray(studyMembers.role, requiredRoles),
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

// Helper function to check study access for analytics
async function checkStudyAccess(
  database: typeof db,
  userId: string,
  studyId: string,
  requiredRoles: ("owner" | "researcher" | "wizard" | "observer")[] = [
    "owner",
    "researcher",
  ],
) {
  const membership = await database
    .select()
    .from(studyMembers)
    .where(
      and(
        eq(studyMembers.studyId, studyId),
        eq(studyMembers.userId, userId),
        inArray(studyMembers.role, requiredRoles),
      ),
    )
    .limit(1);

  if (!membership[0]) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "Insufficient permissions to access this study",
    });
  }
}

export const analyticsRouter = createTRPCRouter({
  createAnnotation: protectedProcedure
    .input(
              z.object({
          trialId: z.string(),
          startTime: z.date(),
          endTime: z.date().optional(),
          category: z.string(),
          label: z.string(),
          description: z.string().optional(),
          tags: z.array(z.string()).optional(),
          metadata: z.any().optional(),
        }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.trialId);

      const annotationResults = await db
        .insert(annotations)
        .values({
          trialId: input.trialId,
          annotatorId: userId,
          timestampStart: input.startTime,
          timestampEnd: input.endTime,
          category: input.category,
          label: input.label,
          description: input.description,
          tags: input.tags,
          metadata: input.metadata,
        })
        .returning();

      const annotation = annotationResults[0];
      if (!annotation) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create annotation",
        });
      }

      return annotation;
    }),

  updateAnnotation: protectedProcedure
    .input(
              z.object({
          id: z.string(),
          startTime: z.date().optional(),
          endTime: z.date().optional(),
          category: z.string().optional(),
          label: z.string().optional(),
          description: z.string().optional(),
          tags: z.array(z.string()).optional(),
          metadata: z.any().optional(),
        }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Get annotation to check access
      const existingAnnotation = await db
        .select({
          id: annotations.id,
          trialId: annotations.trialId,
          annotatorId: annotations.annotatorId,
        })
        .from(annotations)
        .where(eq(annotations.id, input.id))
        .limit(1);

      if (!existingAnnotation[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Annotation not found",
        });
      }

      // Check trial access
      await checkTrialAccess(db, userId, existingAnnotation[0].trialId);

      // Only allow annotation creator or study owners/researchers to edit
      if (existingAnnotation[0].annotatorId !== userId) {
        await checkTrialAccess(db, userId, existingAnnotation[0].trialId, [
          "owner",
          "researcher",
        ]);
      }

      const updateData: {
        updatedAt: Date;
        timestampStart?: Date;
        timestampEnd?: Date;
        category?: string;
        label?: string;
        description?: string;
        tags?: string[];
        metadata?: Record<string, unknown>;
      } = {
        updatedAt: new Date(),
      };

      if (input.startTime !== undefined)
        updateData.timestampStart = input.startTime;
      if (input.endTime !== undefined) updateData.timestampEnd = input.endTime;
      if (input.category !== undefined) updateData.category = input.category;
      if (input.label !== undefined) updateData.label = input.label;
      if (input.description !== undefined)
        updateData.description = input.description;
      if (input.tags !== undefined) updateData.tags = input.tags;
      if (input.metadata !== undefined) updateData.metadata = input.metadata as Record<string, unknown>;

      const annotationResults = await db
        .update(annotations)
        .set(updateData)
        .where(eq(annotations.id, input.id))
        .returning();

      const annotation = annotationResults[0];
      if (!annotation) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update annotation",
        });
      }

      return annotation;
    }),

  deleteAnnotation: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Get annotation to check access
      const existingAnnotation = await db
        .select({
          id: annotations.id,
          trialId: annotations.trialId,
          annotatorId: annotations.annotatorId,
        })
        .from(annotations)
        .where(eq(annotations.id, input.id))
        .limit(1);

      if (!existingAnnotation[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Annotation not found",
        });
      }

      // Check trial access
      await checkTrialAccess(db, userId, existingAnnotation[0].trialId);

      // Only allow annotation creator or study owners/researchers to delete
      if (existingAnnotation[0].annotatorId !== userId) {
        await checkTrialAccess(db, userId, existingAnnotation[0].trialId, [
          "owner",
          "researcher",
        ]);
      }

      await db.delete(annotations).where(eq(annotations.id, input.id));

      return { success: true };
    }),

  getAnnotations: protectedProcedure
    .input(
              z.object({
          trialId: z.string(),
          category: z.string().optional(),
          annotatorId: z.string().optional(),
          startTime: z.date().optional(),
          endTime: z.date().optional(),
          tags: z.array(z.string()).optional(),
          limit: z.number().min(1).max(1000).default(100),
          offset: z.number().min(0).default(0),
        }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.trialId);

      const conditions: SQL[] = [eq(annotations.trialId, input.trialId)];

      if (input.category) {
        conditions.push(eq(annotations.category, input.category));
      }
      if (input.annotatorId) {
        conditions.push(eq(annotations.annotatorId, input.annotatorId));
      }
      if (input.startTime !== undefined) {
        conditions.push(gte(annotations.timestampStart, input.startTime));
      }
      if (input.endTime !== undefined) {
        conditions.push(lte(annotations.timestampEnd, input.endTime));
      }

      const rawResults = await db
        .select()
        .from(annotations)
        .where(and(...conditions))
        .orderBy(asc(annotations.timestampStart))
        .limit(input.limit)
        .offset(input.offset);

      // Map to expected output format
      const results = rawResults.map((annotation) => ({
        id: annotation.id,
        trialId: annotation.trialId,
        annotatorId: annotation.annotatorId,
        startTime: annotation.timestampStart,
        endTime: annotation.timestampEnd,
        category: annotation.category,
        label: annotation.label,
        description: annotation.description,
        tags: annotation.tags as string[],
        metadata: annotation.metadata,
        createdAt: annotation.createdAt,
        updatedAt: annotation.updatedAt,
      }));

      // Filter by tags if provided
      if (input.tags && input.tags.length > 0) {
        return results.filter((annotation) => {
          if (!annotation.tags || !Array.isArray(annotation.tags)) return false;
          return input.tags!.some((tag) =>
            annotation.tags.includes(tag),
          );
        });
      }

      return results;
    }),

  exportData: protectedProcedure
    .input(
              z.object({
          studyId: z.string(),
          exportType: z.enum(["full", "trials", "analysis", "media"]),
          format: z.enum(["csv", "json", "xlsx"]),
          filters: z.any().optional(),
        }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkStudyAccess(db, userId, input.studyId);

      // Create export job
      const exportJobResults = await db
        .insert(exportJobs)
        .values({
          studyId: input.studyId,
          requestedBy: userId,
          exportType: input.exportType,
          format: input.format,
          filters: input.filters,
          status: "pending",
        })
        .returning();

      const exportJob = exportJobResults[0];
      if (!exportJob) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create export job",
        });
      }

      // TODO: Trigger background job to process export
      // This would typically be handled by a queue system like Bull/BullMQ
      // For now, we'll simulate the process

      // Simulate processing time
      // Capture variables for setTimeout closure
      const jobId = exportJob.id;
      const studyId = input.studyId;
      const format = input.format;
      const database = db;

      setTimeout(() => {
        // Mock file generation
        const fileName = `study-${studyId}-export-${Date.now()}.${format}`;
        const fileUrl = `https://mock-r2-bucket.com/exports/${fileName}`;

        database
          .update(exportJobs)
          .set({
            status: "completed",
            storagePath: fileUrl,
            completedAt: new Date(),
          })
          .where(eq(exportJobs.id, jobId))
          .then(() => {
            // Success handled
          })
          .catch((error: unknown) => {
                          database
                .update(exportJobs)
                .set({
                  status: "failed",
                  errorMessage:
                    error instanceof Error
                      ? error.message
                      : "Export processing failed",
                })
              .where(eq(exportJobs.id, jobId))
              .catch(() => {
                // Error handling the error update - ignore for now
              });
          });
      }, 5000); // 5 second delay

      return {
        jobId: exportJob.id,
        status: exportJob.status,
        estimatedCompletionTime: new Date(Date.now() + 30000), // 30 seconds
      };
    }),

  getExportStatus: protectedProcedure
    .input(
      z.object({
        jobId: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      const exportJob = await db
        .select({
          id: exportJobs.id,
          studyId: exportJobs.studyId,
          requestedBy: exportJobs.requestedBy,
          exportType: exportJobs.exportType,
          format: exportJobs.format,
          status: exportJobs.status,
          storagePath: exportJobs.storagePath,
          errorMessage: exportJobs.errorMessage,
          filters: exportJobs.filters,
          createdAt: exportJobs.createdAt,
          completedAt: exportJobs.completedAt,
        })
        .from(exportJobs)
        .where(eq(exportJobs.id, input.jobId))
        .limit(1);

      if (!exportJob[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Export job not found",
        });
      }

      // Check user has access to the study
      await checkStudyAccess(db, userId, exportJob[0].studyId);

      return exportJob[0];
    }),

  getExportHistory: protectedProcedure
    .input(
      z.object({
        studyId: z.string(),
        status: z.enum(exportStatusEnum.enumValues).optional(),
        limit: z.number().min(1).max(100).default(20),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkStudyAccess(db, userId, input.studyId);

      const conditions: SQL[] = [eq(exportJobs.studyId, input.studyId)];

      if (input.status) {
        conditions.push(eq(exportJobs.status, input.status));
      }

      const results = await db
        .select()
        .from(exportJobs)
        .where(and(...conditions))
        .orderBy(desc(exportJobs.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return results;
    }),

  getTrialStatistics: protectedProcedure
    .input(
      z.object({
        studyId: z.string(),
        experimentId: z.string().optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkStudyAccess(db, userId, input.studyId);

      // Get trial statistics
      const conditions: SQL[] = [eq(experiments.studyId, input.studyId)];
      if (input.experimentId) {
        conditions.push(eq(trials.experimentId, input.experimentId));
      }

      const trialStats = await db
        .select({
          trial: trials,
          experiment: experiments,
        })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .where(and(...conditions));

      // Calculate statistics
      const stats = {
        totalTrials: trialStats.length,
        completedTrials: trialStats.filter((t) => t.trial.status === "completed")
          .length,
        runningTrials: trialStats.filter((t) => t.trial.status === "in_progress")
          .length,
        abortedTrials: trialStats.filter((t) => t.trial.status === "aborted").length,
        avgDuration: 0,
        totalDuration: 0,
      };

      const completedTrials = trialStats.filter(
        (t) => t.trial.status === "completed" && t.trial.duration !== null,
      );

      if (completedTrials.length > 0) {
        const durations = completedTrials.map((t) => t.trial.duration!);
        stats.totalDuration = durations.reduce((sum, d) => sum + d, 0);
        stats.avgDuration = stats.totalDuration / durations.length;
      }

      return stats;
    }),
});
