import { TRPCError } from "@trpc/server";
import { and, count, desc, eq, ilike, inArray, or } from "drizzle-orm";
import { z } from "zod";

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import type { db } from "~/server/db";
import {
  activityLogs, consentForms, participantConsents, participants, studyMembers, trials
} from "~/server/db/schema";
import { getUploadUrl, validateFile } from "~/lib/storage/minio";

// Helper function to check study access
async function checkStudyAccess(
  database: typeof db,
  userId: string,
  studyId: string,
  requiredRole?: string[],
) {
  const membership = await database.query.studyMembers.findFirst({
    where: and(
      eq(studyMembers.studyId, studyId),
      eq(studyMembers.userId, userId),
    ),
  });

  if (!membership) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You don't have access to this study",
    });
  }

  if (requiredRole && !requiredRole.includes(membership.role)) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "You don't have permission to perform this action",
    });
  }

  return membership;
}

export const participantsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        search: z.string().optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { studyId, page, limit, search } = input;
      const offset = (page - 1) * limit;
      const userId = ctx.session.user.id;

      // Check study access
      await checkStudyAccess(ctx.db, userId, studyId);

      // Build where conditions
      const conditions = [eq(participants.studyId, studyId)];

      if (search) {
        conditions.push(
          or(
            ilike(participants.participantCode, `%${search}%`),
            ilike(participants.name, `%${search}%`),
            ilike(participants.email, `%${search}%`),
          )!,
        );
      }

      const whereClause = and(...conditions);

      // Get participants with consent info
      const participantsList = await ctx.db.query.participants.findMany({
        where: whereClause,
        with: {
          consents: {
            with: {
              consentForm: {
                columns: {
                  id: true,
                  title: true,
                  version: true,
                },
              },
            },
            orderBy: [desc(participantConsents.signedAt)],
          },
          trials: {
            columns: {
              id: true,
              status: true,
              scheduledAt: true,
              completedAt: true,
            },
            orderBy: [desc(trials.scheduledAt)],
          },
        },
        columns: {
          // Exclude sensitive data from list view
          demographics: false,
          notes: false,
        },
        limit,
        offset,
        orderBy: [desc(participants.createdAt)],
      });

      // Get total count
      const totalCountResult = await ctx.db
        .select({ count: count() })
        .from(participants)
        .where(whereClause);

      const totalCount = totalCountResult[0]?.count ?? 0;

      return {
        participants: participantsList.map((participant) => ({
          ...participant,
          trialCount: participant.trials.length,
          hasConsent: participant.consents.length > 0,
          latestConsent: participant.consents[0] ?? null,
        })),
        pagination: {
          page,
          limit,
          total: totalCount,
          pages: Math.ceil(totalCount / limit),
        },
      };
    }),

  get: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
        with: {
          study: {
            columns: {
              id: true,
              name: true,
            },
          },
          consents: {
            with: {
              consentForm: true,
            },
            orderBy: [desc(participantConsents.signedAt)],
          },
          trials: {
            with: {
              experiment: {
                columns: {
                  id: true,
                  name: true,
                },
              },
            },
            orderBy: [desc(trials.scheduledAt)],
          },
        },
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check study access
      await checkStudyAccess(ctx.db, userId, participant.studyId);

      return participant;
    }),

  create: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        participantCode: z.string().min(1).max(50),
        email: z.string().email().optional(),
        name: z.string().max(255).optional(),
        demographics: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, input.studyId, [
        "owner",
        "researcher",
      ]);

      // Check if participant code already exists in this study
      const existingParticipant = await ctx.db.query.participants.findFirst({
        where: and(
          eq(participants.studyId, input.studyId),
          eq(participants.participantCode, input.participantCode),
        ),
      });

      if (existingParticipant) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "Participant code already exists in this study",
        });
      }

      // Check if email already exists in this study (if provided)
      if (input.email) {
        const existingEmail = await ctx.db.query.participants.findFirst({
          where: and(
            eq(participants.studyId, input.studyId),
            eq(participants.email, input.email),
          ),
        });

        if (existingEmail) {
          throw new TRPCError({
            code: "CONFLICT",
            message: "Email already registered for this study",
          });
        }
      }

      const [newParticipant] = await ctx.db
        .insert(participants)
        .values({
          studyId: input.studyId,
          participantCode: input.participantCode,
          email: input.email,
          name: input.name,
          demographics: input.demographics ?? {},
        })
        .returning();

      if (!newParticipant) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create participant",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: input.studyId,
        userId,
        action: "participant_created",
        description: `Created participant "${input.participantCode}"`,
        resourceType: "participant",
        resourceId: newParticipant.id,
      });

      return newParticipant;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        participantCode: z.string().min(1).max(50).optional(),
        email: z.string().email().optional(),
        name: z.string().max(255).optional(),
        demographics: z.any().optional(),
        notes: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const userId = ctx.session.user.id;

      // Get participant to check study access
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, id),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, participant.studyId, [
        "owner",
        "researcher",
      ]);

      // Check if participant code already exists (if being updated)
      if (
        updateData.participantCode &&
        updateData.participantCode !== participant.participantCode
      ) {
        const existingParticipant = await ctx.db.query.participants.findFirst({
          where: and(
            eq(participants.studyId, participant.studyId),
            eq(participants.participantCode, updateData.participantCode),
          ),
        });

        if (existingParticipant) {
          throw new TRPCError({
            code: "CONFLICT",
            message: "Participant code already exists in this study",
          });
        }
      }

      // Check if email already exists (if being updated)
      if (updateData.email && updateData.email !== participant.email) {
        const existingEmail = await ctx.db.query.participants.findFirst({
          where: and(
            eq(participants.studyId, participant.studyId),
            eq(participants.email, updateData.email),
          ),
        });

        if (existingEmail) {
          throw new TRPCError({
            code: "CONFLICT",
            message: "Email already registered for this study",
          });
        }
      }

      const [updatedParticipant] = await ctx.db
        .update(participants)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(participants.id, id))
        .returning();

      if (!updatedParticipant) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update participant",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: participant.studyId,
        userId,
        action: "participant_updated",
        description: `Updated participant "${participant.participantCode}"`,
        resourceType: "participant",
        resourceId: id,
      });

      return updatedParticipant;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get participant to check study access
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
        with: {
          trials: {
            columns: {
              id: true,
            },
          },
        },
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, participant.studyId, [
        "owner",
        "researcher",
      ]);

      // Check if participant has any trials
      if (participant.trials.length > 0) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message:
            "Cannot delete participant with existing trials. Archive the participant instead.",
        });
      }

      // Delete participant (this will cascade to consent records)
      await ctx.db.delete(participants).where(eq(participants.id, input.id));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: participant.studyId,
        userId,
        action: "participant_deleted",
        description: `Deleted participant "${participant.participantCode}"`,
        resourceType: "participant",
        resourceId: input.id,
      });

      return { success: true };
    }),

  getConsentUploadUrl: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        participantId: z.string().uuid(),
        filename: z.string(),
        contentType: z.string(),
        size: z.number().max(10 * 1024 * 1024), // 10MB limit
      })
    )
    .mutation(async ({ ctx, input }) => {
      const { studyId, participantId, filename, contentType, size } = input;
      const userId = ctx.session.user.id;

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, studyId, ["owner", "researcher", "wizard"]);

      // Validate file type
      const allowedTypes = ["pdf", "png", "jpg", "jpeg"];
      const validation = validateFile(filename, size, allowedTypes);
      if (!validation.valid) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: validation.error,
        });
      }

      // Generate key: studies/{studyId}/participants/{participantId}/consent/{timestamp}-{filename}
      const key = `studies/${studyId}/participants/${participantId}/consent/${Date.now()}-${filename.replace(/[^a-zA-Z0-9.-]/g, "_")}`;

      // Generate presigned URL
      const url = await getUploadUrl(key, contentType);

      return { url, key };
    }),

  recordConsent: protectedProcedure
    .input(
      z.object({
        participantId: z.string().uuid(),
        consentFormId: z.string().uuid(),
        signatureData: z.string().optional(),
        ipAddress: z.string().optional(),
        storagePath: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { participantId, consentFormId, signatureData, ipAddress, storagePath } = input;
      const userId = ctx.session.user.id;

      // Get participant to check study access
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, participantId),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check study access with researcher/wizard permission
      await checkStudyAccess(ctx.db, userId, participant.studyId, [
        "owner",
        "researcher",
        "wizard",
      ]);

      // Verify consent form exists and belongs to the study
      const consentForm = await ctx.db.query.consentForms.findFirst({
        where: eq(consentForms.id, consentFormId),
      });

      if (!consentForm) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Consent form not found",
        });
      }

      if (consentForm.studyId !== participant.studyId) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Consent form doesn't belong to this study",
        });
      }

      // Check if consent already exists
      const existingConsent = await ctx.db.query.participantConsents.findFirst({
        where: and(
          eq(participantConsents.participantId, participantId),
          eq(participantConsents.consentFormId, consentFormId),
        ),
      });

      if (existingConsent) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "Consent already recorded for this form",
        });
      }

      // Record consent
      const [newConsent] = await ctx.db
        .insert(participantConsents)
        .values({
          participantId,
          consentFormId,
          signatureData,
          ipAddress,
          storagePath,
        })
        .returning();

      if (!newConsent) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to record consent",
        });
      }

      // Update participant consent status
      await ctx.db
        .update(participants)
        .set({
          consentGiven: true,
          consentDate: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(participants.id, participantId));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: participant.studyId,
        userId,
        action: "consent_recorded",
        description: `Recorded consent for participant "${participant.participantCode}"`,
        resourceType: "participant",
        resourceId: participantId,
      });

      return newConsent;
    }),

  revokeConsent: protectedProcedure
    .input(
      z.object({
        participantId: z.string().uuid(),
        consentFormId: z.string().uuid(),
        reason: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { participantId, consentFormId, reason } = input;
      const userId = ctx.session.user.id;

      // Get participant to check study access
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, participantId),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, participant.studyId, [
        "owner",
        "researcher",
      ]);

      // Check if consent exists
      const existingConsent = await ctx.db.query.participantConsents.findFirst({
        where: and(
          eq(participantConsents.participantId, participantId),
          eq(participantConsents.consentFormId, consentFormId),
        ),
      });

      if (!existingConsent) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Consent record not found",
        });
      }

      // Remove consent record
      await ctx.db
        .delete(participantConsents)
        .where(eq(participantConsents.id, existingConsent.id));

      // Check if participant has any other consents
      const remainingConsents = await ctx.db.query.participantConsents.findMany(
        {
          where: eq(participantConsents.participantId, participantId),
        },
      );

      // Update participant consent status if no consents remain
      if (remainingConsents.length === 0) {
        await ctx.db
          .update(participants)
          .set({
            consentGiven: false,
            consentDate: null,
            updatedAt: new Date(),
          })
          .where(eq(participants.id, participantId));
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: participant.studyId,
        userId,
        action: "consent_revoked",
        description: `Revoked consent for participant "${participant.participantCode}"${reason ? ` - Reason: ${reason}` : ""}`,
        resourceType: "participant",
        resourceId: participantId,
      });

      return { success: true };
    }),

  getConsentForms: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check study access
      await checkStudyAccess(ctx.db, userId, input.studyId);

      const forms = await ctx.db.query.consentForms.findMany({
        where: eq(consentForms.studyId, input.studyId),
        with: {
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
        },
        orderBy: [desc(consentForms.createdAt)],
      });

      return forms;
    }),

  getUserParticipants: protectedProcedure
    .input(
      z.object({
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        search: z.string().optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { page, limit, search } = input;
      const offset = (page - 1) * limit;
      const userId = ctx.session.user.id;

      // Get all studies user is a member of
      const userStudies = await ctx.db.query.studyMembers.findMany({
        where: eq(studyMembers.userId, userId),
        columns: {
          studyId: true,
        },
      });

      const studyIds = userStudies.map((membership) => membership.studyId);

      if (studyIds.length === 0) {
        return {
          participants: [],
          pagination: {
            page,
            limit,
            total: 0,
            pages: 0,
          },
        };
      }

      // Build where conditions
      const conditions = [inArray(participants.studyId, studyIds)];

      if (search) {
        conditions.push(
          or(
            ilike(participants.participantCode, `%${search}%`),
            ilike(participants.name, `%${search}%`),
            ilike(participants.email, `%${search}%`),
          )!,
        );
      }

      const whereClause = and(...conditions);

      // Get participants
      const userParticipants = await ctx.db.query.participants.findMany({
        where: whereClause,
        with: {
          study: {
            columns: {
              id: true,
              name: true,
            },
          },
        },
        limit,
        offset,
        orderBy: [desc(participants.createdAt)],
      });

      // Get total count
      const totalCountResult = await ctx.db
        .select({ count: count() })
        .from(participants)
        .where(whereClause);

      const totalCount = totalCountResult[0]?.count ?? 0;

      return {
        participants: userParticipants,
        pagination: {
          page,
          limit,
          total: totalCount,
          pages: Math.ceil(totalCount / limit),
        },
      };
    }),
});
