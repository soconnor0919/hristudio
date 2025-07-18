import { z } from "zod";
import { eq, and, desc, asc, gte, lte, inArray, type SQL } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import {
  trials,
  trialEvents,
  wizardInterventions,
  participants,
  experiments,
  studyMembers,
  trialStatusEnum,
} from "~/server/db/schema";
import type { db } from "~/server/db";

// Helper function to check if user has access to trial
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

export const trialsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        studyId: z.string().optional(),
        experimentId: z.string().optional(),
        participantId: z.string().optional(),
        status: z.enum(trialStatusEnum.enumValues).optional(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Build query conditions
      const conditions: SQL[] = [];

      if (input.studyId) {
        conditions.push(eq(experiments.studyId, input.studyId));
      }
      if (input.experimentId) {
        conditions.push(eq(trials.experimentId, input.experimentId));
      }
      if (input.participantId) {
        conditions.push(eq(trials.participantId, input.participantId));
      }
      if (input.status) {
        conditions.push(eq(trials.status, input.status));
      }

      const query = db
        .select({
          id: trials.id,
          participantId: trials.participantId,
          experimentId: trials.experimentId,
          status: trials.status,
          startedAt: trials.startedAt,
          completedAt: trials.completedAt,
          duration: trials.duration,
          notes: trials.notes,
          createdAt: trials.createdAt,
          updatedAt: trials.updatedAt,
          experiment: {
            id: experiments.id,
            name: experiments.name,
            studyId: experiments.studyId,
          },
          participant: {
            id: participants.id,
            participantCode: participants.participantCode,
          },
        })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .innerJoin(participants, eq(trials.participantId, participants.id))
        .innerJoin(studyMembers, eq(studyMembers.studyId, experiments.studyId))
        .where(
          and(
            eq(studyMembers.userId, userId),
            inArray(studyMembers.role, ["owner", "researcher", "wizard"]),
            ...conditions,
          ),
        )
        .orderBy(desc(trials.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return await query;
    }),

  get: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.id);

      const trial = await db
        .select({
          id: trials.id,
          participantId: trials.participantId,
          experimentId: trials.experimentId,
          status: trials.status,
          startedAt: trials.startedAt,
          completedAt: trials.completedAt,
          duration: trials.duration,
          notes: trials.notes,
          metadata: trials.metadata,
          createdAt: trials.createdAt,
          updatedAt: trials.updatedAt,
          experiment: {
            id: experiments.id,
            name: experiments.name,
            description: experiments.description,
            studyId: experiments.studyId,
          },
          participant: {
            id: participants.id,
            participantCode: participants.participantCode,
            demographics: participants.demographics,
          },
        })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .innerJoin(participants, eq(trials.participantId, participants.id))
        .where(eq(trials.id, input.id))
        .limit(1);

      if (!trial[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found",
        });
      }

      return trial[0];
    }),

  create: protectedProcedure
    .input(
      z.object({
        participantId: z.string(),
        experimentId: z.string(),
        notes: z.string().optional(),
        metadata: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check if experiment exists and user has access
      const experiment = await db
        .select({
          id: experiments.id,
          studyId: experiments.studyId,
        })
        .from(experiments)
        .where(eq(experiments.id, input.experimentId))
        .limit(1);

      if (!experiment[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check user access
      const membership = await db
        .select()
        .from(studyMembers)
        .where(
          and(
            eq(studyMembers.studyId, experiment[0].studyId),
            eq(studyMembers.userId, userId),
            inArray(studyMembers.role, ["owner", "researcher"]),
          ),
        )
        .limit(1);

      if (!membership[0]) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Insufficient permissions to create trial",
        });
      }

      // Check if participant exists
      const participant = await db
        .select()
        .from(participants)
        .where(eq(participants.id, input.participantId))
        .limit(1);

      if (!participant[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Create trial
      const [trial] = await db
        .insert(trials)
        .values({
          participantId: input.participantId,
          experimentId: input.experimentId,
          status: "scheduled",
          notes: input.notes,
          metadata: input.metadata,
        })
        .returning();

      return trial;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string(),
        notes: z.string().optional(),
        metadata: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.id);

      const [trial] = await db
        .update(trials)
        .set({
          notes: input.notes,
          metadata: input.metadata,
          updatedAt: new Date(),
        })
        .where(eq(trials.id, input.id))
        .returning();

      return trial;
    }),

  start: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.id, [
        "owner",
        "researcher",
        "wizard",
      ]);

      // Get current trial status
      const currentTrial = await db
        .select()
        .from(trials)
        .where(eq(trials.id, input.id))
        .limit(1);

      if (!currentTrial[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found",
        });
      }

      if (currentTrial[0].status !== "scheduled") {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Trial can only be started from scheduled status",
        });
      }

      // Start trial
      const [trial] = await db
        .update(trials)
        .set({
          status: "in_progress",
          startedAt: new Date(),
        })
        .where(eq(trials.id, input.id))
        .returning();

      // Log trial start event
      await db.insert(trialEvents).values({
        trialId: input.id,
        eventType: "trial_started",
        timestamp: new Date(),
        data: { userId },
      });

      return trial;
    }),

  complete: protectedProcedure
    .input(
      z.object({
        id: z.string(),
        notes: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.id, [
        "owner",
        "researcher",
        "wizard",
      ]);

      const [trial] = await db
        .update(trials)
        .set({
          status: "completed",
          completedAt: new Date(),
          notes: input.notes,
        })
        .where(eq(trials.id, input.id))
        .returning();

      // Log trial completion event
      await db.insert(trialEvents).values({
        trialId: input.id,
        eventType: "trial_completed",
        timestamp: new Date(),
        data: { userId, notes: input.notes },
      });

      return trial;
    }),

  abort: protectedProcedure
    .input(
      z.object({
        id: z.string(),
        reason: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.id, [
        "owner",
        "researcher",
        "wizard",
      ]);

      const [trial] = await db
        .update(trials)
        .set({
          status: "aborted",
          completedAt: new Date(),
        })
        .where(eq(trials.id, input.id))
        .returning();

      // Log trial abort event
      await db.insert(trialEvents).values({
        trialId: input.id,
        eventType: "trial_aborted",
        timestamp: new Date(),
        data: { userId, reason: input.reason },
      });

      return trial;
    }),

  logEvent: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        type: z.string(),
        data: z.any().optional(),
        timestamp: z.date().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.trialId, [
        "owner",
        "researcher",
        "wizard",
      ]);

      const [event] = await db
        .insert(trialEvents)
        .values({
          trialId: input.trialId,
          eventType: input.type,
          timestamp: input.timestamp ?? new Date(),
          data: input.data,
        })
        .returning();

      return event;
    }),

  addIntervention: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        type: z.string(),
        description: z.string(),
        timestamp: z.date().optional(),
        data: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.trialId, [
        "owner",
        "researcher",
        "wizard",
      ]);

      const [intervention] = await db
        .insert(wizardInterventions)
        .values({
          trialId: input.trialId,
          wizardId: userId,
          interventionType: input.type,
          description: input.description,
          timestamp: input.timestamp ?? new Date(),
          parameters: input.data,
        })
        .returning();

      return intervention;
    }),

  getEvents: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        type: z.string().optional(),
        startTime: z.date().optional(),
        endTime: z.date().optional(),
        limit: z.number().min(1).max(1000).default(100),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      await checkTrialAccess(db, userId, input.trialId);

      const conditions = [eq(trialEvents.trialId, input.trialId)];

      if (input.type) {
        conditions.push(eq(trialEvents.eventType, input.type));
      }
      if (input.startTime) {
        conditions.push(gte(trialEvents.timestamp, input.startTime));
      }
      if (input.endTime) {
        conditions.push(lte(trialEvents.timestamp, input.endTime));
      }

      const events = await db
        .select()
        .from(trialEvents)
        .where(and(...conditions))
        .orderBy(asc(trialEvents.timestamp))
        .limit(input.limit)
        .offset(input.offset);

      return events;
    }),
});
