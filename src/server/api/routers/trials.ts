import { TRPCError } from "@trpc/server";
import {
  and,
  asc,
  count,
  desc,
  eq,
  gte,
  inArray,
  lte,
  sql,
  type SQL,
} from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { db } from "~/server/db";
import {
  experiments,
  participants,
  studyMembers,
  trialEvents,
  trials,
  trialStatusEnum,
  wizardInterventions,
  mediaCaptures,
  users,
} from "~/server/db/schema";
import {
  TrialExecutionEngine,
  type ActionDefinition,
} from "~/server/services/trial-execution";

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

// Lazy-initialized execution engine instance
function getExecutionEngine() {
  return new TrialExecutionEngine(db);
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
          sessionNumber: trials.sessionNumber,
          scheduledAt: trials.scheduledAt,
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
          wizard: {
            id: users.id,
            name: users.name,
            email: users.email,
          },
          userRole: studyMembers.role,
        })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .innerJoin(participants, eq(trials.participantId, participants.id))
        .leftJoin(users, eq(users.id, trials.wizardId))
        .innerJoin(studyMembers, eq(studyMembers.studyId, experiments.studyId))
        .where(and(eq(studyMembers.userId, userId), ...conditions))
        .orderBy(desc(trials.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      const results = await query;

      // Aggregate event & media counts (batched)
      const trialIds = results.map((r) => r.id);
      const eventCountMap = new Map<string, number>();
      const mediaCountMap = new Map<string, number>();
      const latestEventAtMap = new Map<string, Date>();
      // Hoisted map for latest event timestamps so it is in scope after aggregation block
      // (removed redeclaration of latestEventAtMap; now hoisted above)

      if (trialIds.length > 0) {
        const eventCounts = await db
          .select({
            trialId: trialEvents.trialId,
            count: count(),
            latest: sql`max(${trialEvents.timestamp})`.as("latest"),
          })
          .from(trialEvents)
          .where(inArray(trialEvents.trialId, trialIds))
          .groupBy(trialEvents.trialId);

        eventCounts.forEach((ec) => {
          eventCountMap.set(ec.trialId, Number(ec.count) || 0);
          if (ec.latest) {
            latestEventAtMap.set(ec.trialId, ec.latest as Date);
          }
        });

        const mediaCounts = await db
          .select({
            trialId: mediaCaptures.trialId,
            count: count(),
          })
          .from(mediaCaptures)
          .where(inArray(mediaCaptures.trialId, trialIds))
          .groupBy(mediaCaptures.trialId);

        mediaCounts.forEach((mc) => {
          mediaCountMap.set(mc.trialId, Number(mc.count) || 0);
        });
      }

      // Add permission flags & counts
      return results.map((trial) => ({
        ...trial,
        eventCount: eventCountMap.get(trial.id) ?? 0,
        mediaCount: mediaCountMap.get(trial.id) ?? 0,
        latestEventAt: latestEventAtMap.get(trial.id) ?? null,
        canAccess: ["owner", "researcher", "wizard"].includes(trial.userRole),
      }));
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
          wizardId: trials.wizardId,
          sessionNumber: trials.sessionNumber,
          status: trials.status,
          scheduledAt: trials.scheduledAt,
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
        scheduledAt: z.date().optional(),
        wizardId: z.string().optional(),
        sessionNumber: z.number().optional(),
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
          scheduledAt: input.scheduledAt,
          wizardId: input.wizardId,
          sessionNumber: input.sessionNumber ?? 1,
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
        scheduledAt: z.date().optional(),
        wizardId: z.string().optional(),
        sessionNumber: z.number().optional(),
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
          scheduledAt: input.scheduledAt,
          wizardId: input.wizardId,
          sessionNumber: input.sessionNumber,
          notes: input.notes,
          metadata: input.metadata,
          updatedAt: sql`CURRENT_TIMESTAMP`,
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

      // Use execution engine to start trial
      const executionEngine = getExecutionEngine();
      const result = await executionEngine.startTrial(input.id, userId);

      if (!result.success) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: result.error ?? "Failed to start trial",
        });
      }

      // Return updated trial data
      const trial = await db
        .select()
        .from(trials)
        .where(eq(trials.id, input.id))
        .limit(1);

      if (!trial[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found after start",
        });
      }

      return trial[0];
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

      // Use execution engine to abort trial
      const executionEngine = getExecutionEngine();
      const result = await executionEngine.abortTrial(input.id, input.reason);

      if (!result.success) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: result.error ?? "Failed to complete trial",
        });
      }

      // Return updated trial data
      const trial = await db
        .select()
        .from(trials)
        .where(eq(trials.id, input.id))
        .limit(1);

      if (!trial[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found after abort",
        });
      }

      return trial[0];
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

  getUserTrials: protectedProcedure
    .input(
      z.object({
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        status: z.enum(trialStatusEnum.enumValues).optional(),
        studyId: z.string().optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { page, limit, status, studyId } = input;
      const offset = (page - 1) * limit;
      const userId = ctx.session.user.id;

      // Get all studies user is a member of with roles
      const userStudies = await ctx.db.query.studyMembers.findMany({
        where: eq(studyMembers.userId, userId),
        columns: {
          studyId: true,
          role: true,
        },
      });

      let studyIds = userStudies.map((membership) => membership.studyId);
      const userStudyRoles = new Map(
        userStudies.map((membership) => [membership.studyId, membership.role]),
      );

      // If studyId is provided, filter to just that study (if user has access)
      if (studyId) {
        if (!studyIds.includes(studyId)) {
          throw new TRPCError({
            code: "FORBIDDEN",
            message: "You don't have access to this study",
          });
        }
        studyIds = [studyId];
      }

      if (studyIds.length === 0) {
        return {
          trials: [],
          pagination: {
            page,
            limit,
            total: 0,
            pages: 0,
          },
        };
      }

      // Build where conditions with study filtering
      const conditions = [];

      if (status) {
        conditions.push(eq(trials.status, status));
      }

      // Get trials from experiments in user's studies using SQL join
      const userTrials = await ctx.db
        .select({
          trial: trials,
          experiment: {
            id: experiments.id,
            name: experiments.name,
            studyId: experiments.studyId,
          },
        })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .where(
          and(
            inArray(experiments.studyId, studyIds),
            ...(conditions.length > 0 ? conditions : []),
          ),
        )
        .limit(limit)
        .offset(offset)
        .orderBy(desc(trials.scheduledAt));

      // Get full trial data with relations for the filtered trials
      const trialIds = userTrials.map((row) => row.trial.id);

      const filteredTrials =
        trialIds.length > 0
          ? await ctx.db.query.trials.findMany({
              where: inArray(trials.id, trialIds),
              with: {
                experiment: {
                  with: {
                    study: {
                      columns: {
                        id: true,
                        name: true,
                      },
                    },
                  },
                  columns: {
                    id: true,
                    name: true,
                    studyId: true,
                  },
                },
                participant: {
                  columns: {
                    id: true,
                    participantCode: true,
                    email: true,
                    name: true,
                  },
                },
                wizard: {
                  columns: {
                    id: true,
                    name: true,
                    email: true,
                  },
                },
                events: {
                  columns: {
                    id: true,
                  },
                },
                mediaCaptures: {
                  columns: {
                    id: true,
                  },
                },
              },
              orderBy: [desc(trials.scheduledAt)],
            })
          : [];

      // Get total count
      const whereConditions = [inArray(experiments.studyId, studyIds)];
      if (status) {
        whereConditions.push(eq(trials.status, status));
      }

      const totalCountResult = await ctx.db
        .select({ count: count() })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .where(and(...whereConditions));

      const totalCount = totalCountResult[0]?.count ?? 0;

      // Transform data to include counts and permission information
      const transformedTrials = filteredTrials.map((trial) => {
        const userRole = userStudyRoles.get(trial.experiment.studyId);
        const canAccess =
          userRole && ["owner", "researcher", "wizard"].includes(userRole);

        return {
          ...trial,
          _count: {
            events: trial.events?.length ?? 0,
            mediaCaptures: trial.mediaCaptures?.length ?? 0,
          },
          userRole,
          canAccess,
        };
      });

      return {
        trials: transformedTrials,
        pagination: {
          page,
          limit,
          total: totalCount,
          pages: Math.ceil(totalCount / limit),
        },
      };
    }),
  // Trial Execution Procedures
  executeCurrentStep: protectedProcedure
    .input(z.object({ trialId: z.string() }))
    .mutation(async ({ ctx, input }) => {
      await checkTrialAccess(ctx.db, ctx.session.user.id, input.trialId);

      const executionEngine = getExecutionEngine();
      const result = await executionEngine.executeCurrentStep(input.trialId);

      if (!result.success) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: result.error ?? "Failed to reset trial",
        });
      }

      return result;
    }),

  advanceToNextStep: protectedProcedure
    .input(z.object({ trialId: z.string() }))
    .mutation(async ({ ctx, input }) => {
      await checkTrialAccess(ctx.db, ctx.session.user.id, input.trialId);

      const executionEngine = getExecutionEngine();
      const result = await executionEngine.advanceToNextStep(input.trialId);

      if (!result.success) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: result.error ?? "Failed to advance to next step",
        });
      }

      return result;
    }),

  getExecutionStatus: protectedProcedure
    .input(z.object({ trialId: z.string() }))
    .query(async ({ ctx, input }) => {
      await checkTrialAccess(ctx.db, ctx.session.user.id, input.trialId);

      const executionEngine = getExecutionEngine();
      const status = executionEngine.getTrialStatus(input.trialId);
      const currentStep = executionEngine.getCurrentStep(input.trialId);

      return {
        status,
        currentStep,
      };
    }),

  getCurrentStep: protectedProcedure
    .input(z.object({ trialId: z.string() }))
    .query(async ({ ctx, input }) => {
      await checkTrialAccess(ctx.db, ctx.session.user.id, input.trialId);

      const executionEngine = getExecutionEngine();
      return executionEngine.getCurrentStep(input.trialId);
    }),

  completeWizardAction: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        actionId: z.string(),
        data: z.record(z.string(), z.unknown()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      await checkTrialAccess(ctx.db, ctx.session.user.id, input.trialId);

      // Log wizard action completion
      await ctx.db.insert(trialEvents).values({
        trialId: input.trialId,
        eventType: "wizard_action_completed",
        actionId: input.actionId,
        data: input.data,
        timestamp: new Date(),
        createdBy: ctx.session.user.id,
      });

      return { success: true };
    }),

  executeRobotAction: protectedProcedure
    .input(
      z.object({
        trialId: z.string(),
        pluginName: z.string(),
        actionId: z.string(),
        parameters: z.record(z.string(), z.unknown()).optional().default({}),
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

      // Use execution engine to execute robot action
      const executionEngine = getExecutionEngine();

      // Create action definition for execution
      const actionDefinition: ActionDefinition = {
        id: `${input.pluginName}.${input.actionId}`,
        stepId: "manual", // Manual execution
        name: input.actionId,
        type: `${input.pluginName}.${input.actionId}`,
        orderIndex: 0,
        parameters: input.parameters,
        timeout: 30000,
        required: false,
      };

      const result = await executionEngine.executeAction(
        input.trialId,
        actionDefinition,
      );

      if (!result.success) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: result.error ?? "Robot action execution failed",
        });
      }

      // Log the manual robot action execution
      await db.insert(trialEvents).values({
        trialId: input.trialId,
        eventType: "manual_robot_action",
        actionId: actionDefinition.id,
        data: {
          userId,
          pluginName: input.pluginName,
          actionId: input.actionId,
          parameters: input.parameters,
          result: result.data,
          duration: result.duration,
        },
        timestamp: new Date(),
        createdBy: userId,
      });

      return {
        success: true,
        data: result.data,
        duration: result.duration,
      };
    }),
});
