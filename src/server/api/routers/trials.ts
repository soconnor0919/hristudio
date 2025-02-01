import { z } from "zod";
import { eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { trials, experiments, participants, trialStatusEnum } from "~/server/db/schema";
import { checkPermissions } from "~/lib/permissions/server";
import { PERMISSIONS } from "~/lib/permissions/constants";

const createTrialSchema = z.object({
  experimentId: z.string().uuid(),
  participantId: z.string().uuid(),
  startedAt: z.date().optional(),
  completedAt: z.date().optional(),
  status: z.enum(trialStatusEnum.enumValues).default("scheduled"),
  data: z.record(z.unknown()).optional()
});

export const trialRouter = createTRPCRouter({
  getByExperiment: protectedProcedure
    .input(z.object({ experimentId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId)
      });

      if (!experiment) throw new TRPCError({ code: "NOT_FOUND" });

      await checkPermissions({
        studyId: experiment.studyId,
        permission: PERMISSIONS.VIEW_EXPERIMENT
      });

      return ctx.db.query.trials.findMany({
        where: eq(trials.experimentId, input.experimentId),
        with: {
          participant: true
        }
      });
    }),

  getById: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const trial = await ctx.db.query.trials.findFirst({
        where: eq(trials.id, input.id),
        with: {
          experiment: {
            with: {
              study: true
            }
          },
          participant: true
        }
      });

      if (!trial) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found"
        });
      }

      await checkPermissions({
        studyId: trial.experiment.studyId,
        permission: PERMISSIONS.VIEW_EXPERIMENT
      });

      return trial;
    }),

  create: protectedProcedure
    .input(createTrialSchema)
    .mutation(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId)
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found"
        });
      }

      await checkPermissions({
        studyId: experiment.studyId,
        permission: PERMISSIONS.CONDUCT_TRIALS
      });

      const [trial] = await ctx.db
        .insert(trials)
        .values(input)
        .returning();

      return trial;
    }),

  update: protectedProcedure
    .input(z.object({
      id: z.string().uuid(),
      ...createTrialSchema.partial().shape
    }))
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input;

      const trial = await ctx.db.query.trials.findFirst({
        where: eq(trials.id, id),
        with: {
          experiment: true
        }
      });

      if (!trial) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found"
        });
      }

      await checkPermissions({
        studyId: trial.experiment.studyId,
        permission: PERMISSIONS.CONDUCT_TRIALS
      });

      const [updated] = await ctx.db
        .update(trials)
        .set({
          ...data,
          updatedAt: new Date()
        })
        .where(eq(trials.id, id))
        .returning();

      return updated;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const trial = await ctx.db.query.trials.findFirst({
        where: eq(trials.id, input.id),
        with: {
          experiment: true
        }
      });

      if (!trial) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found"
        });
      }

      await checkPermissions({
        studyId: trial.experiment.studyId,
        permission: PERMISSIONS.CONDUCT_TRIALS
      });

      await ctx.db.delete(trials).where(eq(trials.id, input.id));

      return { success: true };
    })
}); 