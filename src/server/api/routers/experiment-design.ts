import { z } from "zod";
import { eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { experiments, steps, actions, actionTypeEnum, stepTypeEnum } from "~/server/db/schema";
import { checkPermissions } from "~/lib/permissions/server";
import { PERMISSIONS } from "~/lib/permissions/constants";
import type { InferSelectModel } from "drizzle-orm";

type Step = InferSelectModel<typeof steps>;
type Action = InferSelectModel<typeof actions>;

const createStepSchema = z.object({
  experimentId: z.string().uuid(),
  title: z.string().min(1).max(256),
  description: z.string().optional(),
  type: z.enum(stepTypeEnum.enumValues),
  order: z.number().int().min(0),
  config: z.record(z.unknown()).optional(),
});

const createActionSchema = z.object({
  stepId: z.string().uuid(),
  type: z.enum(actionTypeEnum.enumValues),
  parameters: z.record(z.unknown()).optional(),
  order: z.number().int().min(0),
});

export const experimentDesignRouter = createTRPCRouter({
  getSteps: protectedProcedure
    .input(z.object({ experimentId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      await checkPermissions({
        studyId: experiment.studyId,
        permission: PERMISSIONS.VIEW_EXPERIMENT,
      });

      const dbSteps = await ctx.db.query.steps.findMany({
        where: eq(steps.experimentId, input.experimentId),
        with: {
          actions: {
            orderBy: (actions, { asc }) => [asc(actions.order)],
          },
        },
        orderBy: (steps, { asc }) => [asc(steps.order)],
      });

      return dbSteps.map(step => ({
        ...step,
        config: step.config ? JSON.parse(step.config) : null,
        actions: step.actions.map(action => ({
          ...action,
          parameters: action.parameters ? JSON.parse(action.parameters) : null,
        })),
      }));
    }),

  createStep: protectedProcedure
    .input(createStepSchema)
    .mutation(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      await checkPermissions({
        studyId: experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      const [step] = await ctx.db
        .insert(steps)
        .values({
          experimentId: input.experimentId,
          title: input.title,
          description: input.description,
          type: input.type,
          order: input.order,
          config: input.config ? JSON.stringify(input.config) : null,
        })
        .returning();

      if (!step) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create step",
        });
      }

      return {
        ...step,
        config: step.config ? JSON.parse(step.config) : null,
      };
    }),

  updateStep: protectedProcedure
    .input(z.object({
      id: z.string().uuid(),
      ...createStepSchema.partial().shape,
    }))
    .mutation(async ({ ctx, input }) => {
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, input.id),
        with: {
          experiment: true,
        },
      });

      if (!step) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      await checkPermissions({
        studyId: step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      const { id, config, ...rest } = input;
      const [updated] = await ctx.db
        .update(steps)
        .set({
          ...rest,
          config: config ? JSON.stringify(config) : undefined,
          updatedAt: new Date(),
        })
        .where(eq(steps.id, id))
        .returning();

      if (!updated) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update step",
        });
      }

      return {
        ...updated,
        config: updated.config ? JSON.parse(updated.config) : null,
      };
    }),

  deleteStep: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, input.id),
        with: {
          experiment: true,
        },
      });

      if (!step) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      await checkPermissions({
        studyId: step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      await ctx.db.delete(steps).where(eq(steps.id, input.id));

      return { success: true };
    }),

  createAction: protectedProcedure
    .input(createActionSchema)
    .mutation(async ({ ctx, input }) => {
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, input.stepId),
        with: {
          experiment: true,
        },
      });

      if (!step) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      await checkPermissions({
        studyId: step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      const [action] = await ctx.db
        .insert(actions)
        .values({
          stepId: input.stepId,
          type: input.type,
          parameters: input.parameters ? JSON.stringify(input.parameters) : null,
          order: input.order,
        })
        .returning();

      if (!action) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create action",
        });
      }

      return {
        ...action,
        parameters: action.parameters ? JSON.parse(action.parameters) : null,
      };
    }),

  updateAction: protectedProcedure
    .input(z.object({
      id: z.string().uuid(),
      ...createActionSchema.partial().shape,
    }))
    .mutation(async ({ ctx, input }) => {
      const action = await ctx.db.query.actions.findFirst({
        where: eq(actions.id, input.id),
        with: {
          step: {
            with: {
              experiment: true,
            },
          },
        },
      });

      if (!action) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Action not found",
        });
      }

      await checkPermissions({
        studyId: action.step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      const { id, parameters, ...rest } = input;
      const [updated] = await ctx.db
        .update(actions)
        .set({
          ...rest,
          parameters: parameters ? JSON.stringify(parameters) : undefined,
          updatedAt: new Date(),
        })
        .where(eq(actions.id, id))
        .returning();

      if (!updated) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update action",
        });
      }

      return {
        ...updated,
        parameters: updated.parameters ? JSON.parse(updated.parameters) : null,
      };
    }),

  deleteAction: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const action = await ctx.db.query.actions.findFirst({
        where: eq(actions.id, input.id),
        with: {
          step: {
            with: {
              experiment: true,
            },
          },
        },
      });

      if (!action) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Action not found",
        });
      }

      await checkPermissions({
        studyId: action.step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      await ctx.db.delete(actions).where(eq(actions.id, input.id));

      return { success: true };
    }),

  updateStepOrder: protectedProcedure
    .input(z.object({
      experimentId: z.string().uuid(),
      stepIds: z.array(z.string().uuid()),
    }))
    .mutation(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      await checkPermissions({
        studyId: experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      return ctx.db.transaction(async (tx) => {
        await tx
          .update(steps)
          .set({ order: -1 })
          .where(eq(steps.experimentId, input.experimentId));

        for (const [index, id] of input.stepIds.entries()) {
          await tx
            .update(steps)
            .set({ order: index + 1 })
            .where(eq(steps.id, id));
        }
      });
    }),

  updateActionOrder: protectedProcedure
    .input(z.object({
      stepId: z.string().uuid(),
      actionIds: z.array(z.string().uuid()),
    }))
    .mutation(async ({ ctx, input }) => {
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, input.stepId),
        with: {
          experiment: true,
        },
      });

      if (!step) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      await checkPermissions({
        studyId: step.experiment.studyId,
        permission: PERMISSIONS.MANAGE_EXPERIMENTS,
      });

      return ctx.db.transaction(async (tx) => {
        await tx
          .update(actions)
          .set({ order: -1 })
          .where(eq(actions.stepId, input.stepId));

        for (const [index, id] of input.actionIds.entries()) {
          await tx
            .update(actions)
            .set({ order: index + 1 })
            .where(eq(actions.id, id));
        }
      });
    }),
}); 