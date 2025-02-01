import { z } from "zod";
import { eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { experiments, steps, studyMembers } from "~/server/db/schema";

const createStepSchema = z.object({
  experimentId: z.string().uuid(),
  title: z.string().min(1).max(256),
  description: z.string().optional(),
  type: z.enum(["instruction", "interaction", "survey"]),
  config: z.string().optional(), // JSON string
  order: z.number().int().min(0),
});

export const stepRouter = createTRPCRouter({
  create: protectedProcedure
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      if (membership.role !== "admin" && membership.role !== "principal_investigator") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have permission to create steps",
        });
      }

      const [step] = await ctx.db
        .insert(steps)
        .values(input)
        .returning();

      if (!step) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create step",
        });
      }

      return step;
    }),

  list: protectedProcedure
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      const stepList = await ctx.db.query.steps.findMany({
        where: eq(steps.experimentId, input.experimentId),
        orderBy: (steps, { asc }) => [asc(steps.order)],
        with: {
          actions: {
            orderBy: (actions, { asc }) => [asc(actions.order)],
          },
        },
      });

      return stepList.map((step) => ({
        ...step,
        config: step.config ? JSON.parse(step.config) : null,
      }));
    }),

  byId: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, input.id),
        with: {
          experiment: {
            with: {
              study: true,
            },
          },
          actions: {
            orderBy: (actions, { asc }) => [asc(actions.order)],
          },
        },
      });

      if (!step) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, step.experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      return {
        ...step,
        config: step.config ? JSON.parse(step.config) : null,
        role: membership.role,
      };
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        title: z.string().min(1).max(256).optional(),
        description: z.string().optional(),
        type: z.enum(["instruction", "interaction", "survey"]).optional(),
        config: z.string().optional(), // JSON string
        order: z.number().int().min(0).optional(),
      })
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input;

      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, id),
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, step.experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      if (membership.role !== "admin" && membership.role !== "principal_investigator") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have permission to update this step",
        });
      }

      const [updated] = await ctx.db
        .update(steps)
        .set(data)
        .where(eq(steps.id, id))
        .returning();

      if (!updated) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      return {
        ...updated,
        config: updated.config ? JSON.parse(updated.config) : null,
        role: membership.role,
      };
    }),

  delete: protectedProcedure
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, step.experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      if (membership.role !== "admin" && membership.role !== "principal_investigator") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have permission to delete this step",
        });
      }

      const [deleted] = await ctx.db
        .delete(steps)
        .where(eq(steps.id, input.id))
        .returning();

      if (!deleted) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Step not found",
        });
      }

      return deleted;
    }),
}); 