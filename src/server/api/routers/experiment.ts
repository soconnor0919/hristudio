import { z } from "zod";
import { eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { experiments, studyMembers } from "~/server/db/schema";

const createExperimentSchema = z.object({
  studyId: z.string().uuid(),
  robotId: z.string().uuid(),
  title: z.string().min(1).max(256),
  description: z.string().optional(),
  estimatedDuration: z.number().int().min(0).optional(),
  order: z.number().int().min(0),
});

export const experimentRouter = createTRPCRouter({
  create: protectedProcedure
    .input(createExperimentSchema)
    .mutation(async ({ ctx, input }) => {
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, input.studyId),
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
          message: "You don't have permission to create experiments",
        });
      }

      const [experiment] = await ctx.db
        .insert(experiments)
        .values(input)
        .returning();

      if (!experiment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create experiment",
        });
      }

      return experiment;
    }),

  list: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, input.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      const experimentList = await ctx.db.query.experiments.findMany({
        where: eq(experiments.studyId, input.studyId),
        orderBy: (experiments, { asc }) => [asc(experiments.order)],
        with: {
          robot: true,
        },
      });

      return experimentList;
    }),

  byId: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.id),
        with: {
          robot: true,
          study: true,
        },
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

      return {
        ...experiment,
        role: membership.role,
      };
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        title: z.string().min(1).max(256).optional(),
        description: z.string().optional(),
        estimatedDuration: z.number().int().min(0).optional(),
        order: z.number().int().min(0).optional(),
      })
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input;

      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, id),
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
          message: "You don't have permission to update this experiment",
        });
      }

      const [updated] = await ctx.db
        .update(experiments)
        .set(data)
        .where(eq(experiments.id, id))
        .returning();

      if (!updated) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      return {
        ...updated,
        role: membership.role,
      };
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.id),
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
          message: "You don't have permission to delete this experiment",
        });
      }

      const [deleted] = await ctx.db
        .delete(experiments)
        .where(eq(experiments.id, input.id))
        .returning();

      if (!deleted) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      return deleted;
    }),
}); 