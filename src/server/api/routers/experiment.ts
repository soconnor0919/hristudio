import { z } from "zod";
import { and, eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { experiments, studyMembers } from "~/server/db/schema";
import { type Step } from "~/lib/experiments/types";

const createExperimentSchema = z.object({
  studyId: z.number(),
  title: z.string().min(1, "Title is required"),
  description: z.string().optional(),
  steps: z.array(z.object({
    id: z.string(),
    title: z.string(),
    description: z.string().optional(),
    actions: z.array(z.object({
      id: z.string(),
      type: z.string(),
      parameters: z.record(z.any()),
      order: z.number(),
    })),
    order: z.number(),
  })).default([]),
});

const updateExperimentSchema = z.object({
  id: z.number(),
  title: z.string().min(1, "Title is required"),
  description: z.string().optional(),
  status: z.enum(["draft", "active", "archived"]).optional(),
  steps: z.array(z.object({
    id: z.string(),
    title: z.string(),
    description: z.string().optional(),
    actions: z.array(z.object({
      id: z.string(),
      type: z.string(),
      parameters: z.record(z.any()),
      order: z.number(),
    })),
    order: z.number(),
  })).optional(),
});

export const experimentRouter = createTRPCRouter({
  getByStudyId: protectedProcedure
    .input(z.object({ studyId: z.number() }))
    .query(async ({ ctx, input }) => {
      // Check if user is a member of the study
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to view experiments in this study",
        });
      }

      return ctx.db.query.experiments.findMany({
        where: eq(experiments.studyId, input.studyId),
        orderBy: experiments.createdAt,
      });
    }),

  getById: protectedProcedure
    .input(z.object({ id: z.number() }))
    .query(async ({ ctx, input }) => {
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.id),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check if user has access to the study
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, experiment.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to view this experiment",
        });
      }

      return experiment;
    }),

  create: protectedProcedure
    .input(createExperimentSchema)
    .mutation(async ({ ctx, input }) => {
      // Check if user has permission to create experiments
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || !["owner", "admin", "principal_investigator"]
        .includes(membership.role.toLowerCase())) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to create experiments in this study",
        });
      }

      const [experiment] = await ctx.db
        .insert(experiments)
        .values({
          studyId: input.studyId,
          title: input.title,
          description: input.description,
          steps: input.steps as Step[],
          version: 1,
          status: "draft",
          createdById: ctx.session.user.id,
          createdAt: new Date(),
          updatedAt: new Date(),
        })
        .returning();

      return experiment;
    }),

  update: protectedProcedure
    .input(updateExperimentSchema)
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

      // Check if user has permission to edit experiments
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, experiment.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || !["owner", "admin", "principal_investigator"]
        .includes(membership.role.toLowerCase())) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to edit this experiment",
        });
      }

      const [updatedExperiment] = await ctx.db
        .update(experiments)
        .set({
          title: input.title,
          description: input.description,
          status: input.status,
          steps: input.steps as Step[] | undefined,
          version: experiment.version + 1,
          updatedAt: new Date(),
        })
        .where(eq(experiments.id, input.id))
        .returning();

      return updatedExperiment;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.number() }))
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

      // Check if user has permission to delete experiments
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, experiment.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || !["owner", "admin", "principal_investigator"]
        .includes(membership.role.toLowerCase())) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to delete this experiment",
        });
      }

      await ctx.db
        .delete(experiments)
        .where(eq(experiments.id, input.id));

      return { success: true };
    }),
}); 