import { z } from "zod";
import { eq } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { actions, steps, studyMembers } from "~/server/db/schema";

const createActionSchema = z.object({
  stepId: z.string().uuid(),
  type: z.enum(["movement", "speech", "wait", "data_collection"]),
  parameters: z.string().optional(), // JSON string
  order: z.number().int().min(0),
});

export const actionRouter = createTRPCRouter({
  create: protectedProcedure
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
          message: "You don't have permission to create actions",
        });
      }

      const [action] = await ctx.db
        .insert(actions)
        .values(input)
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

  list: protectedProcedure
    .input(z.object({ stepId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, step.experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      const actionList = await ctx.db.query.actions.findMany({
        where: eq(actions.stepId, input.stepId),
        orderBy: (actions, { asc }) => [asc(actions.order)],
      });

      return actionList.map((action) => ({
        ...action,
        parameters: action.parameters ? JSON.parse(action.parameters) : null,
      }));
    }),

  byId: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const action = await ctx.db.query.actions.findFirst({
        where: eq(actions.id, input.id),
        with: {
          step: {
            with: {
              experiment: {
                with: {
                  study: true,
                },
              },
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, action.step.experiment.studyId),
      });

      if (!membership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      return {
        ...action,
        parameters: action.parameters ? JSON.parse(action.parameters) : null,
        role: membership.role,
      };
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        type: z.enum(["movement", "speech", "wait", "data_collection"]).optional(),
        parameters: z.string().optional(), // JSON string
        order: z.number().int().min(0).optional(),
      })
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input;

      const action = await ctx.db.query.actions.findFirst({
        where: eq(actions.id, id),
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, action.step.experiment.studyId),
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
          message: "You don't have permission to update this action",
        });
      }

      const [updated] = await ctx.db
        .update(actions)
        .set(data)
        .where(eq(actions.id, id))
        .returning();

      if (!updated) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Action not found",
        });
      }

      return {
        ...updated,
        parameters: updated.parameters ? JSON.parse(updated.parameters) : null,
        role: membership.role,
      };
    }),

  delete: protectedProcedure
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

      const membership = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.studyId, action.step.experiment.studyId),
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
          message: "You don't have permission to delete this action",
        });
      }

      const [deleted] = await ctx.db
        .delete(actions)
        .where(eq(actions.id, input.id))
        .returning();

      if (!deleted) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Action not found",
        });
      }

      return deleted;
    }),
}); 