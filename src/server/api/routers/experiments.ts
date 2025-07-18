import { z } from "zod";
import { TRPCError } from "@trpc/server";
import { and, eq, desc, asc, inArray } from "drizzle-orm";

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import type { db } from "~/server/db";
import {
  experiments,
  steps,
  actions,
  studyMembers,
  robots,
  activityLogs,
  experimentStatusEnum,
  stepTypeEnum,
} from "~/server/db/schema";

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

export const experimentsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        status: z.enum(experimentStatusEnum.enumValues).optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { studyId, status } = input;
      const userId = ctx.session.user.id;

      // Check study access
      await checkStudyAccess(ctx.db, userId, studyId);

      const conditions = [eq(experiments.studyId, studyId)];
      if (status) {
        conditions.push(eq(experiments.status, status));
      }

      const experimentsList = await ctx.db.query.experiments.findMany({
        where: and(...conditions),
        with: {
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
          robot: {
            columns: {
              id: true,
              name: true,
              manufacturer: true,
            },
          },
          steps: {
            columns: {
              id: true,
              name: true,
              type: true,
              orderIndex: true,
            },
            orderBy: [asc(steps.orderIndex)],
          },
          trials: {
            columns: {
              id: true,
              status: true,
            },
          },
        },
        orderBy: [desc(experiments.updatedAt)],
      });

      return experimentsList.map((exp) => ({
        ...exp,
        stepCount: exp.steps.length,
        trialCount: exp.trials.length,
      }));
    }),

  get: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.id),
        with: {
          study: {
            columns: {
              id: true,
              name: true,
            },
          },
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
          robot: true,
          steps: {
            with: {
              actions: {
                orderBy: [asc(actions.orderIndex)],
              },
            },
            orderBy: [asc(steps.orderIndex)],
          },
        },
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access
      await checkStudyAccess(ctx.db, userId, experiment.studyId);

      return experiment;
    }),

  create: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        name: z.string().min(1).max(255),
        description: z.string().optional(),
        robotId: z.string().uuid().optional(),
        estimatedDuration: z.number().int().min(1).optional(),
        metadata: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, input.studyId, [
        "owner",
        "researcher",
      ]);

      // Validate robot exists if provided
      if (input.robotId) {
        const robot = await ctx.db.query.robots.findFirst({
          where: eq(robots.id, input.robotId),
        });
        if (!robot) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Robot not found",
          });
        }
      }

      const newExperimentResults = await ctx.db
        .insert(experiments)
        .values({
          ...input,
          createdBy: userId,
        })
        .returning();

      const newExperiment = newExperimentResults[0];
      if (!newExperiment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create experiment",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: input.studyId,
        userId,
        action: "experiment_created",
        description: `Created experiment "${newExperiment.name}"`,
        resourceType: "experiment",
        resourceId: newExperiment.id,
      });

      return newExperiment;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        name: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        status: z.enum(experimentStatusEnum.enumValues).optional(),
        estimatedDuration: z.number().int().min(1).optional(),
        metadata: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const userId = ctx.session.user.id;

      // Get experiment to check study access
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, id),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      const updatedExperimentResults = await ctx.db
        .update(experiments)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(experiments.id, id))
        .returning();

      const updatedExperiment = updatedExperimentResults[0];
      if (!updatedExperiment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update experiment",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "experiment_updated",
        description: `Updated experiment "${updatedExperiment.name}"`,
        resourceType: "experiment",
        resourceId: id,
      });

      return updatedExperiment;
    }),

  duplicate: protectedProcedure
    .input(
      z.object({
        experimentId: z.string().uuid(),
        newName: z.string().min(1).max(255),
        includeSteps: z.boolean().default(true),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { experimentId, newName, includeSteps } = input;
      const userId = ctx.session.user.id;

      // Get original experiment
      const originalExperiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, experimentId),
        with: {
          steps: {
            with: {
              actions: {
                orderBy: [asc(actions.orderIndex)],
              },
            },
            orderBy: [asc(steps.orderIndex)],
          },
        },
      });

      if (!originalExperiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, originalExperiment.studyId, [
        "owner",
        "researcher",
      ]);

      // Create duplicate experiment
      const newExperimentResults = await ctx.db
        .insert(experiments)
        .values({
          studyId: originalExperiment.studyId,
          name: newName,
          description: originalExperiment.description,
          robotId: originalExperiment.robotId,
          estimatedDuration: originalExperiment.estimatedDuration,
          metadata: originalExperiment.metadata,
          createdBy: userId,
        })
        .returning();

      const newExperiment = newExperimentResults[0];
      if (!newExperiment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create duplicate experiment",
        });
      }

      // Duplicate steps and actions if requested
      if (includeSteps && originalExperiment.steps.length > 0) {
        for (const step of originalExperiment.steps) {
          const newStepResults = await ctx.db
            .insert(steps)
            .values({
              experimentId: newExperiment.id,
              name: step.name,
              description: step.description,
              type: step.type,
              orderIndex: step.orderIndex,
              durationEstimate: step.durationEstimate,
              required: step.required,
              conditions: step.conditions,
            })
            .returning();

          const newStep = newStepResults[0];
          if (!newStep) {
            throw new TRPCError({
              code: "INTERNAL_SERVER_ERROR",
              message: "Failed to duplicate step",
            });
          }

          // Duplicate actions
          if (step.actions.length > 0) {
            const actionValues = step.actions.map(
              (action: typeof actions.$inferSelect) => ({
                stepId: newStep.id,
                name: action.name,
                description: action.description,
                type: action.type,
                orderIndex: action.orderIndex,
                parameters: action.parameters,
                validationSchema: action.validationSchema,
                timeout: action.timeout,
                retryCount: action.retryCount,
              }),
            );

            await ctx.db.insert(actions).values(actionValues);
          }
        }
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: originalExperiment.studyId,
        userId,
        action: "experiment_duplicated",
        description: `Duplicated experiment "${originalExperiment.name}" as "${newName}"`,
        resourceType: "experiment",
        resourceId: newExperiment.id,
      });

      return newExperiment;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get experiment to check study access
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.id),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      // Soft delete experiment
      await ctx.db
        .update(experiments)
        .set({
          deletedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(experiments.id, input.id));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "experiment_deleted",
        description: `Deleted experiment "${experiment.name}"`,
        resourceType: "experiment",
        resourceId: input.id,
      });

      return { success: true };
    }),

  addStep: protectedProcedure
    .input(
      z.object({
        experimentId: z.string().uuid(),
        name: z.string().min(1).max(255),
        description: z.string().optional(),
        type: z.enum(stepTypeEnum.enumValues),
        orderIndex: z.number().int().min(0),
        durationEstimate: z.number().int().min(0).optional(),
        required: z.boolean().default(true),
        conditions: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { experimentId, ...stepData } = input;
      const userId = ctx.session.user.id;

      // Get experiment to check study access
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, experimentId),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      const newStepResults = await ctx.db
        .insert(steps)
        .values({
          experimentId,
          ...stepData,
        })
        .returning();

      const newStep = newStepResults[0];
      if (!newStep) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create step",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "step_added",
        description: `Added step "${newStep.name}" to experiment "${experiment.name}"`,
        resourceType: "step",
        resourceId: newStep.id,
      });

      return newStep;
    }),

  updateStep: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        name: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        type: z.enum(stepTypeEnum.enumValues).optional(),
        orderIndex: z.number().int().min(0).optional(),
        durationEstimate: z.number().int().min(0).optional(),
        required: z.boolean().optional(),
        conditions: z.record(z.string(), z.any()).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const userId = ctx.session.user.id;

      // Get step and experiment to check study access
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

      // Get experiment to access studyId
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, step.experimentId),
        columns: { studyId: true },
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      const updatedStepResults = await ctx.db
        .update(steps)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(steps.id, id))
        .returning();

      const updatedStep = updatedStepResults[0];
      if (!updatedStep) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update step",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "step_updated",
        description: `Updated step "${updatedStep.name}"`,
        resourceType: "step",
        resourceId: id,
      });

      return updatedStep;
    }),

  deleteStep: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get step and experiment to check study access
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

      // Get experiment to access studyId
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, step.experimentId),
        columns: { studyId: true },
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      // Delete step (cascades to actions)
      await ctx.db.delete(steps).where(eq(steps.id, input.id));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "step_deleted",
        description: `Deleted step "${step.name}"`,
        resourceType: "step",
        resourceId: input.id,
      });

      return { success: true };
    }),

  reorderSteps: protectedProcedure
    .input(
      z.object({
        experimentId: z.string().uuid(),
        stepIds: z.array(z.string().uuid()),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { experimentId, stepIds } = input;
      const userId = ctx.session.user.id;

      // Get experiment to check study access
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, experimentId),
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      // Verify all steps belong to this experiment
      const existingSteps = await ctx.db.query.steps.findMany({
        where: and(
          eq(steps.experimentId, experimentId),
          inArray(steps.id, stepIds),
        ),
      });

      if (existingSteps.length !== stepIds.length) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Some steps don't belong to this experiment",
        });
      }

      // Update order indexes
      for (let i = 0; i < stepIds.length; i++) {
        const stepId = stepIds[i];
        if (stepId) {
          await ctx.db
            .update(steps)
            .set({ orderIndex: i })
            .where(eq(steps.id, stepId));
        }
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "steps_reordered",
        description: `Reordered steps in experiment "${experiment.name}"`,
        resourceType: "experiment",
        resourceId: experimentId,
      });

      return { success: true };
    }),

  addAction: protectedProcedure
    .input(
      z.object({
        stepId: z.string().uuid(),
        name: z.string().min(1).max(255),
        description: z.string().optional(),
        type: z.string().min(1).max(100),
        orderIndex: z.number().int().min(0),
        parameters: z.record(z.string(), z.any()).optional(),
        validationSchema: z.record(z.string(), z.any()).optional(),
        timeout: z.number().int().min(0).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { stepId, ...actionData } = input;
      const userId = ctx.session.user.id;

      // Get step and experiment to check study access
      const step = await ctx.db.query.steps.findFirst({
        where: eq(steps.id, stepId),
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

      // Get experiment to access studyId
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, step.experimentId),
        columns: { studyId: true },
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access with researcher permission
      await checkStudyAccess(ctx.db, userId, experiment.studyId, [
        "owner",
        "researcher",
      ]);

      const newActionResults = await ctx.db
        .insert(actions)
        .values({
          stepId,
          ...actionData,
        })
        .returning();

      const newAction = newActionResults[0];
      if (!newAction) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create action",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: experiment.studyId,
        userId,
        action: "action_added",
        description: `Added action "${newAction.name}" to step "${step.name}"`,
        resourceType: "action",
        resourceId: newAction.id,
      });

      return newAction;
    }),

  updateAction: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        name: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        type: z.string().min(1).max(100).optional(),
        orderIndex: z.number().int().min(0).optional(),
        parameters: z.record(z.string(), z.any()).optional(),
        validationSchema: z.record(z.string(), z.any()).optional(),
        timeout: z.number().int().min(0).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const userId = ctx.session.user.id;

      // Get action, step, and experiment to check study access
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

      // Check study access with researcher permission
      await checkStudyAccess(
        ctx.db,
        userId,
        (action.step.experiment as { studyId: string }).studyId,
        ["owner", "researcher"],
      );

      const updatedActionResults = await ctx.db
        .update(actions)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(actions.id, id))
        .returning();

      const updatedAction = updatedActionResults[0];
      if (!updatedAction) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to update action",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: (action.step.experiment as { studyId: string }).studyId,
        userId,
        action: "action_updated",
        description: `Updated action "${updatedAction.name}"`,
        resourceType: "action",
        resourceId: id,
      });

      return updatedAction;
    }),

  deleteAction: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get action, step, and experiment to check study access
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

      // Check study access with researcher permission
      await checkStudyAccess(
        ctx.db,
        userId,
        (action.step.experiment as { studyId: string }).studyId,
        ["owner", "researcher"],
      );

      // Delete action
      await ctx.db.delete(actions).where(eq(actions.id, input.id));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: (action.step.experiment as { studyId: string }).studyId,
        userId,
        action: "action_deleted",
        description: `Deleted action "${action.name}"`,
        resourceType: "action",
        resourceId: input.id,
      });

      return { success: true };
    }),

  validate: protectedProcedure
    .input(z.object({ experimentId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Get experiment with steps and actions
      const experiment = await ctx.db.query.experiments.findFirst({
        where: eq(experiments.id, input.experimentId),
        with: {
          steps: {
            with: {
              actions: {
                orderBy: [asc(actions.orderIndex)],
              },
            },
            orderBy: [asc(steps.orderIndex)],
          },
        },
      });

      if (!experiment) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }

      // Check study access
      await checkStudyAccess(ctx.db, userId, experiment.studyId);

      const errors: Array<{
        type: string;
        message: string;
        stepId?: string;
        actionId?: string;
      }> = [];
      const warnings: Array<{
        type: string;
        message: string;
        stepId?: string;
        actionId?: string;
      }> = [];

      // Basic validation
      if (experiment.steps.length === 0) {
        errors.push({
          type: "no_steps",
          message: "Experiment must have at least one step",
        });
      }

      // Validate steps
      for (const step of experiment.steps) {
        if (step.actions.length === 0) {
          warnings.push({
            type: "empty_step",
            message: `Step "${step.name}" has no actions`,
            stepId: step.id,
          });
        }

        // Validate actions
        for (const action of step.actions) {
          if (action.timeout && action.timeout < 1) {
            errors.push({
              type: "invalid_timeout",
              message: `Action "${action.name}" has invalid timeout`,
              stepId: step.id,
              actionId: action.id,
            });
          }

          if (action.type === "wait" && !(action.parameters as { duration?: number })?.duration) {
            errors.push({
              type: "missing_duration",
              message: `Wait action "${action.name}" missing duration parameter`,
              stepId: step.id,
              actionId: action.id,
            });
          }
        }
      }

      // Check for duplicate step order indexes
      const orderIndexes = experiment.steps.map((s) => s.orderIndex);
      const uniqueOrderIndexes = new Set(orderIndexes);
      if (orderIndexes.length !== uniqueOrderIndexes.size) {
        errors.push({
          type: "duplicate_order",
          message: "Steps have duplicate order indexes",
        });
      }

      return {
        valid: errors.length === 0,
        errors,
        warnings,
      };
    }),
});
