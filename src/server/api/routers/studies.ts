import { TRPCError } from "@trpc/server";
import { and, count, desc, eq, ilike, inArray, isNull, or } from "drizzle-orm";
import { z } from "zod";

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import {
  activityLogs,
  plugins,
  studies,
  studyMemberRoleEnum,
  studyMembers,
  studyPlugins,
  studyStatusEnum,
  users,
  userSystemRoles,
} from "~/server/db/schema";

export const studiesRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        search: z.string().optional(),
        status: z.enum(studyStatusEnum.enumValues).optional(),
        memberOnly: z.boolean().default(true), // Only show studies user is member of
      }),
    )
    .query(async ({ ctx, input }) => {
      const { page, limit, search, status, memberOnly } = input;
      const offset = (page - 1) * limit;
      const userId = ctx.session.user.id;

      // Build where conditions
      const conditions = [isNull(studies.deletedAt)];

      if (search) {
        conditions.push(
          or(
            ilike(studies.name, `%${search}%`),
            ilike(studies.description, `%${search}%`),
            ilike(studies.institution, `%${search}%`),
          )!,
        );
      }

      if (status) {
        conditions.push(eq(studies.status, status));
      }

      const whereClause = and(...conditions);

      // Check if user is admin (can see all studies)
      const isAdmin = await ctx.db.query.userSystemRoles.findFirst({
        where: and(
          eq(userSystemRoles.userId, userId),
          eq(userSystemRoles.role, "administrator"),
        ),
      });

      let studiesQuery;

      if (isAdmin && !memberOnly) {
        // Admin can see all studies
        studiesQuery = ctx.db.query.studies.findMany({
          where: whereClause,
          with: {
            createdBy: {
              columns: {
                id: true,
                name: true,
                email: true,
              },
            },
            members: {
              with: {
                user: {
                  columns: {
                    id: true,
                    name: true,
                    email: true,
                  },
                },
              },
            },
            experiments: {
              columns: {
                id: true,
              },
            },
            participants: {
              columns: {
                id: true,
              },
            },
          },
          limit,
          offset,
          orderBy: [desc(studies.updatedAt)],
        });
      } else {
        // Regular users see only studies they're members of
        // First get study IDs user is member of
        const userStudyMemberships = await ctx.db.query.studyMembers.findMany({
          where: eq(studyMembers.userId, userId),
          columns: {
            studyId: true,
          },
        });

        const userStudyIds = userStudyMemberships.map((m) => m.studyId);

        if (userStudyIds.length === 0) {
          studiesQuery = Promise.resolve([]);
        } else {
          studiesQuery = ctx.db.query.studies.findMany({
            where: and(whereClause, inArray(studies.id, userStudyIds)),
            with: {
              createdBy: {
                columns: {
                  id: true,
                  name: true,
                  email: true,
                },
              },
              members: {
                with: {
                  user: {
                    columns: {
                      id: true,
                      name: true,
                      email: true,
                    },
                  },
                },
              },
            },
            limit,
            offset,
            orderBy: [desc(studies.updatedAt)],
          });
        }
      }

      // Get total count
      const countQuery = ctx.db
        .select({ count: count() })
        .from(studies)
        .where(whereClause);

      const [studiesList, totalCountResult] = await Promise.all([
        studiesQuery,
        countQuery,
      ]);

      const totalCount = totalCountResult[0]?.count ?? 0;

      return {
        studies: studiesList,
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

      const study = await ctx.db.query.studies.findFirst({
        where: eq(studies.id, input.id),
        with: {
          createdBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
          members: {
            with: {
              user: {
                columns: {
                  id: true,
                  name: true,
                  email: true,
                },
              },
              invitedBy: {
                columns: {
                  id: true,
                  name: true,
                  email: true,
                },
              },
            },
          },
          experiments: {
            columns: {
              id: true,
              name: true,
              status: true,
              createdAt: true,
            },
          },
          participants: {
            columns: {
              id: true,
              participantCode: true,
              consentGiven: true,
              createdAt: true,
            },
          },
        },
      });

      if (!study) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      // Check if user has access to this study
      const userMembership = study.members.find((m) => m.userId === userId);
      const isAdmin = await ctx.db.query.userSystemRoles.findFirst({
        where: and(
          eq(userSystemRoles.userId, userId),
          eq(userSystemRoles.role, "administrator"),
        ),
      });

      if (!userMembership && !isAdmin) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have access to this study",
        });
      }

      return {
        ...study,
        userRole: userMembership?.role,
      };
    }),

  create: protectedProcedure
    .input(
      z.object({
        name: z.string().min(1).max(255),
        description: z.string().optional(),
        institution: z.string().max(255).optional(),
        irbProtocol: z.string().max(100).optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      const [newStudy] = await ctx.db
        .insert(studies)
        .values({
          ...input,
          createdBy: userId,
        })
        .returning();

      if (!newStudy) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create study",
        });
      }

      // Add creator as owner
      await ctx.db.insert(studyMembers).values({
        studyId: newStudy.id,
        userId,
        role: "owner",
      });

      // Auto-install core plugin in new study
      const corePlugin = await ctx.db.query.plugins.findFirst({
        where: eq(plugins.name, "HRIStudio Core System"),
      });

      if (corePlugin) {
        await ctx.db.insert(studyPlugins).values({
          studyId: newStudy.id,
          pluginId: corePlugin.id,
          configuration: {},
          installedBy: userId,
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: newStudy.id,
        userId,
        action: "study_created",
        description: `Created study "${newStudy.name}"`,
      });

      return newStudy;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        name: z.string().min(1).max(255).optional(),
        description: z.string().optional(),
        institution: z.string().max(255).optional(),
        irbProtocol: z.string().max(100).optional(),
        status: z.enum(studyStatusEnum.enumValues).optional(),
        settings: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const userId = ctx.session.user.id;

      // Check if user has permission to update this study
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, id),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership || !["owner", "researcher"].includes(membership.role)) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have permission to update this study",
        });
      }

      const [updatedStudy] = await ctx.db
        .update(studies)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(studies.id, id))
        .returning();

      if (!updatedStudy) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: id,
        userId,
        action: "study_updated",
        description: `Updated study "${updatedStudy.name}"`,
      });

      return updatedStudy;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check if user is owner of the study
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.id),
          eq(studyMembers.userId, userId),
          eq(studyMembers.role, "owner"),
        ),
      });

      if (!membership) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only study owners can delete studies",
        });
      }

      // Soft delete the study
      await ctx.db
        .update(studies)
        .set({
          deletedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(studies.id, input.id));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId: input.id,
        userId,
        action: "study_deleted",
        description: "Study deleted",
      });

      return { success: true };
    }),

  addMember: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        email: z.string().email(),
        role: z.enum(studyMemberRoleEnum.enumValues),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { studyId, email, role } = input;
      const userId = ctx.session.user.id;

      // Check if current user has permission to add members
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, studyId),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership || !["owner", "researcher"].includes(membership.role)) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have permission to add members to this study",
        });
      }

      // Find user by email
      const targetUser = await ctx.db.query.users.findFirst({
        where: eq(users.email, email),
      });

      if (!targetUser) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Check if user is already a member
      const existingMembership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, studyId),
          eq(studyMembers.userId, targetUser.id),
        ),
      });

      if (existingMembership) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "User is already a member of this study",
        });
      }

      // Add member
      const [newMember] = await ctx.db
        .insert(studyMembers)
        .values({
          studyId,
          userId: targetUser.id,
          role,
          invitedBy: userId,
        })
        .returning();

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId,
        userId,
        action: "member_added",
        description: `Added ${targetUser.name ?? targetUser.email} as ${role}`,
      });

      return newMember;
    }),

  removeMember: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        memberId: z.string().uuid(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { studyId, memberId } = input;
      const userId = ctx.session.user.id;

      // Check if current user has permission to remove members
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, studyId),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership || membership.role !== "owner") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only study owners can remove members",
        });
      }

      // Get member info for logging
      const memberToRemove = await ctx.db.query.studyMembers.findFirst({
        where: eq(studyMembers.id, memberId),
        with: {
          user: {
            columns: {
              name: true,
              email: true,
            },
          },
        },
      });

      if (!memberToRemove || memberToRemove.studyId !== studyId) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Member not found",
        });
      }

      // Prevent removing the last owner
      if (memberToRemove.role === "owner") {
        const ownerCount = await ctx.db
          .select({ count: count() })
          .from(studyMembers)
          .where(
            and(
              eq(studyMembers.studyId, studyId),
              eq(studyMembers.role, "owner"),
            ),
          );

        if ((ownerCount[0]?.count ?? 0) <= 1) {
          throw new TRPCError({
            code: "BAD_REQUEST",
            message: "Cannot remove the last owner of the study",
          });
        }
      }

      // Remove member
      await ctx.db.delete(studyMembers).where(eq(studyMembers.id, memberId));

      // Log activity
      await ctx.db.insert(activityLogs).values({
        studyId,
        userId,
        action: "member_removed",
        description: `Removed ${memberToRemove.user?.name ?? memberToRemove.user?.email ?? "Unknown user"}`,
      });

      return { success: true };
    }),

  getMembers: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check if user has access to this study
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You don't have access to this study",
        });
      }

      const members = await ctx.db.query.studyMembers.findMany({
        where: eq(studyMembers.studyId, input.studyId),
        with: {
          user: {
            columns: {
              id: true,
              name: true,
              email: true,
              image: true,
            },
          },
          invitedBy: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
        },
        orderBy: [desc(studyMembers.joinedAt)],
      });

      return members;
    }),

  getActivity: protectedProcedure
    .input(
      z.object({
        studyId: z.string().uuid(),
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { studyId, page, limit } = input;
      const offset = (page - 1) * limit;
      const userId = ctx.session.user.id;

      // Check if user has access to this study
      const membership = await ctx.db.query.studyMembers.findFirst({
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

      const activities = await ctx.db.query.activityLogs.findMany({
        where: eq(activityLogs.studyId, studyId),
        with: {
          user: {
            columns: {
              id: true,
              name: true,
              email: true,
            },
          },
        },
        limit,
        offset,
        orderBy: [desc(activityLogs.createdAt)],
      });

      const totalCount = await ctx.db
        .select({ count: count() })
        .from(activityLogs)
        .where(eq(activityLogs.studyId, studyId));

      return {
        activities,
        pagination: {
          page,
          limit,
          total: totalCount[0]?.count ?? 0,
          pages: Math.ceil((totalCount[0]?.count ?? 0) / limit),
        },
      };
    }),
});
