import { TRPCError } from "@trpc/server";
import { and, count, eq, ilike, or, type SQL } from "drizzle-orm";
import { z } from "zod";
import bcrypt from "bcryptjs";

import {
  adminProcedure,
  createTRPCRouter,
  protectedProcedure,
} from "~/server/api/trpc";
import { systemRoleEnum, users, userSystemRoles } from "~/server/db/schema";

export const usersRouter = createTRPCRouter({
  list: adminProcedure
    .input(
      z.object({
        page: z.number().min(1).default(1),
        limit: z.number().min(1).max(100).default(20),
        search: z.string().optional(),
        role: z.enum(systemRoleEnum.enumValues).optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { page, limit, search, role } = input;
      const offset = (page - 1) * limit;

      // Build where conditions
      const conditions: SQL[] = [];

      if (search) {
        conditions.push(
          or(
            ilike(users.name, `%${search}%`),
            ilike(users.email, `%${search}%`),
          )!,
        );
      }

      const whereClause =
        conditions.length > 0 ? and(...conditions) : undefined;

      // Get users with their roles
      const usersQuery = ctx.db.query.users.findMany({
        where: whereClause,
        with: {
          systemRoles: true,
        },
        columns: {
          password: false, // Exclude password
        },
        limit,
        offset,
        orderBy: (users, { asc }) => [asc(users.createdAt)],
      });

      // Get total count
      const countQuery = ctx.db
        .select({ count: count() })
        .from(users)
        .where(whereClause);

      const [usersList, totalCountResult] = await Promise.all([
        usersQuery,
        countQuery,
      ]);

      const totalCount = totalCountResult[0]?.count ?? 0;

      // Filter by role if specified
      let filteredUsers = usersList;
      if (role) {
        filteredUsers = usersList.filter((user) =>
          user.systemRoles.some((sr) => sr.role === role),
        );
      }

      return {
        users: filteredUsers.map((user) => ({
          ...user,
          roles: user.systemRoles.map((sr) => sr.role),
        })),
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
      const user = await ctx.db.query.users.findFirst({
        where: eq(users.id, input.id),
        with: {
          systemRoles: {
            with: {
              grantedByUser: {
                columns: {
                  id: true,
                  name: true,
                  email: true,
                },
              },
            },
          },
        },
        columns: {
          password: false, // Exclude password
        },
      });

      if (!user) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      return {
        ...user,
        roles: user.systemRoles.map((sr) => ({
          role: sr.role,
          grantedAt: sr.grantedAt,
          grantedBy: sr.grantedByUser,
        })),
      };
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string().uuid(),
        name: z.string().min(1).max(255).optional(),
        email: z.string().email().optional(),
        image: z.string().url().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { id, ...updateData } = input;
      const currentUserId = ctx.session.user.id;

      // Check if user is updating their own profile or is an admin
      const isAdmin = await ctx.db.query.userSystemRoles.findFirst({
        where: and(
          eq(userSystemRoles.userId, currentUserId),
          eq(userSystemRoles.role, "administrator"),
        ),
      });

      if (id !== currentUserId && !isAdmin) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You can only update your own profile",
        });
      }

      // Check if user exists
      const existingUser = await ctx.db.query.users.findFirst({
        where: eq(users.id, id),
      });

      if (!existingUser) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Check if email is already taken by another user
      if (updateData.email && updateData.email !== existingUser.email) {
        const emailExists = await ctx.db.query.users.findFirst({
          where: and(eq(users.email, updateData.email), eq(users.id, id)),
        });

        if (emailExists) {
          throw new TRPCError({
            code: "CONFLICT",
            message: "Email is already taken",
          });
        }
      }

      const [updatedUser] = await ctx.db
        .update(users)
        .set({
          ...updateData,
          updatedAt: new Date(),
        })
        .where(eq(users.id, id))
        .returning({
          id: users.id,
          name: users.name,
          email: users.email,
          image: users.image,
          updatedAt: users.updatedAt,
        });

      return updatedUser;
    }),

  changePassword: protectedProcedure
    .input(
      z.object({
        currentPassword: z.string().min(1, "Current password is required"),
        newPassword: z
          .string()
          .min(6, "Password must be at least 6 characters"),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { currentPassword, newPassword } = input;
      const userId = ctx.session.user.id;

      // Get current user with password
      const user = await ctx.db.query.users.findFirst({
        where: eq(users.id, userId),
      });

      if (!user?.password) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Verify current password
      const isValidPassword = await bcrypt.compare(
        currentPassword,
        user.password,
      );
      if (!isValidPassword) {
        throw new TRPCError({
          code: "UNAUTHORIZED",
          message: "Current password is incorrect",
        });
      }

      // Hash new password
      const hashedNewPassword = await bcrypt.hash(newPassword, 12);

      // Update password
      await ctx.db
        .update(users)
        .set({
          password: hashedNewPassword,
          updatedAt: new Date(),
        })
        .where(eq(users.id, userId));

      return { success: true };
    }),

  assignRole: adminProcedure
    .input(
      z.object({
        userId: z.string().uuid(),
        role: z.enum(systemRoleEnum.enumValues),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { userId, role } = input;
      const currentUserId = ctx.session.user.id;

      // Check if target user exists
      const targetUser = await ctx.db.query.users.findFirst({
        where: eq(users.id, userId),
      });

      if (!targetUser) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Check if role assignment already exists
      const existingRole = await ctx.db.query.userSystemRoles.findFirst({
        where: and(
          eq(userSystemRoles.userId, userId),
          eq(userSystemRoles.role, role),
        ),
      });

      if (existingRole) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "User already has this role",
        });
      }

      // Assign the role
      const [newRole] = await ctx.db
        .insert(userSystemRoles)
        .values({
          userId,
          role,
          grantedBy: currentUserId,
        })
        .returning();

      return newRole;
    }),

  removeRole: adminProcedure
    .input(
      z.object({
        userId: z.string().uuid(),
        role: z.enum(systemRoleEnum.enumValues),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { userId, role } = input;

      // Check if role assignment exists
      const existingRole = await ctx.db.query.userSystemRoles.findFirst({
        where: and(
          eq(userSystemRoles.userId, userId),
          eq(userSystemRoles.role, role),
        ),
      });

      if (!existingRole) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User does not have this role",
        });
      }

      // Remove the role
      await ctx.db
        .delete(userSystemRoles)
        .where(
          and(
            eq(userSystemRoles.userId, userId),
            eq(userSystemRoles.role, role),
          ),
        );

      return { success: true };
    }),

  delete: adminProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const { id } = input;
      const currentUserId = ctx.session.user.id;

      // Prevent self-deletion
      if (id === currentUserId) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "You cannot delete your own account",
        });
      }

      // Check if user exists
      const existingUser = await ctx.db.query.users.findFirst({
        where: eq(users.id, id),
      });

      if (!existingUser) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Soft delete the user
      await ctx.db
        .update(users)
        .set({
          deletedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(users.id, id));

      return { success: true };
    }),

  restore: adminProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const { id } = input;

      // Check if user exists and is deleted
      const existingUser = await ctx.db.query.users.findFirst({
        where: eq(users.id, id),
      });

      if (!existingUser) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      if (!existingUser.deletedAt) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "User is not deleted",
        });
      }

      // Restore the user
      await ctx.db
        .update(users)
        .set({
          deletedAt: null,
          updatedAt: new Date(),
        })
        .where(eq(users.id, id));

      return { success: true };
    }),

  getWizards: protectedProcedure.query(async ({ ctx }) => {
    const userId = ctx.session.user.id;

    // Get all studies user is a member of
    const userStudies = await ctx.db.query.studyMembers.findMany({
      where: eq(studyMembers.userId, userId),
      columns: {
        studyId: true,
      },
    });

    const studyIds = userStudies.map((membership) => membership.studyId);

    if (studyIds.length === 0) {
      return [];
    }

    // Get all users who are members of the same studies and have wizard/researcher roles
    const studyMembersWithUsers = await ctx.db.query.studyMembers.findMany({
      where: inArray(studyMembers.studyId, studyIds),
      with: {
        user: {
          with: {
            systemRoles: true,
          },
          columns: {
            id: true,
            name: true,
            email: true,
          },
        },
      },
    });

    // Filter for users with wizard or researcher roles and deduplicate
    const wizardUsers = new Map();

    studyMembersWithUsers.forEach((member) => {
      const user = member.user;
      const hasWizardRole = user.systemRoles.some(
        (role) =>
          role.role === "wizard" ||
          role.role === "researcher" ||
          role.role === "administrator",
      );

      if (hasWizardRole && !wizardUsers.has(user.id)) {
        wizardUsers.set(user.id, {
          id: user.id,
          name: user.name,
          email: user.email,
          role:
            user.systemRoles.find(
              (role) =>
                role.role === "wizard" ||
                role.role === "researcher" ||
                role.role === "administrator",
            )?.role || "wizard",
        });
      }
    });

    return Array.from(wizardUsers.values());
  }),
});
