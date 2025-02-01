import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { z } from "zod";
import { eq, and } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { studyMembers, users, studyRoleEnum } from "~/server/db/schema";
import { checkPermissions } from "~/lib/permissions/server";
import { PERMISSIONS } from "~/lib/permissions/constants";

const inviteSchema = z.object({
  studyId: z.string().uuid(),
  email: z.string().email(),
  role: z.enum(studyRoleEnum.enumValues),
});

export const studyMembersRouter = createTRPCRouter({
  getAll: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.VIEW_STUDY,
      });

      return ctx.db.query.studyMembers.findMany({
        where: eq(studyMembers.studyId, input.studyId),
        with: {
          user: true,
        },
      });
    }),

  invite: protectedProcedure
    .input(inviteSchema)
    .mutation(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.MANAGE_MEMBERS,
      });

      // Check if user exists
      const user = await ctx.db.query.users.findFirst({
        where: eq(users.email, input.email),
      });

      if (!user) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Check if already a member
      const existingMember = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, user.id)
        ),
      });

      if (existingMember) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "User is already a member of this study",
        });
      }

      // Add member
      const [member] = await ctx.db
        .insert(studyMembers)
        .values({
          studyId: input.studyId,
          userId: user.id,
          role: input.role,
        })
        .returning();

      return member;
    }),

  remove: protectedProcedure
    .input(z.object({
      studyId: z.string().uuid(),
      userId: z.string().uuid(),
    }))
    .mutation(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.MANAGE_MEMBERS,
      });

      await ctx.db
        .delete(studyMembers)
        .where(
          and(
            eq(studyMembers.studyId, input.studyId),
            eq(studyMembers.userId, input.userId)
          )
        );

      return { success: true };
    }),

  updateRole: protectedProcedure
    .input(z.object({
      studyId: z.string().uuid(),
      userId: z.string().uuid(),
      role: z.enum(studyRoleEnum.enumValues),
    }))
    .mutation(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.MANAGE_MEMBERS,
      });

      const [member] = await ctx.db
        .update(studyMembers)
        .set({ role: input.role })
        .where(
          and(
            eq(studyMembers.studyId, input.studyId),
            eq(studyMembers.userId, input.userId)
          )
        )
        .returning();

      if (!member) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Member not found",
        });
      }

      return member;
    }),
}); 