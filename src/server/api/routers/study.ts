import { z } from "zod"
import { eq } from "drizzle-orm"
import { TRPCError } from "@trpc/server"

import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc"
import { studies, studyMembers } from "~/server/db/schema"

export const studyRouter = createTRPCRouter({
  getMyStudies: protectedProcedure.query(async ({ ctx }) => {
    const myStudies = await ctx.db
      .select({
        id: studies.id,
        title: studies.title,
        description: studies.description,
        role: studyMembers.role,
      })
      .from(studies)
      .innerJoin(
        studyMembers,
        eq(studies.id, studyMembers.studyId),
      )
      .where(eq(studyMembers.userId, ctx.session.user.id))
      .orderBy(studies.createdAt)

    return myStudies
  }),

  getById: protectedProcedure
    .input(z.object({ id: z.number() }))
    .query(async ({ ctx, input }) => {
      const study = await ctx.db
        .select({
          id: studies.id,
          title: studies.title,
          description: studies.description,
          role: studyMembers.role,
        })
        .from(studies)
        .innerJoin(
          studyMembers,
          eq(studies.id, studyMembers.studyId),
        )
        .where(
          eq(studies.id, input.id),
          eq(studyMembers.userId, ctx.session.user.id)
        )
        .limit(1)
        .then((rows) => rows[0])

      if (!study) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        })
      }

      return study
    }),

  create: protectedProcedure
    .input(
      z.object({
        title: z.string().min(1).max(256),
        description: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const study = await ctx.db.transaction(async (tx) => {
        const [newStudy] = await tx
          .insert(studies)
          .values({
            title: input.title,
            description: input.description,
            createdById: ctx.session.user.id,
          })
          .returning()

        if (!newStudy) {
          throw new Error("Failed to create study")
        }

        await tx.insert(studyMembers).values({
          studyId: newStudy.id,
          userId: ctx.session.user.id,
          role: "admin",
        })

        return newStudy
      })

      return study
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.number(),
        title: z.string().min(1).max(256),
        description: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      // Check if user has access to study
      const member = await ctx.db
        .select({ role: studyMembers.role })
        .from(studyMembers)
        .where(
          eq(studyMembers.studyId, input.id),
          eq(studyMembers.userId, ctx.session.user.id)
        )
        .limit(1)
        .then((rows) => rows[0])

      if (!member) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        })
      }

      if (member.role !== "admin") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to update this study",
        })
      }

      const [study] = await ctx.db
        .update(studies)
        .set({
          title: input.title,
          description: input.description,
        })
        .where(eq(studies.id, input.id))
        .returning()

      if (!study) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        })
      }

      return study
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ ctx, input }) => {
      // Check if user has access to study
      const member = await ctx.db
        .select({ role: studyMembers.role })
        .from(studyMembers)
        .where(
          eq(studyMembers.studyId, input.id),
          eq(studyMembers.userId, ctx.session.user.id)
        )
        .limit(1)
        .then((rows) => rows[0])

      if (!member) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        })
      }

      if (member.role !== "admin") {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to delete this study",
        })
      }

      await ctx.db.transaction(async (tx) => {
        // Delete study members first (foreign key constraint)
        await tx
          .delete(studyMembers)
          .where(eq(studyMembers.studyId, input.id))

        // Then delete the study
        const [study] = await tx
          .delete(studies)
          .where(eq(studies.id, input.id))
          .returning()

        if (!study) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Study not found",
          })
        }
      })

      return { success: true }
    }),
}) 