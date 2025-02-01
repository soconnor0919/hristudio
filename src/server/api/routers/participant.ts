import { z } from "zod"
import { eq } from "drizzle-orm"
import { TRPCError } from "@trpc/server"
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc"
import { participants } from "~/server/db/schema"
import { checkPermissions } from "~/lib/permissions/server"
import { PERMISSIONS } from "~/lib/permissions/constants"

const createParticipantSchema = z.object({
  studyId: z.string().uuid(),
  identifier: z.string().min(1).max(256),
  email: z.string().email().optional(),
  firstName: z.string().max(256).optional(),
  lastName: z.string().max(256).optional(),
  notes: z.string().optional(),
  status: z.enum(["active", "inactive"]).default("active"),
})

export const participantRouter = createTRPCRouter({
  getAll: protectedProcedure
    .input(z.object({ studyId: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.VIEW_PARTICIPANTS,
      })

      return ctx.db.query.participants.findMany({
        where: eq(participants.studyId, input.studyId),
        orderBy: (participants, { asc }) => [asc(participants.identifier)],
      })
    }),

  getById: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .query(async ({ ctx, input }) => {
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
      })

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        })
      }

      await checkPermissions({
        studyId: participant.studyId,
        permission: PERMISSIONS.VIEW_PARTICIPANTS,
      })

      return participant
    }),

  create: protectedProcedure
    .input(createParticipantSchema)
    .mutation(async ({ ctx, input }) => {
      await checkPermissions({
        studyId: input.studyId,
        permission: PERMISSIONS.MANAGE_PARTICIPANTS,
      })

      const [participant] = await ctx.db
        .insert(participants)
        .values(input)
        .returning()

      return participant
    }),

  update: protectedProcedure
    .input(z.object({
      id: z.string().uuid(),
      ...createParticipantSchema.partial().shape,
    }))
    .mutation(async ({ ctx, input }) => {
      const { id, ...data } = input

      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, id),
      })

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        })
      }

      await checkPermissions({
        studyId: participant.studyId,
        permission: PERMISSIONS.MANAGE_PARTICIPANTS,
      })

      const [updated] = await ctx.db
        .update(participants)
        .set({
          ...data,
          updatedAt: new Date(),
        })
        .where(eq(participants.id, id))
        .returning()

      return updated
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.string().uuid() }))
    .mutation(async ({ ctx, input }) => {
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
      })

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        })
      }

      await checkPermissions({
        studyId: participant.studyId,
        permission: PERMISSIONS.MANAGE_PARTICIPANTS,
      })

      await ctx.db.delete(participants).where(eq(participants.id, input.id))

      return { success: true }
    }),
}) 