import { z } from "zod"
import { and, eq, count } from "drizzle-orm"
import { TRPCError } from "@trpc/server"
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc"
import { participants, studyMembers, type ParticipantStatus } from "~/server/db/schema"
import { ROLES, PERMISSIONS } from "~/lib/permissions/constants"
import { checkPermissions } from "~/lib/permissions/server"
import { studyActivities } from "~/server/db/schema/studies"

const createParticipantSchema = z.object({
  studyId: z.number(),
  identifier: z.string().optional(),
  email: z.string().email().optional().or(z.literal("")),
  firstName: z.string().optional(),
  lastName: z.string().optional(),
  notes: z.string().optional(),
  status: z.enum(["active", "inactive", "completed", "withdrawn"]).default("active"),
})

const updateParticipantSchema = z.object({
  id: z.number(),
  identifier: z.string().optional(),
  email: z.string().email().optional().or(z.literal("")),
  firstName: z.string().optional(),
  lastName: z.string().optional(),
  notes: z.string().optional(),
  status: z.enum(["active", "inactive", "completed", "withdrawn"]).optional(),
})

export const participantRouter = createTRPCRouter({
  getById: protectedProcedure
    .input(z.object({ id: z.number() }))
    .query(async ({ ctx, input }) => {
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check if user has permission to view participants
      await checkPermissions({
        studyId: participant.studyId,
        permission: "VIEW_PARTICIPANT_ANONYMIZED",
        session: ctx.session,
      });

      // Check if user has permission to view identifiable information
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, participant.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      const canViewIdentifiable = membership && [
        ROLES.OWNER.toLowerCase(),
        ROLES.ADMIN.toLowerCase(),
        ROLES.PRINCIPAL_INVESTIGATOR.toLowerCase()
      ].includes(membership.role.toLowerCase());

      if (!canViewIdentifiable) {
        return {
          ...participant,
          identifier: participant.identifier ? "REDACTED" : null,
          email: participant.email ? "REDACTED" : null,
          firstName: participant.firstName ? "REDACTED" : null,
          lastName: participant.lastName ? "REDACTED" : null,
        };
      }

      return participant;
    }),

  getByStudyId: protectedProcedure
    .input(z.object({ studyId: z.number() }))
    .query(async ({ ctx, input }) => {
      // Check if user has permission to view participants
      await checkPermissions({
        studyId: input.studyId,
        permission: "VIEW_PARTICIPANT_ANONYMIZED",
        session: ctx.session,
      });

      // Get participants
      const studyParticipants = await ctx.db.query.participants.findMany({
        where: eq(participants.studyId, input.studyId),
        orderBy: participants.createdAt,
      });

      // Check if user has permission to view identifiable information
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      const canViewIdentifiable = membership && [
        ROLES.OWNER.toLowerCase(),
        ROLES.ADMIN.toLowerCase(),
        ROLES.PRINCIPAL_INVESTIGATOR.toLowerCase()
      ].includes(membership.role.toLowerCase());

      if (!canViewIdentifiable) {
        return studyParticipants.map(participant => ({
          ...participant,
          identifier: participant.identifier ? "REDACTED" : null,
          email: participant.email ? "REDACTED" : null,
          firstName: participant.firstName ? "REDACTED" : null,
          lastName: participant.lastName ? "REDACTED" : null,
        }));
      }

      return studyParticipants;
    }),

  create: protectedProcedure
    .input(createParticipantSchema)
    .mutation(async ({ ctx, input }) => {
      // Check if user has permission to add participants
      await checkPermissions({
        studyId: input.studyId,
        permission: "ADD_PARTICIPANT",
        session: ctx.session,
      });

      const [participant] = await ctx.db
        .insert(participants)
        .values({
          ...input,
          createdAt: new Date(),
          updatedAt: new Date(),
        })
        .returning();

      // Log activity
      await ctx.db.insert(studyActivities).values({
        studyId: input.studyId,
        userId: ctx.session.user.id,
        type: "participant_added",
        description: `Added participant ${input.identifier ?? 'without identifier'}`,
      });

      return participant;
    }),

  update: protectedProcedure
    .input(updateParticipantSchema)
    .mutation(async ({ ctx, input }) => {
      // First get the participant to check study membership
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check if user has permission to edit participants
      await checkPermissions({
        studyId: participant.studyId,
        permission: "EDIT_PARTICIPANT",
        session: ctx.session,
      });

      const [updatedParticipant] = await ctx.db
        .update(participants)
        .set({
          ...input,
          updatedAt: new Date(),
        })
        .where(eq(participants.id, input.id))
        .returning();

      // Log activity
      await ctx.db.insert(studyActivities).values({
        studyId: participant.studyId,
        userId: ctx.session.user.id,
        type: "participant_updated",
        description: `Updated participant ${participant.identifier ?? 'without identifier'}`,
      });

      return updatedParticipant;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ ctx, input }) => {
      // First get the participant to check study membership
      const participant = await ctx.db.query.participants.findFirst({
        where: eq(participants.id, input.id),
      });

      if (!participant) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Participant not found",
        });
      }

      // Check if user has permission to delete participants
      await checkPermissions({
        studyId: participant.studyId,
        permission: "DELETE_PARTICIPANT",
        session: ctx.session,
      });

      // Log activity before deletion
      await ctx.db.insert(studyActivities).values({
        studyId: participant.studyId,
        userId: ctx.session.user.id,
        type: "participant_removed",
        description: `Removed participant ${participant.identifier ?? 'without identifier'}`,
      });

      await ctx.db.delete(participants).where(eq(participants.id, input.id));

      return { success: true };
    }),

  getCount: protectedProcedure
    .input(z.object({ studyId: z.number() }))
    .query(async ({ ctx, input }) => {
      // Check if user has permission to view participants
      await checkPermissions({
        studyId: input.studyId,
        permission: "VIEW_PARTICIPANT_ANONYMIZED",
        session: ctx.session,
      });

      const [result] = await ctx.db
        .select({ count: count() })
        .from(participants)
        .where(eq(participants.studyId, input.studyId));

      return result.count;
    }),
}); 