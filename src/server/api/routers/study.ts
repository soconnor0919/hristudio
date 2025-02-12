import { z } from "zod"
import { and, eq, desc } from "drizzle-orm"
import { TRPCError } from "@trpc/server"
import { randomBytes } from "crypto"
import { addDays } from "date-fns"

import { createTRPCRouter, protectedProcedure, publicProcedure } from "~/server/api/trpc"
import { studies, studyMembers, studyMetadata, studyActivities, studyInvitations } from "~/server/db/schema/studies"
import { users } from "~/server/db/schema/auth"
import { checkPermissions } from "~/lib/permissions/server"
import { type Permission } from "~/lib/permissions/constants"
import { db } from '~/server/db'
import { ROLES } from "~/lib/permissions/constants"
import { EmailService } from "~/server/email/service"
import { PERMISSIONS } from "~/lib/permissions/constants"

const createStudySchema = z.object({
  title: z.string().min(1).max(255),
  description: z.string().optional(),
})

const updateStudySchema = z.object({
  id: z.number(),
  title: z.string().min(1).max(255),
  description: z.string().optional(),
})

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
      .innerJoin(studyMembers, eq(studyMembers.studyId, studies.id))
      .where(eq(studyMembers.userId, ctx.session.user.id))
      .orderBy(studies.createdAt);
    return myStudies;
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
        .innerJoin(studyMembers, eq(studyMembers.studyId, studies.id))
        .where(and(eq(studies.id, input.id), eq(studyMembers.userId, ctx.session.user.id)))
        .limit(1)
        .then((rows) => rows[0]);

      if (!study) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      return study;
    }),

  create: protectedProcedure
    .input(createStudySchema)
    .mutation(async ({ ctx, input }) => {
      const study = await ctx.db.transaction(async (tx) => {
        const [newStudy] = await tx
          .insert(studies)
          .values({
            title: input.title,
            description: input.description,
            createdById: ctx.session.user.id,
          })
          .returning();

        if (!newStudy) {
          throw new TRPCError({
            code: "INTERNAL_SERVER_ERROR",
            message: "Failed to create study",
          });
        }

        // Assign creator as owner
        await tx.insert(studyMembers).values({
          studyId: newStudy.id,
          userId: ctx.session.user.id,
          role: ROLES.OWNER,
        });

        // Log activity
        await tx.insert(studyActivities).values({
          studyId: newStudy.id,
          userId: ctx.session.user.id,
          type: "study_created",
          description: "Created study and assigned as owner",
        });

        return newStudy;
      });

      return study;
    }),

  update: protectedProcedure
    .input(updateStudySchema)
    .mutation(async ({ ctx, input }) => {
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.id),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to edit this study",
        });
      }

      const [updatedStudy] = await ctx.db
        .update(studies)
        .set({
          title: input.title,
          description: input.description,
          updatedAt: new Date(),
        })
        .where(eq(studies.id, input.id))
        .returning();

      if (!updatedStudy) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      return updatedStudy;
    }),

  delete: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ ctx, input }) => {
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.id),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You do not have permission to delete this study",
        });
      }

      await ctx.db.transaction(async (tx) => {
        await tx.delete(studyMembers).where(eq(studyMembers.studyId, input.id));
        const [deletedStudy] = await tx
          .delete(studies)
          .where(eq(studies.id, input.id))
          .returning();

        if (!deletedStudy) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Study not found",
          });
        }
      });

      return { success: true };
    }),

  createMutation: protectedProcedure
    .input(
      z.object({
        title: z.string().min(1, "Title is required"),
        description: z.string().optional(),
      })
    )
    .mutation(async ({ input, ctx }) => {
      if (!ctx.session?.user?.id) {
        throw new TRPCError({
          code: "UNAUTHORIZED",
          message: "User must be authenticated to create a study",
        });
      }
      try {
        const result = await db
          .insert(studies)
          .values({
            title: input.title,
            description: input.description ?? "",
            createdById: ctx.session.user.id,
            createdAt: new Date(),
            updatedAt: new Date(),
          })
          .returning();
        return result[0];
      } catch (error) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create study",
          cause: error,
        });
      }
    }),

  getMembers: protectedProcedure
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
          message: "You do not have permission to view study members",
        });
      }

      const members = await ctx.db
        .select({
          userId: studyMembers.userId,
          role: studyMembers.role,
          email: users.email,
          firstName: users.firstName,
          lastName: users.lastName,
        })
        .from(studyMembers)
        .innerJoin(users, eq(users.id, studyMembers.userId))
        .where(eq(studyMembers.studyId, input.studyId));

      return members.map(member => ({
        ...member,
        name: member.firstName && member.lastName 
          ? `${member.firstName} ${member.lastName}`
          : "Unknown",
      }));
    }),

  getPendingInvitations: protectedProcedure
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
          message: "You do not have permission to view study invitations",
        });
      }

      const invitations = await ctx.db
        .select({
          id: studyInvitations.id,
          email: studyInvitations.email,
          role: studyInvitations.role,
          createdAt: studyInvitations.createdAt,
          expiresAt: studyInvitations.expiresAt,
          creatorName: users.firstName,
        })
        .from(studyInvitations)
        .innerJoin(users, eq(users.id, studyInvitations.createdById))
        .where(and(
          eq(studyInvitations.studyId, input.studyId),
          eq(studyInvitations.status, "pending"),
        ))
        .orderBy(desc(studyInvitations.createdAt));

      return invitations;
    }),

  revokeInvitation: protectedProcedure
    .input(z.object({ 
      studyId: z.number(),
      invitationId: z.number(),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user is an admin
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only admins can revoke invitations",
        });
      }

      // Update invitation status
      await ctx.db.transaction(async (tx) => {
        const [invitation] = await tx
          .update(studyInvitations)
          .set({
            status: "revoked",
            updatedAt: new Date(),
          })
          .where(and(
            eq(studyInvitations.id, input.invitationId),
            eq(studyInvitations.studyId, input.studyId),
            eq(studyInvitations.status, "pending"),
          ))
          .returning();

        if (!invitation) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Invitation not found or already used",
          });
        }

        // Log activity
        await tx.insert(studyActivities).values({
          studyId: input.studyId,
          userId: ctx.session.user.id,
          type: "invitation_revoked",
          description: `Revoked invitation for ${invitation.email}`,
        });
      });

      return { success: true };
    }),

  inviteMember: protectedProcedure
    .input(z.object({
      studyId: z.number(),
      email: z.string().email(),
      role: z.enum(["researcher", "observer", "wizard", "principal_investigator", "admin"] as const),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user has permission to invite members
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You must be a member to invite others",
        });
      }

      // Define role hierarchy
      const roleHierarchy = {
        [ROLES.OWNER.toLowerCase()]: [ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR, ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD].map(r => r.toLowerCase()),
        [ROLES.ADMIN.toLowerCase()]: [ROLES.PRINCIPAL_INVESTIGATOR, ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD].map(r => r.toLowerCase()),
        [ROLES.PRINCIPAL_INVESTIGATOR.toLowerCase()]: [ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD].map(r => r.toLowerCase()),
      };

      const userRole = membership.role.toLowerCase();
      const targetRole = input.role.toLowerCase();

      // Check if user can invite with the specified role
      const allowedRoles = roleHierarchy[userRole] ?? [];
      if (!allowedRoles.includes(targetRole)) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You cannot invite members with this role",
        });
      }

      // Get study details for the email
      const study = await ctx.db.query.studies.findFirst({
        where: eq(studies.id, input.studyId),
      });

      if (!study) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Study not found",
        });
      }

      // Check if there's already a pending invitation
      const existingInvitation = await ctx.db.query.studyInvitations.findFirst({
        where: and(
          eq(studyInvitations.studyId, input.studyId),
          eq(studyInvitations.email, input.email),
          eq(studyInvitations.status, "pending"),
        ),
      });

      if (existingInvitation) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "An invitation has already been sent to this email",
        });
      }

      // Check if the user is already a member (if they exist)
      const existingUser = await ctx.db.query.users.findFirst({
        where: eq(users.email, input.email),
      });

      if (existingUser) {
        const existingMembership = await ctx.db.query.studyMembers.findFirst({
          where: and(
            eq(studyMembers.studyId, input.studyId),
            eq(studyMembers.userId, existingUser.id),
          ),
        });

        if (existingMembership) {
          throw new TRPCError({
            code: "BAD_REQUEST",
            message: "User is already a member of this study",
          });
        }
      }

      // Generate a secure random token
      const token = randomBytes(32).toString("hex");
      const expiresAt = addDays(new Date(), 7); // 7 days from now

      // Create the invitation
      const [invitation] = await ctx.db
        .insert(studyInvitations)
        .values({
          studyId: input.studyId,
          email: input.email,
          role: input.role,
          token,
          expiresAt,
          createdById: ctx.session.user.id,
        })
        .returning();

      // Log invitation sent activity
      await ctx.db.insert(studyActivities).values({
        studyId: input.studyId,
        userId: ctx.session.user.id,
        type: "invitation_sent",
        description: `Sent invitation to ${input.email} for role ${input.role}`,
      });

      // Send the invitation email
      const inviteUrl = `${process.env.NEXTAUTH_URL}/invite?token=${token}`;
      const emailService = new EmailService();
      
      await emailService.sendStudyInvitation({
        to: input.email,
        studyTitle: study.title,
        role: input.role,
        inviteUrl,
      });

      return invitation;
    }),

  updateMemberRole: protectedProcedure
    .input(z.object({
      studyId: z.number(),
      userId: z.string(),
      role: z.enum([ROLES.ADMIN, ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD]),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user is an admin
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only admins can update member roles",
        });
      }

      // Get user details for activity log
      const user = await ctx.db.query.users.findFirst({
        where: eq(users.id, input.userId),
      });

      if (!user) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "User not found",
        });
      }

      // Update role and log activity
      await ctx.db.transaction(async (tx) => {
        await tx
          .update(studyMembers)
          .set({ role: input.role })
          .where(
            and(
              eq(studyMembers.studyId, input.studyId),
              eq(studyMembers.userId, input.userId),
            ),
          );

        await tx.insert(studyActivities).values({
          studyId: input.studyId,
          userId: ctx.session.user.id,
          type: "member_role_changed",
          description: `Updated ${user.firstName} ${user.lastName}'s role to ${input.role}`,
        });
      });

      return { success: true };
    }),

  getMetadata: protectedProcedure
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
          message: "You do not have permission to view study metadata",
        });
      }

      return ctx.db.query.studyMetadata.findMany({
        where: eq(studyMetadata.studyId, input.studyId),
        orderBy: studyMetadata.key,
      });
    }),

  addMetadata: protectedProcedure
    .input(z.object({
      studyId: z.number(),
      key: z.string().min(1),
      value: z.string(),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user is an admin
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only admins can add metadata",
        });
      }

      // Check if key already exists
      const existing = await ctx.db.query.studyMetadata.findFirst({
        where: and(
          eq(studyMetadata.studyId, input.studyId),
          eq(studyMetadata.key, input.key),
        ),
      });

      if (existing) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "A field with this name already exists",
        });
      }

      // Add metadata and log activity
      await ctx.db.transaction(async (tx) => {
        await tx.insert(studyMetadata).values({
          studyId: input.studyId,
          key: input.key,
          value: input.value,
        });

        await tx.insert(studyActivities).values({
          studyId: input.studyId,
          userId: ctx.session.user.id,
          type: "study_updated",
          description: `Added metadata field: ${input.key}`,
        });
      });

      return { success: true };
    }),

  deleteMetadata: protectedProcedure
    .input(z.object({
      studyId: z.number(),
      key: z.string(),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user is an admin
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role !== ROLES.ADMIN) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only admins can delete metadata",
        });
      }

      // Delete metadata and log activity
      await ctx.db.transaction(async (tx) => {
        await tx
          .delete(studyMetadata)
          .where(
            and(
              eq(studyMetadata.studyId, input.studyId),
              eq(studyMetadata.key, input.key),
            ),
          );

        await tx.insert(studyActivities).values({
          studyId: input.studyId,
          userId: ctx.session.user.id,
          type: "study_updated",
          description: `Deleted metadata field: ${input.key}`,
        });
      });

      return { success: true };
    }),

  getActivities: protectedProcedure
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
          message: "You do not have permission to view study activities",
        });
      }

      const activities = await ctx.db
        .select({
          id: studyActivities.id,
          type: studyActivities.type,
          description: studyActivities.description,
          userId: studyActivities.userId,
          userName: users.firstName,
          createdAt: studyActivities.createdAt,
        })
        .from(studyActivities)
        .innerJoin(users, eq(users.id, studyActivities.userId))
        .where(eq(studyActivities.studyId, input.studyId))
        .orderBy(desc(studyActivities.createdAt))
        .limit(50);

      return activities;
    }),

  getInvitation: publicProcedure
    .input(z.object({ token: z.string() }))
    .query(async ({ ctx, input }) => {
      const invitation = await db.query.studyInvitations.findFirst({
        where: eq(studyInvitations.token, input.token),
        with: {
          study: true,
          creator: true,
        },
      });

      if (!invitation) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Invitation not found",
        });
      }

      if (invitation.status !== "pending") {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invitation has already been used",
        });
      }

      if (new Date() > invitation.expiresAt) {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invitation has expired",
        });
      }

      return invitation;
    }),

  acceptInvitation: protectedProcedure
    .input(z.object({ token: z.string() }))
    .mutation(async ({ ctx, input }) => {
      const invitation = await ctx.db.query.studyInvitations.findFirst({
        where: eq(studyInvitations.token, input.token),
      });

      if (!invitation) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Invitation not found",
        });
      }

      if (invitation.status !== "pending") {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invitation has already been used",
        });
      }

      if (new Date() > invitation.expiresAt) {
        // Log expired invitation
        await ctx.db.transaction(async (tx) => {
          await tx
            .update(studyInvitations)
            .set({
              status: "expired",
              updatedAt: new Date(),
            })
            .where(eq(studyInvitations.id, invitation.id));

          await tx.insert(studyActivities).values({
            studyId: invitation.studyId,
            userId: ctx.session.user.id,
            type: "invitation_expired",
            description: `Invitation for ${invitation.email} expired`,
          });
        });

        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Invitation has expired",
        });
      }

      if (ctx.session.user.email !== invitation.email) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "This invitation was sent to a different email address",
        });
      }

      // Add the user to the study with the specified role
      await ctx.db.transaction(async (tx) => {
        // Update invitation status
        await tx
          .update(studyInvitations)
          .set({
            status: "accepted",
            updatedAt: new Date(),
          })
          .where(eq(studyInvitations.id, invitation.id));

        // Add study membership
        await tx.insert(studyMembers).values({
          studyId: invitation.studyId,
          userId: ctx.session.user.id,
          role: invitation.role,
        });

        // Log invitation accepted activity
        await tx.insert(studyActivities).values({
          studyId: invitation.studyId,
          userId: ctx.session.user.id,
          type: "invitation_accepted",
          description: `Accepted invitation and joined study as ${invitation.role}`,
        });
      });

      return { success: true };
    }),

  transferOwnership: protectedProcedure
    .input(z.object({
      studyId: z.number(),
      newOwnerId: z.string(),
    }))
    .mutation(async ({ ctx, input }) => {
      // Check if user is the owner
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, ctx.session.user.id),
        ),
      });

      if (!membership || membership.role.toLowerCase() !== ROLES.OWNER.toLowerCase()) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Only the owner can transfer ownership",
        });
      }

      // Check if new owner exists and is a member
      const newOwnerMembership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, input.newOwnerId),
        ),
      });

      if (!newOwnerMembership) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "New owner must be a member of the study",
        });
      }

      // Transfer ownership in a transaction
      await ctx.db.transaction(async (tx) => {
        // Change current owner to admin
        await tx
          .update(studyMembers)
          .set({ role: ROLES.ADMIN })
          .where(
            and(
              eq(studyMembers.studyId, input.studyId),
              eq(studyMembers.userId, ctx.session.user.id),
            ),
          );

        // Set new owner
        await tx
          .update(studyMembers)
          .set({ role: ROLES.OWNER })
          .where(
            and(
              eq(studyMembers.studyId, input.studyId),
              eq(studyMembers.userId, input.newOwnerId),
            ),
          );

        // Log activity
        await tx.insert(studyActivities).values({
          studyId: input.studyId,
          userId: ctx.session.user.id,
          type: "ownership_transferred",
          description: `Transferred study ownership to ${newOwnerMembership.userId}`,
        });
      });

      return { success: true };
    }),

  // Additional endpoints (like getOverview or getAll) can be added if needed.
}); 