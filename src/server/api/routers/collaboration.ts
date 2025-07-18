import { z } from "zod";
import { eq, and, desc, inArray, isNull } from "drizzle-orm";
import { TRPCError } from "@trpc/server";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import {
  comments,
  attachments,
  sharedResources,
  experiments,
  trials,
  studyMembers,
} from "~/server/db/schema";
import type { db } from "~/server/db";

// Helper function to check if user has access to a resource
async function checkResourceAccess(
  database: typeof db,
  userId: string,
  resourceType: string,
  resourceId: string,
  requiredRoles: ("owner" | "researcher" | "wizard" | "observer")[] = [
    "owner",
    "researcher",
    "wizard",
  ],
) {
  let studyId: string | undefined;

  switch (resourceType) {
    case "study":
      studyId = resourceId;
      break;
    case "experiment":
      const experiment = await database
        .select({ studyId: experiments.studyId })
        .from(experiments)
        .where(eq(experiments.id, resourceId))
        .limit(1);
      if (!experiment[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Experiment not found",
        });
      }
      studyId = experiment[0].studyId;
      break;
    case "trial":
      const trial = await database
        .select({ studyId: experiments.studyId })
        .from(trials)
        .innerJoin(experiments, eq(trials.experimentId, experiments.id))
        .where(eq(trials.id, resourceId))
        .limit(1);
      if (!trial[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Trial not found",
        });
      }
      studyId = trial[0].studyId;
      break;
    default:
      throw new TRPCError({
        code: "BAD_REQUEST",
        message: "Invalid resource type",
      });
  }

  if (!studyId) {
    throw new TRPCError({
      code: "NOT_FOUND",
      message: "Resource not found",
    });
  }

  const membership = await database
    .select()
    .from(studyMembers)
    .where(
      and(
        eq(studyMembers.studyId, studyId),
        eq(studyMembers.userId, userId),
        inArray(studyMembers.role, requiredRoles),
      ),
    )
    .limit(1);

  if (!membership[0]) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "Insufficient permissions to access this resource",
    });
  }

  return studyId;
}

// Helper function to generate presigned upload URL for attachments
async function generateAttachmentUploadUrl(
  fileName: string,
  contentType: string,
  studyId: string,
): Promise<{ uploadUrl: string; fileUrl: string }> {
  // TODO: Implement actual R2 presigned URL generation for attachments
  const key = `studies/${studyId}/attachments/${Date.now()}-${fileName}`;

  // Mock implementation - replace with actual R2 integration
  return {
    uploadUrl: `https://mock-r2-bucket.com/upload/${key}`,
    fileUrl: `https://mock-r2-bucket.com/files/${key}`,
  };
}

export const collaborationRouter = createTRPCRouter({
  createComment: protectedProcedure
    .input(
      z.object({
        resourceType: z.enum(["study", "experiment", "trial"]),
        resourceId: z.string(),
        content: z.string().min(1).max(5000),
        parentId: z.string().optional(), // For threaded comments
        metadata: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check access to the resource
      await checkResourceAccess(
        db,
        userId,
        input.resourceType,
        input.resourceId,
      );

      // If this is a reply, verify parent comment exists and belongs to same resource
      if (input.parentId) {
        const parentComment = await db
          .select({
            id: comments.id,
            resourceType: comments.resourceType,
            resourceId: comments.resourceId,
          })
          .from(comments)
          .where(eq(comments.id, input.parentId))
          .limit(1);

        if (!parentComment[0]) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Parent comment not found",
          });
        }

        if (
          parentComment[0].resourceType !== input.resourceType ||
          parentComment[0].resourceId !== input.resourceId
        ) {
          throw new TRPCError({
            code: "BAD_REQUEST",
            message: "Parent comment does not belong to the same resource",
          });
        }
      }

      const commentResults = await db
        .insert(comments)
        .values({
          resourceType: input.resourceType,
          resourceId: input.resourceId,
          authorId: userId,
          content: input.content,
          parentId: input.parentId,
          metadata: input.metadata,
        })
        .returning();

      const comment = commentResults[0];
      if (!comment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create comment",
        });
      }

      return comment;
    }),

  getComments: protectedProcedure
    .input(
      z.object({
        resourceType: z.enum(["study", "experiment", "trial"]),
        resourceId: z.string(),
        parentId: z.string().optional(), // Get replies to a specific comment
        includeReplies: z.boolean().default(true),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check access to the resource
      await checkResourceAccess(
        db,
        userId,
        input.resourceType,
        input.resourceId,
      );

      const conditions = [
        eq(comments.resourceType, input.resourceType),
        eq(comments.resourceId, input.resourceId),
      ];

      if (input.parentId) {
        conditions.push(eq(comments.parentId, input.parentId));
      } else if (!input.includeReplies) {
        // Only get top-level comments
        conditions.push(isNull(comments.parentId));
      }

      const results = await db
        .select({
          id: comments.id,
          resourceType: comments.resourceType,
          resourceId: comments.resourceId,
          authorId: comments.authorId,
          content: comments.content,
          parentId: comments.parentId,
          metadata: comments.metadata,
          createdAt: comments.createdAt,
          updatedAt: comments.updatedAt,
        })
        .from(comments)
        .where(and(...conditions))
        .orderBy(desc(comments.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return results;
    }),

  deleteComment: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Get the comment to check ownership and resource access
      const comment = await db
        .select({
          id: comments.id,
          resourceType: comments.resourceType,
          resourceId: comments.resourceId,
          authorId: comments.authorId,
        })
        .from(comments)
        .where(eq(comments.id, input.id))
        .limit(1);

      if (!comment[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Comment not found",
        });
      }

      // Check access to the resource
      await checkResourceAccess(
        db,
        userId,
        comment[0].resourceType,
        comment[0].resourceId,
      );

      // Only allow comment author or study owners/researchers to delete
      if (comment[0].authorId !== userId) {
        await checkResourceAccess(
          db,
          userId,
          comment[0].resourceType,
          comment[0].resourceId,
          ["owner", "researcher"],
        );
      }

      // Soft delete by updating content (preserve for audit trail)
      await db
        .update(comments)
        .set({
          content: "[Comment deleted]",
          updatedAt: new Date(),
        })
        .where(eq(comments.id, input.id))
        .returning();

      return { success: true };
    }),

  uploadAttachment: protectedProcedure
    .input(
      z.object({
        resourceType: z.enum(["study", "experiment", "trial"]),
        resourceId: z.string(),
        fileName: z.string(),
        fileSize: z.number().min(1),
        contentType: z.string(),
        description: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check access to the resource
      const studyId = await checkResourceAccess(
        db,
        userId,
        input.resourceType,
        input.resourceId,
      );

      // Generate presigned upload URL
      const { uploadUrl, fileUrl } = await generateAttachmentUploadUrl(
        input.fileName,
        input.contentType,
        studyId,
      );

      // Create attachment record
      const attachmentResults = await db
        .insert(attachments)
        .values({
          resourceType: input.resourceType,
          resourceId: input.resourceId,
          fileName: input.fileName,
          fileSize: input.fileSize,
          filePath: fileUrl,
          contentType: input.contentType,
          description: input.description,
          uploadedBy: userId,
        })
        .returning();

      const attachment = attachmentResults[0];
      if (!attachment) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create attachment",
        });
      }

      return {
        attachment,
        uploadUrl,
      };
    }),

  getAttachments: protectedProcedure
    .input(
      z.object({
        resourceType: z.enum(["study", "experiment", "trial"]),
        resourceId: z.string(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check access to the resource
      await checkResourceAccess(
        db,
        userId,
        input.resourceType,
        input.resourceId,
      );

      const results = await db
        .select({
          id: attachments.id,
          resourceType: attachments.resourceType,
          resourceId: attachments.resourceId,
          fileName: attachments.fileName,
          fileSize: attachments.fileSize,
          filePath: attachments.filePath,
          contentType: attachments.contentType,
          description: attachments.description,
          uploadedBy: attachments.uploadedBy,
          createdAt: attachments.createdAt,
        })
        .from(attachments)
        .where(
          and(
            eq(attachments.resourceType, input.resourceType),
            eq(attachments.resourceId, input.resourceId),
          ),
        )
        .orderBy(desc(attachments.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return results;
    }),

  // Token-based sharing functionality
  createShareLink: protectedProcedure
    .input(
      z.object({
        resourceType: z.enum(["study", "experiment", "trial"]),
        resourceId: z.string(),
        permissions: z.array(z.enum(["read", "comment", "annotate"])).default(["read"]),
        expiresAt: z.date().optional(),
        description: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check access to the resource (only owners and researchers can share)
      const studyId = await checkResourceAccess(
        db,
        userId,
        input.resourceType,
        input.resourceId,
        ["owner", "researcher"],
      );

      // Generate a unique share token
      const shareToken = `share_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;

      const sharedResourceResults = await db
        .insert(sharedResources)
        .values({
          studyId: studyId,
          resourceType: input.resourceType,
          resourceId: input.resourceId,
          sharedBy: userId,
          shareToken: shareToken,
          permissions: input.permissions,
          expiresAt: input.expiresAt,
        })
        .returning();

      const sharedResource = sharedResourceResults[0];
      if (!sharedResource) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create shared resource",
        });
      }

      // Generate the share URL
      const shareUrl = `${process.env.NEXT_PUBLIC_APP_URL}/shared/${shareToken}`;

      return {
        ...sharedResource,
        shareUrl,
      };
    }),

  getSharedResources: protectedProcedure
    .input(
      z.object({
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Get resources shared by the current user
      const results = await db
        .select({
          id: sharedResources.id,
          studyId: sharedResources.studyId,
          resourceType: sharedResources.resourceType,
          resourceId: sharedResources.resourceId,
          shareToken: sharedResources.shareToken,
          permissions: sharedResources.permissions,
          expiresAt: sharedResources.expiresAt,
          accessCount: sharedResources.accessCount,
          createdAt: sharedResources.createdAt,
        })
        .from(sharedResources)
        .where(eq(sharedResources.sharedBy, userId))
        .orderBy(desc(sharedResources.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      // Add share URLs to the results
      return results.map((resource) => ({
        ...resource,
        shareUrl: `${process.env.NEXT_PUBLIC_APP_URL}/shared/${resource.shareToken}`,
      }));
    }),

  revokeShare: protectedProcedure
    .input(
      z.object({
        shareId: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;
      const userId = ctx.session.user.id;

      // Check if the share exists and belongs to the user
      const share = await db
        .select({
          id: sharedResources.id,
          sharedBy: sharedResources.sharedBy,
        })
        .from(sharedResources)
        .where(eq(sharedResources.id, input.shareId))
        .limit(1);

      if (!share[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Share not found",
        });
      }

      if (share[0].sharedBy !== userId) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "You can only revoke your own shares",
        });
      }

      // Delete the share
      await db.delete(sharedResources).where(eq(sharedResources.id, input.shareId));

      return { success: true };
    }),

  // Public endpoint for accessing shared resources (no authentication required)
  accessSharedResource: protectedProcedure
    .input(
      z.object({
        shareToken: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;

      // Find the shared resource
      const sharedResource = await db
        .select({
          id: sharedResources.id,
          studyId: sharedResources.studyId,
          resourceType: sharedResources.resourceType,
          resourceId: sharedResources.resourceId,
          permissions: sharedResources.permissions,
          expiresAt: sharedResources.expiresAt,
          accessCount: sharedResources.accessCount,
        })
        .from(sharedResources)
        .where(eq(sharedResources.shareToken, input.shareToken))
        .limit(1);

      if (!sharedResource[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Share link not found or has expired",
        });
      }

      // Check if the share has expired
      if (sharedResource[0].expiresAt && sharedResource[0].expiresAt < new Date()) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Share link has expired",
        });
      }

      // Increment access count
      await db
        .update(sharedResources)
        .set({
          accessCount: sharedResource[0].accessCount + 1,
        })
        .where(eq(sharedResources.id, sharedResource[0].id));

      return {
        resourceType: sharedResource[0].resourceType,
        resourceId: sharedResource[0].resourceId,
        permissions: sharedResource[0].permissions,
        // Note: The actual resource data would be fetched based on resourceType and resourceId
        // This is just the metadata about the share
      };
    }),
});
