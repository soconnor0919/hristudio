import { TRPCError } from "@trpc/server";
import { and, count, desc, eq, gte, inArray, lte, type SQL } from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import type { db } from "~/server/db";
import {
    annotations, auditLogs, experiments, mediaCaptures, participants, studies, systemSettings, trials, users, userSystemRoles
} from "~/server/db/schema";

// Helper function to check if user has system admin access
async function checkSystemAdminAccess(database: typeof db, userId: string) {
  const adminRole = await database
    .select()
    .from(userSystemRoles)
    .where(
      and(
        eq(userSystemRoles.userId, userId),
        eq(userSystemRoles.role, "administrator"),
      ),
    )
    .limit(1);

  if (!adminRole[0]) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "System administrator access required",
    });
  }
}

export const adminRouter = createTRPCRouter({
  getSystemStats: protectedProcedure
    .input(
      z.object({
        dateRange: z
          .object({
            startDate: z.date(),
            endDate: z.date(),
          })
          .optional(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      const dateConditions = [];
      if (input.dateRange) {
        dateConditions.push(
          gte(users.createdAt, input.dateRange.startDate),
          lte(users.createdAt, input.dateRange.endDate),
        );
      }

      // Get user statistics
      const totalUsersResult = await database
        .select({ count: count() })
        .from(users);
      const totalUsers = totalUsersResult[0]?.count ?? 0;

      const newUsersResult = input.dateRange
        ? await database
            .select({ count: count() })
            .from(users)
            .where(and(...dateConditions))
        : [];
      const newUsers = newUsersResult[0]?.count ?? 0;

      // Get study statistics
      const totalStudiesResult = await database
        .select({ count: count() })
        .from(studies);
      const totalStudies = totalStudiesResult[0]?.count ?? 0;

      const activeStudiesResult = await database
        .select({ count: count() })
        .from(studies)
        .where(eq(studies.status, "active"));
      const activeStudies = activeStudiesResult[0]?.count ?? 0;

      // Get experiment statistics
      const totalExperimentsResult = await database
        .select({ count: count() })
        .from(experiments);
      const totalExperiments = totalExperimentsResult[0]?.count ?? 0;

      // Get trial statistics
      const totalTrialsResult = await database
        .select({ count: count() })
        .from(trials);
      const totalTrials = totalTrialsResult[0]?.count ?? 0;

      const completedTrialsResult = await database
        .select({ count: count() })
        .from(trials)
        .where(eq(trials.status, "completed"));
      const completedTrials = completedTrialsResult[0]?.count ?? 0;

      const runningTrialsResult = await database
        .select({ count: count() })
        .from(trials)
        .where(eq(trials.status, "in_progress"));
      const runningTrials = runningTrialsResult[0]?.count ?? 0;

      // Get participant statistics
      const totalParticipantsResult = await database
        .select({ count: count() })
        .from(participants);
      const totalParticipants = totalParticipantsResult[0]?.count ?? 0;

      // Get storage statistics
      const totalMediaFilesResult = await database
        .select({ count: count() })
        .from(mediaCaptures);
      const totalMediaFiles = totalMediaFilesResult[0]?.count ?? 0;

      const totalStorageSizeResult = await database
        .select({ totalSize: mediaCaptures.fileSize })
        .from(mediaCaptures);

      const storageUsed = totalStorageSizeResult.reduce(
        (sum, file) => sum + (file.totalSize ?? 0),
        0,
      );

      // Get annotation statistics
      const totalAnnotationsResult = await database
        .select({ count: count() })
        .from(annotations);
      const totalAnnotations = totalAnnotationsResult[0]?.count ?? 0;

      return {
        users: {
          total: totalUsers,
          new: newUsers,
          active: totalUsers, // Users with recent activity
        },
        studies: {
          total: totalStudies,
          active: activeStudies,
          inactive: totalStudies - activeStudies,
        },
        experiments: {
          total: totalExperiments,
        },
        trials: {
          total: totalTrials,
          completed: completedTrials,
          running: runningTrials,
          scheduled: totalTrials - completedTrials - runningTrials,
        },
        participants: {
          total: totalParticipants,
        },
        storage: {
          totalFiles: totalMediaFiles,
          totalSize: storageUsed,
          averageFileSize:
            totalMediaFiles > 0 ? storageUsed / totalMediaFiles : 0,
        },
        annotations: {
          total: totalAnnotations,
        },
        system: {
          uptime: process.uptime(),
          nodeVersion: process.version,
          platform: process.platform,
          memory: process.memoryUsage(),
        },
      };
    }),

  getSystemSettings: protectedProcedure.query(async ({ ctx }) => {
    const { db: database } = ctx;
    const userId = ctx.session.user.id;

    await checkSystemAdminAccess(database, userId);

    const settings = await database
      .select()
      .from(systemSettings)
      .orderBy(systemSettings.key);

    // Convert to key-value object
    const settingsObj: Record<string, unknown> = {};
    settings.forEach((setting) => {
      settingsObj[setting.key] = setting.value;
    });

    return settingsObj;
  }),

  updateSystemSettings: protectedProcedure
    .input(
      z.object({
        settings: z.record(z.string(), z.unknown()),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      const updatedSettings = [];

      for (const [key, value] of Object.entries(input.settings)) {
        // Check if setting exists
        const existingSetting = await database
          .select()
          .from(systemSettings)
          .where(eq(systemSettings.key, key))
          .limit(1);

        if (existingSetting[0]) {
          // Update existing setting
          const updatedResults = await database
            .update(systemSettings)
            .set({
              value,
              updatedAt: new Date(),
              updatedBy: userId,
            })
            .where(eq(systemSettings.key, key))
            .returning();
          const updated = updatedResults[0];
          if (updated) {
            updatedSettings.push(updated);
          }
        } else {
          // Create new setting
          const createdResults = await database
            .insert(systemSettings)
            .values({
              key,
              value,
              updatedBy: userId,
            })
            .returning();
          const created = createdResults[0];
          if (created) {
            updatedSettings.push(created);
          }
        }

        // Log the setting change
        await database.insert(auditLogs).values({
          userId,
          action: existingSetting[0]
            ? "UPDATE_SYSTEM_SETTING"
            : "CREATE_SYSTEM_SETTING",
          resourceType: "system_setting",
          resourceId: key,
          changes: {
            key,
            oldValue: existingSetting[0]?.value,
            newValue: value,
          },
        });
      }

      return {
        success: true,
        updatedSettings,
      };
    }),

  getAuditLog: protectedProcedure
    .input(
      z.object({
        userId: z.string().optional(),
        action: z.string().optional(),
        resourceType: z.string().optional(),
        resourceId: z.string().optional(),
        dateRange: z
          .object({
            startDate: z.date(),
            endDate: z.date(),
          })
          .optional(),
        limit: z.number().min(1).max(1000).default(100),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      const conditions: SQL[] = [];

      if (input.userId) {
        conditions.push(eq(auditLogs.userId, input.userId));
      }
      if (input.action) {
        conditions.push(eq(auditLogs.action, input.action));
      }
      if (input.resourceType) {
        conditions.push(eq(auditLogs.resourceType, input.resourceType));
      }
      if (input.resourceId) {
        conditions.push(eq(auditLogs.resourceId, input.resourceId));
      }
      if (input.dateRange) {
        conditions.push(
                  gte(auditLogs.createdAt, input.dateRange.startDate),
        lte(auditLogs.createdAt, input.dateRange.endDate),
        );
      }

      const logs = await database
        .select()
        .from(auditLogs)
        .innerJoin(users, eq(auditLogs.userId, users.id))
        .where(conditions.length > 0 ? and(...conditions) : undefined)
        .orderBy(desc(auditLogs.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      return logs;
    }),

  createBackup: protectedProcedure
    .input(
      z.object({
        includeMediaFiles: z.boolean().default(false),
        description: z.string().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      // Log the backup request
      await database.insert(auditLogs).values({
        userId,
        action: "CREATE_BACKUP",
        resourceType: "system",
        resourceId: "backup",
        changes: {
          includeMediaFiles: input.includeMediaFiles,
          description: input.description,
        },
      });

      // TODO: Implement actual backup logic
      // This would typically involve:
      // 1. Creating a database dump
      // 2. Optionally backing up media files from R2
      // 3. Compressing the backup
      // 4. Storing it in a secure location
      // 5. Returning backup metadata

      // For now, return a mock response
      const backupId = `backup-${Date.now()}`;
      const estimatedSize = input.includeMediaFiles ? "2.5GB" : "250MB";

      return {
        backupId,
        status: "initiated",
        estimatedSize,
        estimatedCompletionTime: new Date(Date.now() + 15 * 60 * 1000), // 15 minutes
        includeMediaFiles: input.includeMediaFiles,
        description: input.description,
        createdBy: userId,
        createdAt: new Date(),
      };
    }),

  getBackupStatus: protectedProcedure
    .input(
      z.object({
        backupId: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      // TODO: Implement actual backup status checking
      // This would query a backup jobs table or external service

      // Mock response
      return {
        backupId: input.backupId,
        status: "completed",
        progress: 100,
        fileSize: "245MB",
        downloadUrl: `https://mock-backup-storage.com/backups/${input.backupId}.tar.gz`,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000), // 7 days
        createdAt: new Date(Date.now() - 15 * 60 * 1000), // 15 minutes ago
        completedAt: new Date(),
      };
    }),

  getUserManagement: protectedProcedure
    .input(
      z.object({
        search: z.string().optional(),
        role: z.string().optional(),
        status: z.enum(["active", "inactive"]).optional(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      const conditions: SQL[] = [];

      // TODO: Add search functionality when implemented
      // if (input.search) {
      //   conditions.push(
      //     or(
      //       ilike(users.name, `%${input.search}%`),
      //       ilike(users.email, `%${input.search}%`)
      //     )
      //   );
      // }

      const userList = await database
        .select({
          id: users.id,
          name: users.name,
          email: users.email,
          emailVerified: users.emailVerified,
          image: users.image,
          createdAt: users.createdAt,
          updatedAt: users.updatedAt,
        })
        .from(users)
        .where(conditions.length > 0 ? and(...conditions) : undefined)
        .orderBy(desc(users.createdAt))
        .limit(input.limit)
        .offset(input.offset);

      // Get system roles for each user
      const userIds = userList.map((u) => u.id);
      const systemRoles =
        userIds.length > 0
          ? await database
              .select()
              .from(userSystemRoles)
              .where(inArray(userSystemRoles.userId, userIds))
          : [];

      // Combine user data with roles
      const usersWithRoles = userList.map((user) => ({
        ...user,
        systemRoles: systemRoles
          .filter((role) => role.userId === user.id)
          .map((role) => role.role),
      }));

      return usersWithRoles;
    }),

  updateUserSystemRole: protectedProcedure
    .input(
      z.object({
        targetUserId: z.string(),
        role: z.enum(["administrator", "researcher"]),
        action: z.enum(["grant", "revoke"]),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db: database } = ctx;
      const userId = ctx.session.user.id;

      await checkSystemAdminAccess(database, userId);

      // Prevent self-modification of admin role
      if (input.targetUserId === userId && input.role === "administrator") {
        throw new TRPCError({
          code: "BAD_REQUEST",
          message: "Cannot modify your own admin role",
        });
      }

      if (input.action === "grant") {
        // Check if role already exists
        const existingRole = await database
          .select()
          .from(userSystemRoles)
          .where(
            and(
              eq(userSystemRoles.userId, input.targetUserId),
              eq(userSystemRoles.role, input.role),
            ),
          )
          .limit(1);

        if (existingRole[0]) {
          throw new TRPCError({
            code: "CONFLICT",
            message: "User already has this role",
          });
        }

        await database.insert(userSystemRoles).values({
          userId: input.targetUserId,
          role: input.role,
          grantedBy: userId,
        });
      } else {
        // Revoke role
        await database
          .delete(userSystemRoles)
          .where(
            and(
              eq(userSystemRoles.userId, input.targetUserId),
              eq(userSystemRoles.role, input.role),
            ),
          );
      }

      // Log the role change
      await database.insert(auditLogs).values({
        userId,
        action:
          input.action === "grant" ? "GRANT_SYSTEM_ROLE" : "REVOKE_SYSTEM_ROLE",
        resourceType: "user",
        resourceId: input.targetUserId,
        changes: {
          role: input.role,
          action: input.action,
        },
      });

      return { success: true };
    }),
});
