import { TRPCError } from "@trpc/server";
import { and, desc, eq, type SQL } from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import {
  pluginStatusEnum,
  plugins,
  studyMembers,
  studyPlugins,
} from "~/server/db/schema";

export const pluginsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        robotId: z.string().optional(),
        status: z.enum(pluginStatusEnum.enumValues).optional(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const conditions: SQL[] = [];

      if (input.robotId) {
        conditions.push(eq(plugins.robotId, input.robotId));
      }

      if (input.status) {
        conditions.push(eq(plugins.status, input.status));
      }

      const query = ctx.db
        .select({
          id: plugins.id,
          robotId: plugins.robotId,
          name: plugins.name,
          version: plugins.version,
          description: plugins.description,
          author: plugins.author,
          repositoryUrl: plugins.repositoryUrl,
          trustLevel: plugins.trustLevel,
          status: plugins.status,
          createdAt: plugins.createdAt,
          updatedAt: plugins.updatedAt,
          metadata: plugins.metadata,
        })
        .from(plugins);

      const results = await (
        conditions.length > 0 ? query.where(and(...conditions)) : query
      )
        .orderBy(desc(plugins.updatedAt))
        .limit(input.limit)
        .offset(input.offset);

      return results;
    }),

  get: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const pluginResults = await ctx.db
        .select()
        .from(plugins)
        .where(eq(plugins.id, input.id))
        .limit(1);

      const plugin = pluginResults[0];

      if (!plugin) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Plugin not found",
        });
      }

      return plugin;
    }),

  install: protectedProcedure
    .input(
      z.object({
        studyId: z.string(),
        pluginId: z.string(),
        configuration: z.any().optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check if user has appropriate access
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership || !["owner", "researcher"].includes(membership.role)) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Insufficient permissions to install plugins",
        });
      }

      // Check if plugin exists
      const plugin = await ctx.db
        .select()
        .from(plugins)
        .where(eq(plugins.id, input.pluginId))
        .limit(1);

      if (!plugin[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Plugin not found",
        });
      }

      // Check if plugin is already installed
      const existing = await ctx.db
        .select()
        .from(studyPlugins)
        .where(
          and(
            eq(studyPlugins.studyId, input.studyId),
            eq(studyPlugins.pluginId, input.pluginId),
          ),
        )
        .limit(1);

      if (existing[0]) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "Plugin already installed for this study",
        });
      }

      const installations = await ctx.db
        .insert(studyPlugins)
        .values({
          studyId: input.studyId,
          pluginId: input.pluginId,
          configuration: input.configuration ?? {},
          installedBy: userId,
        })
        .returning();

      const installation = installations[0];
      if (!installation) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to install plugin",
        });
      }

      return installation;
    }),

  uninstall: protectedProcedure
    .input(
      z.object({
        studyId: z.string(),
        pluginId: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const userId = ctx.session.user.id;

      // Check if user has appropriate access
      const membership = await ctx.db.query.studyMembers.findFirst({
        where: and(
          eq(studyMembers.studyId, input.studyId),
          eq(studyMembers.userId, userId),
        ),
      });

      if (!membership || !["owner", "researcher"].includes(membership.role)) {
        throw new TRPCError({
          code: "FORBIDDEN",
          message: "Insufficient permissions to uninstall plugins",
        });
      }

      const result = await ctx.db
        .delete(studyPlugins)
        .where(
          and(
            eq(studyPlugins.studyId, input.studyId),
            eq(studyPlugins.pluginId, input.pluginId),
          ),
        )
        .returning();

      if (!result[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Plugin installation not found",
        });
      }

      return { success: true };
    }),

  getActions: protectedProcedure
    .input(
      z.object({
        pluginId: z.string(),
      }),
    )
    .query(async ({ ctx, input }) => {
      const plugin = await ctx.db
        .select({
          id: plugins.id,
          actionDefinitions: plugins.actionDefinitions,
        })
        .from(plugins)
        .where(eq(plugins.id, input.pluginId))
        .limit(1);

      if (!plugin[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Plugin not found",
        });
      }

      return plugin[0].actionDefinitions ?? [];
    }),
});
