import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "../trpc";
import {
  addRepository,
  getPlugins,
  getRepositories,
  removeRepository,
} from "~/lib/plugin-store/service";
import { TRPCError } from "@trpc/server";
import { eq } from "drizzle-orm";
import { installedPlugins } from "~/server/db/schema/store";

export const pluginStoreRouter = createTRPCRouter({
  // Get all repositories
  getRepositories: protectedProcedure.query(async () => {
    try {
      return await getRepositories();
    } catch (error) {
      console.error("Failed to get repositories:", error);
      throw new TRPCError({
        code: "INTERNAL_SERVER_ERROR",
        message: "Failed to get repositories",
      });
    }
  }),

  // Get all available plugins
  getPlugins: protectedProcedure.query(async () => {
    try {
      return await getPlugins();
    } catch (error) {
      console.error("Failed to get plugins:", error);
      throw new TRPCError({
        code: "INTERNAL_SERVER_ERROR",
        message: "Failed to get plugins",
      });
    }
  }),

  // Add a new repository
  addRepository: protectedProcedure
    .input(
      z.object({
        url: z.string().url(),
      }),
    )
    .mutation(async ({ input }) => {
      try {
        return await addRepository(input.url);
      } catch (error) {
        console.error("Failed to add repository:", error);
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message:
            error instanceof Error ? error.message : "Failed to add repository",
        });
      }
    }),

  // Remove a repository
  removeRepository: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ input }) => {
      try {
        await removeRepository(input.id);
        return { success: true };
      } catch (error) {
        console.error("Failed to remove repository:", error);
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message:
            error instanceof Error
              ? error.message
              : "Failed to remove repository",
        });
      }
    }),

  // Install a plugin
  installPlugin: protectedProcedure
    .input(
      z.object({
        robotId: z.string(),
        repositoryId: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      try {
        // Get plugin details
        const plugin = await getPlugins().then((plugins) =>
          plugins.find((p) => p.robotId === input.robotId),
        );

        if (!plugin) {
          throw new TRPCError({
            code: "NOT_FOUND",
            message: "Plugin not found",
          });
        }

        // Check if already installed
        const existing = await ctx.db.query.installedPlugins.findFirst({
          where: eq(installedPlugins.robotId, input.robotId),
        });

        if (existing) {
          throw new TRPCError({
            code: "BAD_REQUEST",
            message: "Plugin already installed",
          });
        }

        // Install plugin
        const [installed] = await ctx.db
          .insert(installedPlugins)
          .values({
            robotId: input.robotId,
            repositoryId: input.repositoryId,
            enabled: true,
            config: {},
          })
          .returning();

        return installed;
      } catch (error) {
        console.error("Failed to install plugin:", error);
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message:
            error instanceof Error ? error.message : "Failed to install plugin",
        });
      }
    }),

  // Uninstall a plugin
  uninstallPlugin: protectedProcedure
    .input(
      z.object({
        robotId: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      try {
        await ctx.db
          .delete(installedPlugins)
          .where(eq(installedPlugins.robotId, input.robotId));
        return { success: true };
      } catch (error) {
        console.error("Failed to uninstall plugin:", error);
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message:
            error instanceof Error
              ? error.message
              : "Failed to uninstall plugin",
        });
      }
    }),

  // Get installed plugins
  getInstalledPlugins: protectedProcedure.query(async ({ ctx }) => {
    try {
      return await ctx.db.query.installedPlugins.findMany({
        orderBy: (installedPlugins, { asc }) => [asc(installedPlugins.robotId)],
      });
    } catch (error) {
      console.error("Failed to get installed plugins:", error);
      throw new TRPCError({
        code: "INTERNAL_SERVER_ERROR",
        message: "Failed to get installed plugins",
      });
    }
  }),
});
