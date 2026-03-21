import { TRPCError } from "@trpc/server";
import { and, desc, eq, inArray, type SQL } from "drizzle-orm";
import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";
import { exec } from "child_process";
import { promisify } from "util";
 
const execAsync = promisify(exec);
import type { db } from "~/server/db";
import {
  communicationProtocolEnum,
  plugins,
  pluginStatusEnum,
  robots,
  studyMembers,
  studyPlugins,
} from "~/server/db/schema";

// Helper function to check if user has study access for robot operations
async function checkStudyAccess(
  database: typeof db,
  userId: string,
  studyId: string,
  requiredRoles: string[] = ["owner", "researcher"],
) {
  const membership = await database
    .select()
    .from(studyMembers)
    .where(
      and(
        eq(studyMembers.studyId, studyId),
        eq(studyMembers.userId, userId),
        inArray(
          studyMembers.role,
          requiredRoles as Array<
            "owner" | "researcher" | "wizard" | "observer"
          >,
        ),
      ),
    )
    .limit(1);

  if (!membership[0]) {
    throw new TRPCError({
      code: "FORBIDDEN",
      message: "Insufficient permissions to access this study",
    });
  }
}

export const robotsRouter = createTRPCRouter({
  list: protectedProcedure
    .input(
      z.object({
        protocol: z.enum(communicationProtocolEnum.enumValues).optional(),
        limit: z.number().min(1).max(100).default(50),
        offset: z.number().min(0).default(0),
      }),
    )
    .query(async ({ ctx, input }) => {
      const { db } = ctx;

      const conditions: SQL[] = [];

      if (input.protocol) {
        conditions.push(eq(robots.communicationProtocol, input.protocol));
      }

      const query = db
        .select({
          id: robots.id,
          name: robots.name,
          manufacturer: robots.manufacturer,
          model: robots.model,
          description: robots.description,
          capabilities: robots.capabilities,
          communicationProtocol: robots.communicationProtocol,
          createdAt: robots.createdAt,
          updatedAt: robots.updatedAt,
        })
        .from(robots);

      const results = await (
        conditions.length > 0 ? query.where(and(...conditions)) : query
      )
        .orderBy(desc(robots.updatedAt))
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
      const { db } = ctx;

      const robot = await db
        .select({
          id: robots.id,
          name: robots.name,
          manufacturer: robots.manufacturer,
          model: robots.model,
          description: robots.description,
          capabilities: robots.capabilities,
          communicationProtocol: robots.communicationProtocol,
          createdAt: robots.createdAt,
          updatedAt: robots.updatedAt,
        })
        .from(robots)
        .where(eq(robots.id, input.id))
        .limit(1);

      if (!robot[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Robot not found",
        });
      }

      return robot[0];
    }),

  create: protectedProcedure
    .input(
      z.object({
        name: z.string().min(1).max(255),
        manufacturer: z.string().max(255).optional(),
        model: z.string().max(255).optional(),
        description: z.string().optional(),
        capabilities: z.array(z.unknown()).optional(),
        communicationProtocol: z
          .enum(communicationProtocolEnum.enumValues)
          .optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;

      const insertedRobots = await db
        .insert(robots)
        .values({
          name: input.name,
          manufacturer: input.manufacturer,
          model: input.model,
          description: input.description,
          capabilities: input.capabilities ?? [],
          communicationProtocol: input.communicationProtocol,
        })
        .returning();

      const robot = insertedRobots[0];
      if (!robot) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message: "Failed to create robot",
        });
      }

      return robot;
    }),

  update: protectedProcedure
    .input(
      z.object({
        id: z.string(),
        name: z.string().min(1).max(255).optional(),
        manufacturer: z.string().max(255).optional(),
        model: z.string().max(255).optional(),
        description: z.string().optional(),
        capabilities: z.array(z.unknown()).optional(),
        communicationProtocol: z
          .enum(communicationProtocolEnum.enumValues)
          .optional(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;

      const updatedRobots = await db
        .update(robots)
        .set({
          name: input.name,
          manufacturer: input.manufacturer,
          model: input.model,
          description: input.description,
          capabilities: input.capabilities,
          communicationProtocol: input.communicationProtocol,
          updatedAt: new Date(),
        })
        .where(eq(robots.id, input.id))
        .returning();

      const robot = updatedRobots[0];
      if (!robot) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Robot not found",
        });
      }

      return robot;
    }),

  delete: protectedProcedure
    .input(
      z.object({
        id: z.string(),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { db } = ctx;

      const deletedRobots = await db
        .delete(robots)
        .where(eq(robots.id, input.id))
        .returning();

      if (!deletedRobots[0]) {
        throw new TRPCError({
          code: "NOT_FOUND",
          message: "Robot not found",
        });
      }

      return { success: true };
    }),

  // Plugin management routes
  plugins: createTRPCRouter({
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
        const { db } = ctx;

        const conditions: SQL[] = [];

        if (input.robotId) {
          conditions.push(eq(plugins.robotId, input.robotId));
        }

        if (input.status) {
          conditions.push(eq(plugins.status, input.status));
        }

        const query = db
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
        const { db } = ctx;

        const pluginResults = await db
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
        const { db } = ctx;
        const userId = ctx.session.user.id;

        await checkStudyAccess(db, userId, input.studyId, [
          "owner",
          "researcher",
        ]);

        // Check if plugin exists
        const plugin = await db
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
        const existing = await db
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

        const installations = await db
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
        const { db } = ctx;
        const userId = ctx.session.user.id;

        await checkStudyAccess(db, userId, input.studyId, [
          "owner",
          "researcher",
        ]);

        const result = await db
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
        const { db } = ctx;

        const plugin = await db
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

    getStudyPlugins: protectedProcedure
      .input(
        z.object({
          studyId: z.string(),
        }),
      )
      .query(async ({ ctx, input }) => {
        const { db } = ctx;
        const userId = ctx.session.user.id;

        await checkStudyAccess(db, userId, input.studyId, [
          "owner",
          "researcher",
          "wizard",
          "observer",
        ]);

        const installedPlugins = await db
          .select({
            plugin: {
              id: plugins.id,
              robotId: plugins.robotId,
              name: plugins.name,
              version: plugins.version,
              description: plugins.description,
              author: plugins.author,
              repositoryUrl: plugins.repositoryUrl,
              trustLevel: plugins.trustLevel,
              status: plugins.status,
              actionDefinitions: plugins.actionDefinitions,
              createdAt: plugins.createdAt,
              updatedAt: plugins.updatedAt,
              metadata: plugins.metadata,
            },
            installation: {
              id: studyPlugins.id,
              configuration: studyPlugins.configuration,
              installedAt: studyPlugins.installedAt,
              installedBy: studyPlugins.installedBy,
            },
          })
          .from(studyPlugins)
          .innerJoin(plugins, eq(studyPlugins.pluginId, plugins.id))
          .where(eq(studyPlugins.studyId, input.studyId))
          .orderBy(desc(studyPlugins.installedAt));

        return installedPlugins;
      }),
 
    initialize: protectedProcedure
      .input(
        z.object({
          id: z.string(),
        }),
      )
      .mutation(async ({ ctx, input }) => {
        const robotIp = process.env.NAO_ROBOT_IP || process.env.NAO_IP || "134.82.159.168";
        const password = process.env.NAO_PASSWORD || "robolab";
 
        console.log(`[Robots] Initializing robot ${input.id} at ${robotIp}`);
 
        try {
          // 1. Disable Autonomous Life
          const disableAlCmd = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; al = naoqi.ALProxy('ALAutonomousLife', '127.0.0.1', 9559); al.setState('disabled')\\""`;
 
          // 2. Wake Up (Stand Up)
          const wakeUpCmd = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.wakeUp()\\""`;
 
          // Execute commands sequentially
          console.log("[Robots] Executing AL disable...");
          await execAsync(disableAlCmd).catch((e) =>
            console.warn("AL disable failed (non-critical/already disabled):", e),
          );
 
          console.log("[Robots] Executing Wake Up...");
          await execAsync(wakeUpCmd);
 
          return { success: true };
        } catch (error) {
          console.error("Robot initialization failed:", error);
          throw new TRPCError({
            code: "INTERNAL_SERVER_ERROR",
            message: `Failed to initialize robot: ${error instanceof Error ? error.message : "SSH error"}`,
          });
        }
      }),
 
    executeSystemAction: protectedProcedure
      .input(
        z.object({
          id: z.string(), // actionId
          parameters: z.record(z.string(), z.any()),
        }),
      )
      .mutation(async ({ ctx, input }) => {
        const robotIp = process.env.NAO_ROBOT_IP || process.env.NAO_IP || "134.82.159.168";
        const password = process.env.NAO_PASSWORD || "robolab";
 
        console.log(`[Robots] Executing system action ${input.id}`);
 
        try {
          let command = "";
 
          switch (input.id) {
            case "say_with_emotion":
            case "say_text_with_emotion": {
              const text = String(input.parameters.text || "Hello");
              const emotion = String(input.parameters.emotion || "happy");
              // Map emotion to NaoQi tags
              const tag =
                emotion === "happy"
                  ? "^joyful"
                  : emotion === "sad"
                    ? "^sad"
                    : emotion === "thinking"
                      ? "^thoughtful"
                      : "^joyful";
 
              command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; s = naoqi.ALProxy('ALAnimatedSpeech', '127.0.0.1', 9559); s.say('${tag} ${text.replace(/'/g, "\\'")}')\\""`;
              break;
            }
 
            case "wake_up":
              command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.wakeUp()\\""`;
              break;
 
            case "rest":
              command = `sshpass -p "${password}" ssh -o StrictHostKeyChecking=no "nao@${robotIp}" "python2 -c \\"import sys; sys.path.append('/opt/aldebaran/lib/python2.7/site-packages'); import naoqi; m = naoqi.ALProxy('ALMotion', '127.0.0.1', 9559); m.rest()\\""`;
              break;
 
            default:
              throw new Error(`System action ${input.id} not implemented`);
          }
 
          await execAsync(command);
          return { success: true };
        } catch (error) {
          console.error(`System action ${input.id} failed:`, error);
          throw new TRPCError({
            code: "INTERNAL_SERVER_ERROR",
            message: `Action failed: ${error instanceof Error ? error.message : "SSH error"}`,
          });
        }
      }),
  }),
});
