import { z } from "zod";
import bcrypt from "bcryptjs";
import { TRPCError } from "@trpc/server";
import { eq } from "drizzle-orm";

import {
  createTRPCRouter,
  publicProcedure,
  protectedProcedure,
} from "~/server/api/trpc";
import { users } from "~/server/db/schema";

export const authRouter = createTRPCRouter({
  register: publicProcedure
    .input(
      z.object({
        name: z.string().min(1, "Name is required"),
        email: z.string().email("Invalid email address"),
        password: z.string().min(6, "Password must be at least 6 characters"),
      }),
    )
    .mutation(async ({ ctx, input }) => {
      const { name, email, password } = input;

      // Check if user already exists
      const existingUser = await ctx.db.query.users.findFirst({
        where: eq(users.email, email),
      });

      if (existingUser) {
        throw new TRPCError({
          code: "CONFLICT",
          message: "User with this email already exists",
        });
      }

      // Hash password
      const hashedPassword = await bcrypt.hash(password, 12);

      try {
        // Create user
        const newUsers = await ctx.db
          .insert(users)
          .values({
            name,
            email,
            password: hashedPassword,
          })
          .returning({
            id: users.id,
            name: users.name,
            email: users.email,
            createdAt: users.createdAt,
          });

        const newUser = newUsers[0];
        if (!newUser) {
          throw new TRPCError({
            code: "INTERNAL_SERVER_ERROR",
            message: "Failed to create user",
          });
        }

        return newUser;
      } catch (error: unknown) {
        throw new TRPCError({
          code: "INTERNAL_SERVER_ERROR",
          message:
            error instanceof Error ? error.message : "Failed to create user",
        });
      }
    }),

  logout: protectedProcedure.mutation(async ({ ctx: _ctx }) => {
    // Note: Actual logout is handled by NextAuth.js
    // This endpoint is for any additional cleanup if needed
    return { success: true };
  }),

  me: protectedProcedure.query(async ({ ctx }) => {
    const userId = ctx.session.user.id;

    const user = await ctx.db.query.users.findFirst({
      where: eq(users.id, userId),
      with: {
        systemRoles: {
          with: {
            grantedByUser: {
              columns: {
                id: true,
                name: true,
                email: true,
              },
            },
          },
        },
      },
      columns: {
        password: false, // Exclude password from response
      },
    });

    if (!user) {
      throw new TRPCError({
        code: "NOT_FOUND",
        message: "User not found",
      });
    }

    return {
      ...user,
      roles: user.systemRoles.map((sr) => sr.role),
    };
  }),
});
