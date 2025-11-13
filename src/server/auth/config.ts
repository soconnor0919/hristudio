import bcrypt from "bcryptjs";
import { eq } from "drizzle-orm";
import { type DefaultSession, type NextAuthConfig } from "next-auth";
import Credentials from "next-auth/providers/credentials";
import { z } from "zod";

import { db } from "~/server/db";
import { users } from "~/server/db/schema";

/**
 * Module augmentation for `next-auth` types. Allows us to add custom properties to the `session`
 * object and keep type safety.
 *
 * @see https://next-auth.js.org/getting-started/typescript#module-augmentation
 */
declare module "next-auth" {
  interface Session extends DefaultSession {
    user: {
      id: string;
      roles: Array<{
        role: "administrator" | "researcher" | "wizard" | "observer";
        grantedAt: Date;
        grantedBy: string | null;
      }>;
    } & DefaultSession["user"];
  }

  interface User {
    id: string;
    email: string;
    name: string | null;
    image: string | null;
  }
}

/**
 * Options for NextAuth.js used to configure adapters, providers, callbacks, etc.
 *
 * @see https://next-auth.js.org/configuration/options
 */

export const authConfig: NextAuthConfig = {
  session: {
    strategy: "jwt" as const,
    maxAge: 30 * 24 * 60 * 60, // 30 days
  },
  pages: {
    signIn: "/auth/signin",
    error: "/auth/error",
  },
  providers: [
    Credentials({
      name: "credentials",
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" },
      },
      async authorize(credentials) {
        const parsed = z
          .object({
            email: z.string().email(),
            password: z.string().min(6),
          })
          .safeParse(credentials);

        if (!parsed.success) return null;

        const user = await db.query.users.findFirst({
          where: eq(users.email, parsed.data.email),
        });

        if (!user?.password) return null;

        const isValidPassword = await bcrypt.compare(
          parsed.data.password,
          user.password,
        );

        if (!isValidPassword) return null;

        return {
          id: user.id,
          email: user.email,
          name: user.name,
          image: user.image,
        };
      },
    }),
  ],
  callbacks: {
    jwt: async ({ token, user }) => {
      if (user) {
        token.id = user.id;
      }
      return token;
    },
    session: async ({ session, token }) => {
      if (token.id && typeof token.id === 'string') {
        // Fetch user roles from database
        const userWithRoles = await db.query.users.findFirst({
          where: eq(users.id, token.id),
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
        });

        return {
          ...session,
          user: {
            ...session.user,
            id: token.id,
            roles:
              userWithRoles?.systemRoles?.map((sr) => ({
                role: sr.role,
                grantedAt: sr.grantedAt,
                grantedBy: sr.grantedBy,
              })) ?? [],
          },
        };
      }
      return session;
    },
  },
};
