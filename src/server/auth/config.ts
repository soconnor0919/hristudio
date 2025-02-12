import { DrizzleAdapter } from "@auth/drizzle-adapter";
import type { Session } from "@auth/core/types";
import type { DefaultSession } from "@auth/core/types";
import type { User } from "@auth/core/types";
import type { AuthConfig } from "@auth/core/types";
import CredentialsProvider from "@auth/core/providers/credentials";
import bcrypt from "bcryptjs";
import { eq } from "drizzle-orm";

import { db } from "~/server/db";
import {
  accounts,
  sessions,
  users,
  verificationTokens,
} from "~/server/db/schema";

/**
 * Module augmentation for `next-auth` types. Allows us to add custom properties to the `session`
 * object and keep type safety.
 *
 * @see https://next-auth.js.org/getting-started/typescript#module-augmentation
 */
declare module "@auth/core/types" {
  interface Session extends DefaultSession {
    user: {
      id: string;
      email: string;
      firstName: string | null;
      lastName: string | null;
    } & DefaultSession["user"];
  }

  interface User {
    id?: string;
    email?: string | null;
    firstName?: string | null;
    lastName?: string | null;
    password?: string | null;
    emailVerified?: Date | null;
    image?: string | null;
  }
}

/**
 * Options for NextAuth.js used to configure adapters, providers, callbacks, etc.
 *
 * @see https://next-auth.js.org/configuration/options
 */
export const authConfig = {
  adapter: DrizzleAdapter(db, {
    usersTable: users as any,
    accountsTable: accounts,
    sessionsTable: sessions,
    verificationTokensTable: verificationTokens,
  }),
  providers: [
    CredentialsProvider({
      name: "credentials",
      credentials: {
        email: { label: "Email", type: "email" },
        password: { label: "Password", type: "password" }
      },
      async authorize(credentials) {
        if (!credentials?.email || !credentials?.password) {
          throw new Error("Missing credentials");
        }

        const userEmail = credentials.email as string;
        const userPassword = credentials.password as string;

        const user = await db
          .select()
          .from(users)
          .where(eq(users.email, userEmail))
          .limit(1)
          .then(rows => rows[0]);

        if (!user) {
          throw new Error("Invalid credentials");
        }

        if (!user.password) {
          throw new Error("Invalid credentials");
        }

        const isValid = await bcrypt.compare(
          userPassword,
          user.password
        );
        
        if (!isValid) {
          throw new Error("Invalid credentials");
        }
        
        return {
          id: user.id,
          email: user.email,
          firstName: user.firstName,
          lastName: user.lastName,
          emailVerified: user.emailVerified,
          image: user.image,
        };
      }
    })
  ],
  callbacks: {
    session: ({ session, user }: { session: Session; user: User }) => ({
      ...session,
      user: {
        ...session.user,
        id: user.id,
        email: user.email,
        name: user.firstName && user.lastName ? `${user.firstName} ${user.lastName}` : null,
        firstName: user.firstName,
        lastName: user.lastName,
      },
    }),
  },
  pages: {
    signIn: '/auth/signin',
  },
} satisfies AuthConfig;
