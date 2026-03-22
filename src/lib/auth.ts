import { betterAuth } from "better-auth";
import { drizzleAdapter } from "@better-auth/drizzle-adapter";
import { nextCookies } from "better-auth/next-js";
import { db } from "~/server/db";
import {
  users,
  accounts,
  sessions,
  verificationTokens,
} from "~/server/db/schema";
import bcrypt from "bcryptjs";

const baseURL =
  process.env.NEXTAUTH_URL ||
  process.env.BETTER_AUTH_URL ||
  "http://localhost:3000";

export const auth = betterAuth({
  baseURL,
  database: drizzleAdapter(db, {
    provider: "pg",
    schema: {
      user: users,
      account: accounts,
      session: sessions,
      verification: verificationTokens,
    },
  }),
  emailAndPassword: {
    enabled: true,
    password: {
      hash: async (password: string) => {
        return bcrypt.hash(password, 12);
      },
      verify: async ({
        hash,
        password,
      }: {
        hash: string;
        password: string;
      }) => {
        return bcrypt.compare(password, hash);
      },
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 30,
    updateAge: 60 * 60 * 24,
    modelName: "session",
    fields: {
      id: "id",
      token: "token",
      userId: "userId",
      expiresAt: "expiresAt",
      ipAddress: "ipAddress",
      userAgent: "userAgent",
    },
  },
  account: {
    modelName: "account",
    fields: {
      id: "id",
      providerId: "providerId",
      accountId: "accountId",
      userId: "userId",
      accessToken: "accessToken",
      refreshToken: "refreshToken",
      expiresAt: "expiresAt",
      scope: "scope",
    },
  },
  pages: {
    signIn: "/auth/signin",
    error: "/auth/error",
  },
  plugins: [nextCookies()],
});

export type Session = typeof auth.$Infer.Session;
