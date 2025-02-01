import type { NextAuthOptions } from "next-auth";
import NextAuth from "next-auth";
import { cache } from "react";

import { authConfig } from "./config";

const config: NextAuthOptions = {
  ...authConfig,
  providers: authConfig.providers as NextAuthOptions["providers"],
};

const { auth: uncachedAuth, handlers, signIn, signOut } = NextAuth(config);

const auth = cache(uncachedAuth);

export { auth, handlers, signIn, signOut };
