import "server-only";

import { createTRPCReact } from "@trpc/react-query";
import { headers } from "next/headers";
import { cache } from "react";

import { appRouter, type AppRouter } from "~/server/api/root";
import { createTRPCContext } from "~/server/api/trpc";
import { createQueryClient } from "./query-client";

const defaultQueryClientOptions = {
  defaultOptions: {
    queries: {
      retry: false,
      onError: (error: unknown) => {
        const err = error as { message?: string };
        // Ignore unauthorized errors on public pages
        if (err?.message === "UNAUTHORIZED" && typeof window !== "undefined" && window.location.pathname.match(/^\/(auth\/signin|auth\/signup|$)/)) {
          return;
        }
      },
    },
  },
};

const api = createTRPCReact<AppRouter>();

/**
 * This wraps the `createTRPCContext` helper and provides the required context for the tRPC API when
 * handling a tRPC call from a React Server Component.
 */
const createContext = cache(async () => {
  const heads = new Headers();
  heads.set("x-trpc-source", "rsc");
  
  return createTRPCContext({
    headers: heads,
  });
});

const getQueryClient = cache(() => createQueryClient(defaultQueryClientOptions));
const getCaller = cache(async () => appRouter.createCaller(await createContext()));

export { api, getCaller };
