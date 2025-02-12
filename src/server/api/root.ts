import { createTRPCRouter } from "~/server/api/trpc";
import { studyRouter } from "./routers/study";
import { participantRouter } from "./routers/participant";
import { experimentRouter } from "./routers/experiment";

/**
 * This is the primary router for your server.
 *
 * All routers added in /api/routers should be manually added here.
 */
export const appRouter = createTRPCRouter({
  study: studyRouter,
  participant: participantRouter,
  experiment: experimentRouter,
});

// export type definition of API
export type AppRouter = typeof appRouter;
