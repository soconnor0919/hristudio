import { createTRPCRouter } from "~/server/api/trpc";
import { studyRouter } from "~/server/api/routers/study";
import { participantRouter } from "~/server/api/routers/participant";
import { userRouter } from "~/server/api/routers/user";

/**
 * This is the primary router for your server.
 *
 * All routers added in /api/routers should be manually added here.
 */
export const appRouter = createTRPCRouter({
  study: studyRouter,
  participant: participantRouter,
  user: userRouter,
});

// export type definition of API
export type AppRouter = typeof appRouter;
