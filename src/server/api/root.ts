import { adminRouter } from "~/server/api/routers/admin";
import { analyticsRouter } from "~/server/api/routers/analytics";
import { authRouter } from "~/server/api/routers/auth";
import { collaborationRouter } from "~/server/api/routers/collaboration";
import { experimentsRouter } from "~/server/api/routers/experiments";
import { mediaRouter } from "~/server/api/routers/media";
import { participantsRouter } from "~/server/api/routers/participants";
import { robotsRouter } from "~/server/api/routers/robots";
import { studiesRouter } from "~/server/api/routers/studies";
import { trialsRouter } from "~/server/api/routers/trials";
import { usersRouter } from "~/server/api/routers/users";
import { createCallerFactory, createTRPCRouter } from "~/server/api/trpc";

/**
 * This is the primary router for your server.
 *
 * All routers added in /api/routers should be manually added here.
 */
export const appRouter = createTRPCRouter({
  auth: authRouter,
  users: usersRouter,
  studies: studiesRouter,
  experiments: experimentsRouter,
  participants: participantsRouter,
  trials: trialsRouter,
  robots: robotsRouter,
  media: mediaRouter,
  analytics: analyticsRouter,
  collaboration: collaborationRouter,
  admin: adminRouter,
});

// export type definition of API
export type AppRouter = typeof appRouter;

/**
 * Create a server-side caller for the tRPC API.
 * @example
 * const trpc = createCaller(createContext);
 * const res = await trpc.post.all();
 *       ^? Post[]
 */
export const createCaller = createCallerFactory(appRouter);
