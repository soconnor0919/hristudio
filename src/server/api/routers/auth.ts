import { z } from "zod";
import { createTRPCRouter, protectedProcedure } from "~/server/api/trpc";

export const authRouter = createTRPCRouter({
  getSession: protectedProcedure.query(({ ctx }) => {
    return {
      user: ctx.session.user,
      expires: ctx.session.expires,
    };
  }),
}); 