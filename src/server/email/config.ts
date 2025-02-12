import { z } from "zod";
import { env } from "~/env";

export const emailConfigSchema = z.object({
  smtp: z.object({
    host: z.string(),
    port: z.number(),
    secure: z.boolean().default(false),
    auth: z.object({
      user: z.string(),
      pass: z.string(),
    }),
    tls: z.object({
      rejectUnauthorized: z.boolean().default(true),
    }).default({}),
  }),
  from: z.object({
    name: z.string(),
    email: z.string().email(),
  }),
});

export type EmailConfig = z.infer<typeof emailConfigSchema>;

export const emailConfig = {
  smtp: {
    host: env.SMTP_HOST,
    port: Number(env.SMTP_PORT),
    secure: false,
    auth: {
      user: env.SMTP_USER,
      pass: env.SMTP_PASS,
    },
    tls: {
      rejectUnauthorized: true,
    },
  },
  from: {
    name: env.EMAIL_FROM_NAME,
    email: env.EMAIL_FROM_ADDRESS,
  },
} satisfies EmailConfig; 