import { type Config } from "drizzle-kit";

import { env } from "~/env.mjs";

export default {
  schema: "./src/server/db/schema.ts",
  out: "./drizzle",
  dialect: "postgresql",
  dbCredentials: {
    url: env.DATABASE_URL,
  },
  strict: false,
  verbose: true,
  migrations: {
    table: "__drizzle_migrations",
    schema: "public"
  },
  tablesFilter: ["hs_*"],
} satisfies Config;
