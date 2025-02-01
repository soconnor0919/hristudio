export const env = createEnv({
  server: {
    DATABASE_URL: z.string().url(),
    STORAGE_TYPE: z.enum(["s3", "minio", "local"]).default("minio"),
    // ... other server-side env vars
  },
  client: {
    NEXT_PUBLIC_APP_URL: z.string().url(),
    // ... client-side env vars
  },
  runtimeEnv: {
    DATABASE_URL: process.env.DATABASE_URL,
    STORAGE_TYPE: process.env.STORAGE_TYPE,
    NEXT_PUBLIC_APP_URL: process.env.NEXT_PUBLIC_APP_URL
  }
}) 