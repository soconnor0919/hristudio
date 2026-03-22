/**
 * Run `build` or `dev` with `SKIP_ENV_VALIDATION` to skip env validation. This is especially useful
 * for Docker builds.
 */
import "./src/env.js";

/** @type {import("next").NextConfig} */
const nextConfig = {
  // Mark server-only packages as external to prevent bundling in client
  serverExternalPackages: ["postgres", "minio", "child_process"],
};

export default nextConfig;
