/**
 * Run `build` or `dev` with `SKIP_ENV_VALIDATION` to skip env validation. This is especially useful
 * for Docker builds.
 */
await import("./src/env.js");

/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: true,
  experimental: {
    serverActions: {
      bodySizeLimit: '2mb',
    },
  },
  webpack: (config) => {
    config.externals.push({
      "utf-8-validate": "commonjs utf-8-validate",
      bufferutil: "commonjs bufferutil",
    });
    return config;
  },
  // Add this section to disable linting during build
  eslint: {
    ignoreDuringBuilds: true,
  },
  // Add this section to disable type checking during build
  typescript: {
    ignoreBuildErrors: true,
  },
  // Add this section
  async rewrites() {
    return [
      {
        source: '/content/:path*',
        destination: '/api/content/:path*',
      },
    ];
  },
};

export default nextConfig;
