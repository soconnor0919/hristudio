import "./src/env.mjs";
import { createRequire } from 'module';
const require = createRequire(import.meta.url);

/** @type {import('next').NextConfig} */
const config = {
  // Image configuration
  images: {
    remotePatterns: [
      {
        protocol: "https",
        hostname: "**",
      },
      {
        protocol: "http",
        hostname: "localhost",
        port: "3000",
        pathname: "/api/images/**",
      },
    ],
    dangerouslyAllowSVG: true,
    contentDispositionType: 'attachment',
  },

  // Package configuration
  transpilePackages: ["postgres"],

  // Enable experimental features
  experimental: {
    // Enable modern webpack features
    webpackBuildWorker: true,
    // Turbopack configuration (when using --turbo)
    turbo: {
      resolveAlias: {
        // Handle server-only modules in Turbo
        'server-only': 'server-only',
      },
    },
  },

  // Webpack fallbacks (only used when not using Turbo)
  webpack: (config, { isServer }) => {
    if (!isServer) {
      config.resolve.fallback = {
        ...config.resolve.fallback,
        fs: false,
        net: false,
        tls: false,
        crypto: false,
        os: false,
        path: false,
        stream: false,
        perf_hooks: false,
        child_process: false,
      };
    }
    return config;
  },
};

export default config; 