/** @type {import('next').NextConfig} */
const nextConfig = {
  // Ignore type errors due to problems with next.js and delete routes
  typescript: {
    ignoreBuildErrors: true,
  },
}

module.exports = nextConfig
