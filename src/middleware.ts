import { NextResponse } from "next/server"
import { type NextRequest } from "next/server"
import { withAuth } from "next-auth/middleware"

// Export withAuth middleware with custom configuration
export default withAuth(
  // `withAuth` augments your `Request` with the user's token.
  function middleware(req) {
    // If the user is not logged in and trying to access protected routes, redirect to signin
    if (!req.nextauth.token && req.nextUrl.pathname.startsWith("/dashboard")) {
      return NextResponse.redirect(new URL("/auth/signin", req.url))
    }
    return NextResponse.next()
  },
  {
    callbacks: {
      authorized: ({ token }) => !!token
    },
  }
)

// Specify protected routes
export const config = {
  matcher: [
    "/dashboard/:path*",
    "/studies/:path*",
    "/experiments/:path*",
    "/api/trpc/:path*"
  ]
} 