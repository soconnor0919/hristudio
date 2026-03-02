import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { auth } from "~/server/auth";

export default async function UnauthorizedPage() {
  const session = await auth();

  return (
    <div className="flex min-h-screen items-center justify-center bg-gradient-to-br from-slate-50 to-slate-100 px-4">
      <div className="w-full max-w-md">
        {/* Header */}
        <div className="mb-8 text-center">
          <Link href="/" className="inline-block">
            <h1 className="text-3xl font-bold text-slate-900">HRIStudio</h1>
          </Link>
          <p className="mt-2 text-slate-600">Access Denied</p>
        </div>

        {/* Unauthorized Card */}
        <Card>
          <CardHeader className="text-center">
            <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-red-100">
              <svg
                className="h-8 w-8 text-red-600"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.728-.833-2.498 0L4.316 16.5c-.77.833.192 2.5 1.732 2.5z"
                />
              </svg>
            </div>
            <CardTitle>Access Denied</CardTitle>
            <CardDescription>
              You don&apos;t have permission to access this resource
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="rounded-md bg-red-50 p-3 text-sm text-red-700">
              <p className="font-medium">Insufficient Permissions</p>
              <p className="mt-1">
                This page requires additional privileges that your account
                doesn&apos;t have. Please contact your administrator to request
                access.
              </p>
            </div>

            {session?.user && (
              <div className="rounded-md bg-blue-50 p-3 text-sm text-blue-700">
                <p className="font-medium">Current User:</p>
                <p>{session.user.name ?? session.user.email}</p>
                {session.user.roles && session.user.roles.length > 0 ? (
                  <p className="mt-1">
                    Roles: {session.user.roles.map((r) => r.role).join(", ")}
                  </p>
                ) : (
                  <p className="mt-1">No roles assigned</p>
                )}
              </div>
            )}

            <div className="flex gap-3">
              <Button asChild className="flex-1">
                <Link href="/">Go Home</Link>
              </Button>
              <Button asChild variant="outline" className="flex-1">
                <Link href="/studies">My Studies</Link>
              </Button>
            </div>

            <div className="text-center">
              <p className="text-sm text-slate-600">
                Need help?{" "}
                <Link
                  href="/contact"
                  className="font-medium text-blue-600 hover:text-blue-500"
                >
                  Contact Support
                </Link>
              </p>
            </div>
          </CardContent>
        </Card>

        {/* Footer */}
        <div className="mt-8 text-center text-xs text-slate-500">
          <p>
            © 2024 HRIStudio. A platform for Human-Robot Interaction research.
          </p>
        </div>
      </div>
    </div>
  );
}
