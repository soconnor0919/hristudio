import { auth } from "~/server/auth";
import { redirect } from "next/navigation";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

export default async function StudiesPage() {
  const session = await auth();

  if (!session?.user) {
    redirect("/auth/signin");
  }

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      <div className="container mx-auto px-4 py-8">
        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold text-slate-900">Studies</h1>
            <p className="text-slate-600">
              Manage your Human-Robot Interaction research studies
            </p>
          </div>

          <div className="flex items-center gap-4">
            <span className="text-sm text-slate-600">
              Welcome, {session.user.name ?? session.user.email}
            </span>
            <div className="flex gap-2">
              <Button asChild variant="outline" size="sm">
                <Link href="/auth/signout">Sign Out</Link>
              </Button>
              <Button asChild variant="outline">
                <Link href="/">← Back to Home</Link>
              </Button>
            </div>
          </div>
        </div>

        {/* Studies Grid */}
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {/* Create New Study Card */}
          <Card className="border-2 border-dashed border-slate-300 transition-colors hover:border-slate-400">
            <CardHeader className="text-center">
              <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-blue-100">
                <svg
                  className="h-8 w-8 text-blue-600"
                  fill="none"
                  stroke="currentColor"
                  viewBox="0 0 24 24"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M12 4v16m8-8H4"
                  />
                </svg>
              </div>
              <CardTitle>Create New Study</CardTitle>
              <CardDescription>Start a new HRI research study</CardDescription>
            </CardHeader>
            <CardContent>
              <Button className="w-full" disabled>
                Create Study
              </Button>
            </CardContent>
          </Card>

          {/* Example Study Cards */}
          <Card>
            <CardHeader>
              <CardTitle>Robot Navigation Study</CardTitle>
              <CardDescription>
                Investigating user preferences for robot navigation patterns
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex justify-between text-sm text-slate-600">
                <span>Created: Dec 2024</span>
                <span>Status: Active</span>
              </div>
              <div className="flex gap-2">
                <Button size="sm" className="flex-1" disabled>
                  View Details
                </Button>
                <Button size="sm" variant="outline" className="flex-1" disabled>
                  Edit
                </Button>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Social Robot Interaction</CardTitle>
              <CardDescription>
                Analyzing human responses to social robot behaviors
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex justify-between text-sm text-slate-600">
                <span>Created: Nov 2024</span>
                <span>Status: Draft</span>
              </div>
              <div className="flex gap-2">
                <Button size="sm" className="flex-1" disabled>
                  View Details
                </Button>
                <Button size="sm" variant="outline" className="flex-1" disabled>
                  Edit
                </Button>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Empty State for No Studies */}
        <div className="mt-12 text-center">
          <div className="mx-auto mb-4 flex h-24 w-24 items-center justify-center rounded-lg bg-slate-100">
            <svg
              className="h-12 w-12 text-slate-400"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z"
              />
            </svg>
          </div>
          <h3 className="mb-2 text-lg font-semibold text-slate-900">
            Authentication Test Successful!
          </h3>
          <p className="mb-4 text-slate-600">
            You&apos;re viewing a protected page. The authentication system is
            working correctly. This page will be replaced with actual study
            management functionality.
          </p>
          <p className="text-sm text-slate-500">User ID: {session.user.id}</p>
        </div>
      </div>
    </div>
  );
}
