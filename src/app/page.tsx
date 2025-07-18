import Link from "next/link";
import { auth } from "~/server/auth";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

export default async function Home() {
  const session = await auth();

  return (
    <main className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      <div className="container mx-auto px-4 py-16">
        {/* Header */}
        <div className="mb-16 flex items-center justify-between">
          <div>
            <h1 className="mb-2 text-4xl font-bold text-slate-900">
              HRIStudio
            </h1>
            <p className="text-lg text-slate-600">
              Web-based platform for Human-Robot Interaction research
            </p>
          </div>

          <div className="flex items-center gap-4">
            {session?.user ? (
              <div className="flex items-center gap-4">
                <span className="text-sm text-slate-600">
                  Welcome, {session.user.name || session.user.email}
                </span>
                <Button asChild variant="outline">
                  <Link href="/api/auth/signout">Sign Out</Link>
                </Button>
              </div>
            ) : (
              <div className="flex gap-2">
                <Button asChild variant="outline">
                  <Link href="/auth/signin">Sign In</Link>
                </Button>
                <Button asChild>
                  <Link href="/auth/signup">Get Started</Link>
                </Button>
              </div>
            )}
          </div>
        </div>

        {/* Main Content */}
        <div className="mx-auto max-w-4xl">
          {session?.user ? (
            // Authenticated user dashboard
            <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
              <Card>
                <CardHeader>
                  <CardTitle>Experiments</CardTitle>
                  <CardDescription>
                    Design and manage your HRI experiments
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <Button className="w-full" asChild>
                    <Link href="/experiments">View Experiments</Link>
                  </Button>
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle>Wizard Interface</CardTitle>
                  <CardDescription>
                    Control robots during live trials
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <Button className="w-full" asChild>
                    <Link href="/wizard">Open Wizard</Link>
                  </Button>
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle>Data & Analytics</CardTitle>
                  <CardDescription>
                    Analyze trial results and performance
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <Button className="w-full" asChild>
                    <Link href="/analytics">View Data</Link>
                  </Button>
                </CardContent>
              </Card>
            </div>
          ) : (
            // Public landing page
            <div className="text-center">
              <div className="mx-auto mb-12 max-w-3xl">
                <h2 className="mb-6 text-3xl font-bold text-slate-900">
                  Standardize Your Wizard of Oz Studies
                </h2>
                <p className="mb-8 text-xl text-slate-600">
                  HRIStudio provides a comprehensive platform for designing,
                  executing, and analyzing Human-Robot Interaction experiments
                  with standardized Wizard of Oz methodologies.
                </p>

                <div className="mb-12 grid grid-cols-1 gap-8 md:grid-cols-3">
                  <div className="text-center">
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
                          d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z"
                        />
                      </svg>
                    </div>
                    <h3 className="mb-2 text-lg font-semibold text-slate-900">
                      Visual Experiment Designer
                    </h3>
                    <p className="text-slate-600">
                      Drag-and-drop interface for creating complex interaction
                      scenarios
                    </p>
                  </div>

                  <div className="text-center">
                    <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-green-100">
                      <svg
                        className="h-8 w-8 text-green-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M13 10V3L4 14h7v7l9-11h-7z"
                        />
                      </svg>
                    </div>
                    <h3 className="mb-2 text-lg font-semibold text-slate-900">
                      Real-time Control
                    </h3>
                    <p className="text-slate-600">
                      Live robot control with responsive wizard interface
                    </p>
                  </div>

                  <div className="text-center">
                    <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-lg bg-purple-100">
                      <svg
                        className="h-8 w-8 text-purple-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M9 19v-6a2 2 0 00-2-2H5a2 2 0 00-2 2v6a2 2 0 002 2h2a2 2 0 002-2zm0 0V9a2 2 0 012-2h2a2 2 0 012 2v10m-6 0a2 2 0 002 2h2a2 2 0 002-2m0 0V5a2 2 0 012-2h2a2 2 0 012 2v14a2 2 0 01-2 2h-2a2 2 0 01-2-2z"
                        />
                      </svg>
                    </div>
                    <h3 className="mb-2 text-lg font-semibold text-slate-900">
                      Advanced Analytics
                    </h3>
                    <p className="text-slate-600">
                      Comprehensive data capture and analysis tools
                    </p>
                  </div>
                </div>

                <Button size="lg" asChild>
                  <Link href="/auth/signup">Start Your Research</Link>
                </Button>
              </div>
            </div>
          )}
        </div>
      </div>
    </main>
  );
}
