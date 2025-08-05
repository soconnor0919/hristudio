import Link from "next/link";
import { redirect } from "next/navigation";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Logo } from "~/components/ui/logo";
import { auth } from "~/server/auth";

export default async function Home() {
  const session = await auth();

  // Redirect authenticated users to their dashboard
  if (session?.user) {
    redirect("/dashboard");
  }

  return (
    <main className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      {/* Header */}
      <div className="border-b bg-white/50 backdrop-blur-sm">
        <div className="container mx-auto px-4 py-4">
          <div className="flex items-center justify-between">
            <Logo iconSize="md" showText={true} />

            <div className="flex items-center gap-4">
              <Button asChild variant="outline">
                <Link href="/auth/signin">Sign In</Link>
              </Button>
              <Button asChild>
                <Link href="/auth/signup">Get Started</Link>
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Hero Section */}
      <section className="container mx-auto px-4 py-20">
        <div className="mx-auto max-w-4xl text-center">
          <Badge variant="secondary" className="mb-4">
            🤖 Human-Robot Interaction Research Platform
          </Badge>
          <h1 className="mb-6 text-5xl font-bold tracking-tight text-slate-900">
            Standardize Your
            <span className="bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent">
              {" "}
              Wizard of Oz{" "}
            </span>
            Studies
          </h1>
          <p className="mb-8 text-xl leading-relaxed text-slate-600">
            A comprehensive web-based platform that enhances the scientific
            rigor of Human-Robot Interaction experiments while remaining
            accessible to researchers with varying levels of technical
            expertise.
          </p>
          <div className="flex flex-col justify-center gap-4 sm:flex-row">
            <Button size="lg" asChild>
              <Link href="/auth/signup">Start Your Research</Link>
            </Button>
            <Button size="lg" variant="outline" asChild>
              <Link href="#features">Learn More</Link>
            </Button>
          </div>
        </div>
      </section>

      {/* Problem Section */}
      <section className="bg-white/50 py-20">
        <div className="container mx-auto px-4">
          <div className="mx-auto max-w-4xl">
            <div className="mb-12 text-center">
              <h2 className="mb-4 text-3xl font-bold text-slate-900">
                The Challenge of WoZ Studies
              </h2>
              <p className="text-lg text-slate-600">
                While Wizard of Oz is a powerful paradigm for HRI research, it
                faces significant challenges
              </p>
            </div>

            <div className="grid grid-cols-1 gap-8 md:grid-cols-2">
              <Card>
                <CardHeader>
                  <CardTitle className="text-red-600">
                    Reproducibility Issues
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <ul className="space-y-2 text-slate-600">
                    <li>• Wizard behavior variability across trials</li>
                    <li>• Inconsistent experimental conditions</li>
                    <li>• Lack of standardized terminology</li>
                    <li>• Insufficient documentation</li>
                  </ul>
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle className="text-red-600">
                    Technical Barriers
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <ul className="space-y-2 text-slate-600">
                    <li>• Platform-specific robot control systems</li>
                    <li>• Extensive custom coding requirements</li>
                    <li>• Limited to domain experts</li>
                    <li>• Fragmented data collection</li>
                  </ul>
                </CardContent>
              </Card>
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section id="features" className="py-20">
        <div className="container mx-auto px-4">
          <div className="mx-auto max-w-6xl">
            <div className="mb-16 text-center">
              <h2 className="mb-4 text-3xl font-bold text-slate-900">
                Six Key Design Principles
              </h2>
              <p className="text-lg text-slate-600">
                Our platform addresses these challenges through comprehensive
                design principles
              </p>
            </div>

            <div className="grid grid-cols-1 gap-8 md:grid-cols-2 lg:grid-cols-3">
              <Card className="border-blue-200 bg-blue-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-blue-100">
                    <svg
                      className="h-6 w-6 text-blue-600"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10"
                      />
                    </svg>
                  </div>
                  <CardTitle>Integrated Environment</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    All functionalities unified in a single web-based platform
                    with intuitive interfaces
                  </p>
                </CardContent>
              </Card>

              <Card className="border-green-200 bg-green-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-green-100">
                    <svg
                      className="h-6 w-6 text-green-600"
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
                  <CardTitle>Visual Experiment Design</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    Minimal-to-no coding required with drag-and-drop visual
                    programming capabilities
                  </p>
                </CardContent>
              </Card>

              <Card className="border-purple-200 bg-purple-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-purple-100">
                    <svg
                      className="h-6 w-6 text-purple-600"
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
                  <CardTitle>Real-time Control</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    Fine-grained, real-time control of scripted experimental
                    runs with multiple robot platforms
                  </p>
                </CardContent>
              </Card>

              <Card className="border-orange-200 bg-orange-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-orange-100">
                    <svg
                      className="h-6 w-6 text-orange-600"
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
                  <CardTitle>Data Management</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    Comprehensive data collection and logging with structured
                    storage and retrieval
                  </p>
                </CardContent>
              </Card>

              <Card className="border-teal-200 bg-teal-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-teal-100">
                    <svg
                      className="h-6 w-6 text-teal-600"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M8.684 13.342C8.886 12.938 9 12.482 9 12c0-.482-.114-.938-.316-1.342m0 2.684a3 3 0 110-2.684m0 2.684l6.632 3.316m-6.632-6l6.632-3.316m0 0a3 3 0 105.367-2.684 3 3 0 00-5.367 2.684zm0 9.316a3 3 0 105.367 2.684 3 3 0 00-5.367-2.684z"
                      />
                    </svg>
                  </div>
                  <CardTitle>Platform Agnostic</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    Support for wide range of robot hardware through RESTful
                    APIs, ROS, and custom plugins
                  </p>
                </CardContent>
              </Card>

              <Card className="border-indigo-200 bg-indigo-50/50">
                <CardHeader>
                  <div className="mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-indigo-100">
                    <svg
                      className="h-6 w-6 text-indigo-600"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M17 20h5v-2a3 3 0 00-5.356-1.857M17 20H7m10 0v-2c0-.656-.126-1.283-.356-1.857M7 20H2v-2a3 3 0 015.356-1.857M7 20v-2c0-.656.126-1.283.356-1.857m0 0a5.002 5.002 0 019.288 0M15 7a3 3 0 11-6 0 3 3 0 016 0zm6 3a2 2 0 11-4 0 2 2 0 014 0zM7 10a2 2 0 11-4 0 2 2 0 014 0z"
                      />
                    </svg>
                  </div>
                  <CardTitle>Collaboration Support</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-slate-600">
                    Role-based access control and data sharing for effective
                    research team collaboration
                  </p>
                </CardContent>
              </Card>
            </div>
          </div>
        </div>
      </section>

      {/* Architecture Section */}
      <section className="bg-white/50 py-20">
        <div className="container mx-auto px-4">
          <div className="mx-auto max-w-4xl">
            <div className="mb-12 text-center">
              <h2 className="mb-4 text-3xl font-bold text-slate-900">
                Three-Layer Architecture
              </h2>
              <p className="text-lg text-slate-600">
                Modular web application with clear separation of concerns
              </p>
            </div>

            <div className="space-y-8">
              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <div className="h-3 w-3 rounded-full bg-blue-500"></div>
                    <span>User Interface Layer</span>
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
                    <div className="rounded-lg bg-blue-50 p-4 text-center">
                      <h4 className="font-semibold text-blue-900">
                        Experiment Designer
                      </h4>
                      <p className="mt-1 text-sm text-blue-700">
                        Visual programming for experimental protocols
                      </p>
                    </div>
                    <div className="rounded-lg bg-blue-50 p-4 text-center">
                      <h4 className="font-semibold text-blue-900">
                        Wizard Interface
                      </h4>
                      <p className="mt-1 text-sm text-blue-700">
                        Real-time control during trial execution
                      </p>
                    </div>
                    <div className="rounded-lg bg-blue-50 p-4 text-center">
                      <h4 className="font-semibold text-blue-900">
                        Playback & Analysis
                      </h4>
                      <p className="mt-1 text-sm text-blue-700">
                        Data exploration and visualization
                      </p>
                    </div>
                  </div>
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <div className="h-3 w-3 rounded-full bg-green-500"></div>
                    <span>Data Management Layer</span>
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="mb-4 text-slate-600">
                    Secure database functionality with role-based access control
                    (Researcher, Wizard, Observer) for organizing experiment
                    definitions, metadata, and media assets.
                  </p>
                  <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">PostgreSQL</Badge>
                    <Badge variant="secondary">MinIO Storage</Badge>
                    <Badge variant="secondary">Role-based Access</Badge>
                    <Badge variant="secondary">Cloud/On-premise</Badge>
                  </div>
                </CardContent>
              </Card>

              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <div className="h-3 w-3 rounded-full bg-purple-500"></div>
                    <span>Robot Integration Layer</span>
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="mb-4 text-slate-600">
                    Robot-agnostic communication layer supporting multiple
                    integration methods for diverse hardware platforms.
                  </p>
                  <div className="flex flex-wrap gap-2">
                    <Badge variant="secondary">RESTful APIs</Badge>
                    <Badge variant="secondary">ROS Integration</Badge>
                    <Badge variant="secondary">Custom Plugins</Badge>
                    <Badge variant="secondary">Docker Deployment</Badge>
                  </div>
                </CardContent>
              </Card>
            </div>
          </div>
        </div>
      </section>

      {/* Workflow Section */}
      <section className="py-20">
        <div className="container mx-auto px-4">
          <div className="mx-auto max-w-4xl">
            <div className="mb-12 text-center">
              <h2 className="mb-4 text-3xl font-bold text-slate-900">
                Hierarchical Experiment Structure
              </h2>
              <p className="text-lg text-slate-600">
                Standardized terminology and organization for reproducible
                research
              </p>
            </div>

            <div className="relative">
              {/* Hierarchy visualization */}
              <div className="space-y-6">
                <Card className="border-l-4 border-l-blue-500">
                  <CardContent className="pt-6">
                    <div className="flex items-center space-x-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100 text-sm font-semibold text-blue-600">
                        1
                      </div>
                      <div>
                        <h3 className="font-semibold">Study</h3>
                        <p className="text-sm text-slate-600">
                          Top-level container comprising one or more experiments
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>

                <Card className="ml-8 border-l-4 border-l-green-500">
                  <CardContent className="pt-6">
                    <div className="flex items-center space-x-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-green-100 text-sm font-semibold text-green-600">
                        2
                      </div>
                      <div>
                        <h3 className="font-semibold">Experiment</h3>
                        <p className="text-sm text-slate-600">
                          Parameterized template specifying experimental
                          protocol
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>

                <Card className="ml-16 border-l-4 border-l-orange-500">
                  <CardContent className="pt-6">
                    <div className="flex items-center space-x-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-orange-100 text-sm font-semibold text-orange-600">
                        3
                      </div>
                      <div>
                        <h3 className="font-semibold">Trial</h3>
                        <p className="text-sm text-slate-600">
                          Executable instance with specific participant and
                          conditions
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>

                <Card className="ml-24 border-l-4 border-l-purple-500">
                  <CardContent className="pt-6">
                    <div className="flex items-center space-x-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-purple-100 text-sm font-semibold text-purple-600">
                        4
                      </div>
                      <div>
                        <h3 className="font-semibold">Step</h3>
                        <p className="text-sm text-slate-600">
                          Distinct phase containing wizard or robot instructions
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>

                <Card className="ml-32 border-l-4 border-l-pink-500">
                  <CardContent className="pt-6">
                    <div className="flex items-center space-x-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-pink-100 text-sm font-semibold text-pink-600">
                        5
                      </div>
                      <div>
                        <h3 className="font-semibold">Action</h3>
                        <p className="text-sm text-slate-600">
                          Specific atomic task (speech, movement, input
                          gathering, etc.)
                        </p>
                      </div>
                    </div>
                  </CardContent>
                </Card>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="bg-gradient-to-r from-blue-600 to-purple-600 py-20">
        <div className="container mx-auto px-4">
          <div className="mx-auto max-w-4xl text-center text-white">
            <h2 className="mb-4 text-3xl font-bold">
              Ready to Revolutionize Your HRI Research?
            </h2>
            <p className="mb-8 text-xl opacity-90">
              Join researchers worldwide who are using our platform to conduct
              more rigorous, reproducible Wizard of Oz studies.
            </p>
            <div className="flex flex-col justify-center gap-4 sm:flex-row">
              <Button size="lg" variant="secondary" asChild>
                <Link href="/auth/signup">Get Started Free</Link>
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="border-white text-white hover:bg-white hover:text-blue-600"
                asChild
              >
                <Link href="/auth/signin">Sign In</Link>
              </Button>
            </div>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="bg-slate-900 py-12">
        <div className="container mx-auto px-4">
          <div className="text-center text-slate-400">
            <div className="mb-4 flex items-center justify-center">
              <Logo
                iconSize="md"
                showText={true}
                className="text-white [&>div]:bg-white [&>div]:text-blue-600"
              />
            </div>
            <p className="mb-4">
              Advancing Human-Robot Interaction research through standardized
              Wizard of Oz methodologies
            </p>
            <div className="flex justify-center space-x-6 text-sm">
              <Link href="#" className="transition-colors hover:text-white">
                Documentation
              </Link>
              <Link href="#" className="transition-colors hover:text-white">
                API Reference
              </Link>
              <Link href="#" className="transition-colors hover:text-white">
                Research Papers
              </Link>
              <Link href="#" className="transition-colors hover:text-white">
                Support
              </Link>
            </div>
          </div>
        </div>
      </footer>
    </main>
  );
}
