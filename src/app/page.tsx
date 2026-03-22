import Link from "next/link";
import { redirect } from "next/navigation";
import { headers } from "next/headers";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Logo } from "~/components/ui/logo";
import { auth } from "~/lib/auth";
import {
  ArrowRight,
  Beaker,
  Bot,
  Database,
  LayoutTemplate,
  Lock,
  Network,
  PlayCircle,
  Settings2,
  Share2,
  Sparkles,
} from "lucide-react";

export default async function Home() {
  const session = await auth.api.getSession({
    headers: await headers(),
  });

  if (session?.user) {
    redirect("/dashboard");
  }

  return (
    <div className="bg-background text-foreground flex min-h-screen flex-col">
      {/* Navbar */}
      <header className="bg-background/80 sticky top-0 z-50 w-full border-b backdrop-blur-sm">
        <div className="container mx-auto flex h-16 items-center justify-between px-4">
          <Logo iconSize="md" showText={true} />
          <nav className="flex items-center gap-4">
            <Button variant="ghost" asChild className="hidden sm:inline-flex">
              <Link href="#features">Features</Link>
            </Button>
            <Button variant="ghost" asChild className="hidden sm:inline-flex">
              <Link href="#architecture">Architecture</Link>
            </Button>
            <div className="bg-border hidden h-6 w-px sm:block" />
            <Button variant="ghost" asChild>
              <Link href="/auth/signin">Sign In</Link>
            </Button>
            <Button asChild>
              <Link href="/auth/signup">Get Started</Link>
            </Button>
          </nav>
        </div>
      </header>

      <main className="flex-1">
        {/* Hero Section */}
        <section className="relative overflow-hidden pt-20 pb-32 md:pt-32">
          {/* Background Gradients */}
          <div className="bg-primary/20 absolute top-0 left-1/2 -z-10 h-[500px] w-[1000px] -translate-x-1/2 rounded-full opacity-30 blur-3xl dark:opacity-20" />

          <div className="container mx-auto flex flex-col items-center px-4 text-center">
            <Badge
              variant="secondary"
              className="mb-6 rounded-full px-4 py-1.5 text-sm font-medium"
            >
              <Sparkles className="mr-2 h-4 w-4 text-yellow-500" />
              The Modern Standard for HRI Research
            </Badge>

            <h1 className="max-w-4xl text-5xl font-extrabold tracking-tight sm:text-6xl md:text-7xl">
              Reproducible WoZ Studies <br className="hidden md:block" />
              <span className="bg-gradient-to-r from-blue-600 to-violet-600 bg-clip-text text-transparent dark:from-blue-400 dark:to-violet-400">
                Made Simple
              </span>
            </h1>

            <p className="text-muted-foreground mt-6 max-w-2xl text-lg md:text-xl">
              HRIStudio is the open-source platform that bridges the gap between
              ease of use and scientific rigor. Design, execute, and analyze
              human-robot interaction experiments with zero friction.
            </p>

            <div className="mt-10 flex flex-col gap-4 sm:flex-row sm:justify-center">
              <Button size="lg" className="h-12 px-8 text-base" asChild>
                <Link href="/auth/signup">
                  Start Researching
                  <ArrowRight className="ml-2 h-4 w-4" />
                </Link>
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="h-12 px-8 text-base"
                asChild
              >
                <Link
                  href="https://github.com/robolab/hristudio"
                  target="_blank"
                >
                  View on GitHub
                </Link>
              </Button>
            </div>

            {/* Mockup / Visual Interest */}
            <div className="bg-background/50 relative mt-20 w-full max-w-5xl rounded-xl border p-2 shadow-2xl backdrop-blur-sm lg:rounded-2xl lg:p-4">
              <div className="via-foreground/20 absolute inset-x-0 -top-px mx-auto h-px w-3/4 bg-gradient-to-r from-transparent to-transparent" />
              <div className="bg-muted/50 relative flex aspect-[16/9] w-full items-center justify-center overflow-hidden rounded-lg border">
                {/* Placeholder for actual app screenshot */}
                <div className="absolute inset-0 bg-gradient-to-tr from-blue-500/10 to-violet-500/10" />
                <div className="p-8 text-center">
                  <LayoutTemplate className="text-muted-foreground/50 mx-auto mb-4 h-16 w-16" />
                  <p className="text-muted-foreground font-medium">
                    Interactive Experiment Designer
                  </p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Features Bento Grid */}
        <section id="features" className="container mx-auto px-4 py-24">
          <div className="mb-12 text-center">
            <h2 className="text-3xl font-bold tracking-tight md:text-4xl">
              Everything You Need
            </h2>
            <p className="text-muted-foreground mt-4 text-lg">
              Built for the specific needs of HRI researchers and wizards.
            </p>
          </div>

          <div className="grid gap-4 md:grid-cols-3 lg:grid-cols-4 lg:grid-rows-2">
            {/* Visual Designer - Large Item */}
            <Card className="col-span-1 row-span-2 flex flex-col overflow-hidden bg-gradient-to-br from-blue-500/5 to-violet-500/5 md:col-span-2 lg:col-span-2 dark:from-blue-900/10 dark:to-violet-900/10">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <LayoutTemplate className="h-5 w-5 text-blue-500" />
                  Visual Experiment Designer
                </CardTitle>
              </CardHeader>
              <CardContent className="flex-1">
                <p className="text-muted-foreground mb-6">
                  Construct complex branching narratives without writing a
                  single line of code. Our node-based editor handles logic,
                  timing, and robot actions automatically.
                </p>
                <div className="bg-background/50 flex h-full min-h-[200px] items-center justify-center rounded-lg border p-4 shadow-inner">
                  <div className="text-muted-foreground flex items-center gap-2 text-sm">
                    <span className="bg-accent rounded p-2">Start</span>
                    <ArrowRight className="h-4 w-4" />
                    <span className="bg-primary/10 border-primary/20 text-primary rounded border p-2 font-medium">
                      Robot: Greet
                    </span>
                    <ArrowRight className="h-4 w-4" />
                    <span className="bg-accent rounded p-2">Wait: 5s</span>
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Robot Agnostic */}
            <Card className="col-span-1 md:col-span-1 lg:col-span-2">
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <Bot className="h-5 w-5 text-green-500" />
                  Robot Agnostic
                </CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground">
                  Switch between robots instantly. Whether it's a NAO, Pepper,
                  or a custom ROS2 bot, your experiment logic remains strictly
                  separated from hardware implementation.
                </p>
              </CardContent>
            </Card>

            {/* Role Based */}
            <Card className="bg-muted/30 col-span-1 md:col-span-1 lg:col-span-1">
              <CardHeader>
                <CardTitle className="flex items-center gap-2 text-base">
                  <Lock className="h-4 w-4 text-orange-500" />
                  Role-Based Access
                </CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground text-sm">
                  Granular permissions for Principal Investigators, Wizards, and
                  Observers.
                </p>
              </CardContent>
            </Card>

            {/* Data Logging */}
            <Card className="bg-muted/30 col-span-1 md:col-span-1 lg:col-span-1">
              <CardHeader>
                <CardTitle className="flex items-center gap-2 text-base">
                  <Database className="h-4 w-4 text-rose-500" />
                  Full Traceability
                </CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground text-sm">
                  Every wizard action, automated response, and sensor reading is
                  time-stamped and logged.
                </p>
              </CardContent>
            </Card>
          </div>
        </section>

        {/* Architecture Section */}
        <section id="architecture" className="bg-muted/30 border-t py-24">
          <div className="container mx-auto px-4">
            <div className="grid items-center gap-12 lg:grid-cols-2 lg:gap-8">
              <div>
                <h2 className="text-3xl font-bold tracking-tight">
                  Enterprise-Grade Architecture
                </h2>
                <p className="text-muted-foreground mt-4 text-lg">
                  Designed for reliability and scale. HRIStudio uses a modern
                  stack to ensure your data is safe and your experiments run
                  smoothly.
                </p>

                <div className="mt-8 space-y-4">
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Network className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">3-Layer Design</h3>
                      <p className="text-muted-foreground">
                        Clear separation between UI, Data, and Hardware layers
                        for maximum stability.
                      </p>
                    </div>
                  </div>
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Share2 className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">
                        Collaborative by Default
                      </h3>
                      <p className="text-muted-foreground">
                        Real-time state synchronization allows multiple
                        researchers to monitor a single trial.
                      </p>
                    </div>
                  </div>
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Settings2 className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">ROS2 Integration</h3>
                      <p className="text-muted-foreground">
                        Native support for ROS2 nodes, topics, and actions right
                        out of the box.
                      </p>
                    </div>
                  </div>
                </div>
              </div>

              <div className="relative mx-auto w-full max-w-[500px]">
                {/* Abstract representation of architecture */}
                <div className="relative z-10 space-y-4">
                  <Card className="relative left-0 cursor-default border-blue-500/20 bg-blue-500/5 transition-all hover:left-2">
                    <CardHeader className="pb-2">
                      <CardTitle className="font-mono text-sm text-blue-600 dark:text-blue-400">
                        APP LAYER
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <p className="font-semibold">
                        Next.js Dashboard + Experiment Designer
                      </p>
                    </CardContent>
                  </Card>
                  <Card className="relative left-4 cursor-default border-violet-500/20 bg-violet-500/5 transition-all hover:left-6">
                    <CardHeader className="pb-2">
                      <CardTitle className="font-mono text-sm text-violet-600 dark:text-violet-400">
                        DATA LAYER
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <p className="font-semibold">
                        PostgreSQL + MinIO + TRPC API
                      </p>
                    </CardContent>
                  </Card>
                  <Card className="relative left-8 cursor-default border-green-500/20 bg-green-500/5 transition-all hover:left-10">
                    <CardHeader className="pb-2">
                      <CardTitle className="font-mono text-sm text-green-600 dark:text-green-400">
                        HARDWARE LAYER
                      </CardTitle>
                    </CardHeader>
                    <CardContent>
                      <p className="font-semibold">
                        ROS2 Bridge + Robot Plugins
                      </p>
                    </CardContent>
                  </Card>
                </div>
                {/* Decorative blobs */}
                <div className="bg-primary/10 absolute top-1/2 left-1/2 -z-10 h-[300px] w-[300px] -translate-x-1/2 -translate-y-1/2 rounded-full blur-3xl" />
              </div>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className="container mx-auto px-4 py-24 text-center">
          <h2 className="text-3xl font-bold tracking-tight md:text-4xl">
            Ready to upgrade your lab?
          </h2>
          <p className="text-muted-foreground mx-auto mt-4 max-w-2xl text-lg">
            Join the community of researchers building the future of HRI with
            reproducible, open-source tools.
          </p>
          <div className="mt-8">
            <Button
              size="lg"
              className="shadow-primary/20 h-12 px-8 text-base shadow-lg"
              asChild
            >
              <Link href="/auth/signup">Get Started for Free</Link>
            </Button>
          </div>
        </section>
      </main>

      <footer className="bg-muted/40 border-t py-12">
        <div className="container mx-auto flex flex-col items-center justify-between gap-6 px-4 text-center md:flex-row md:text-left">
          <div className="flex flex-col gap-2">
            <Logo iconSize="sm" showText={true} />
            <p className="text-muted-foreground text-sm">
              &copy; {new Date().getFullYear()} HRIStudio. All rights reserved.
            </p>
          </div>
          <div className="text-muted-foreground flex gap-6 text-sm">
            <Link href="#" className="hover:text-foreground">
              Privacy
            </Link>
            <Link href="#" className="hover:text-foreground">
              Terms
            </Link>
            <Link href="#" className="hover:text-foreground">
              GitHub
            </Link>
            <Link href="#" className="hover:text-foreground">
              Documentation
            </Link>
          </div>
        </div>
      </footer>
    </div>
  );
}
