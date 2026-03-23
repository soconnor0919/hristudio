import Link from "next/link";
import Image from "next/image";
import { redirect } from "next/navigation";
import { headers } from "next/headers";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Logo } from "~/components/ui/logo";
import { auth } from "~/lib/auth";
import {
  ArrowRight,
  Bot,
  Database,
  LayoutTemplate,
  Lock,
  Network,
  Settings2,
  Share2,
  Sparkles,
  Users,
  Beaker,
  FileText,
  PlayCircle,
} from "lucide-react";

const screenshots = [
  {
    src: "/images/screenshots/experiment-designer.png",
    alt: "Visual Experiment Designer",
    label: "Design",
    className: "md:col-span-2 md:row-span-2",
  },
  {
    src: "/images/screenshots/wizard-interface.png",
    alt: "Wizard Execution Interface",
    label: "Execute",
    className: "",
  },
  {
    src: "/images/screenshots/dashboard.png",
    alt: "Study Dashboard",
    label: "Dashboard",
    className: "",
  },
];

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
              <Link href="#how-it-works">How It Works</Link>
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
        <section className="relative overflow-hidden pt-20 pb-24 md:pt-32">
          <div className="bg-primary/20 absolute top-0 left-1/2 -z-10 h-[500px] w-[1000px] -translate-x-1/2 rounded-full opacity-30 blur-3xl dark:opacity-20" />

          <div className="container mx-auto flex flex-col items-center px-4 text-center">
            <Badge
              variant="secondary"
              className="mb-6 rounded-full px-4 py-1.5 text-sm font-medium"
            >
              <Sparkles className="mr-2 h-4 w-4 text-yellow-500" />
              Open Source WoZ Platform
            </Badge>

            <h1 className="max-w-4xl text-5xl font-extrabold tracking-tight sm:text-6xl md:text-7xl">
              Wizard-of-Oz Studies <br className="hidden md:block" />
              <span className="bg-gradient-to-r from-cyan-500 to-blue-600 bg-clip-text text-transparent">
                Made Scientific
              </span>
            </h1>

            <p className="text-muted-foreground mt-6 max-w-2xl text-lg md:text-xl">
              HRIStudio is the open-source platform that makes human-robot
              interaction research reproducible, accessible, and collaborative.
              Design experiments, control robots, and analyze results — all in
              one place.
            </p>

            <div className="mt-10 flex flex-col gap-4 sm:flex-row sm:justify-center">
              <Button size="lg" className="h-12 px-8 text-base" asChild>
                <Link href="/auth/signup">
                  Start Your Research
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
          </div>
        </section>

        {/* Screenshots Section */}
        <section id="screenshots" className="container mx-auto px-4 py-12">
          <div className="grid gap-4 md:grid-cols-3">
            {screenshots.map((screenshot, index) => (
              <div
                key={index}
                className={`group bg-muted/50 relative overflow-hidden rounded-xl border ${screenshot.className}`}
              >
                {/* Placeholder - replace src with actual screenshot */}
                <div className="from-muted to-muted/50 absolute inset-0 flex flex-col items-center justify-center bg-gradient-to-br">
                  <div className="bg-background/80 mb-4 rounded-lg px-4 py-2 shadow-sm">
                    <span className="text-muted-foreground text-xs font-medium tracking-wider uppercase">
                      {screenshot.label}
                    </span>
                  </div>
                  <FileText className="text-muted-foreground/30 h-16 w-16" />
                  <p className="text-muted-foreground/50 mt-4 text-sm">
                    Screenshot: {screenshot.alt}
                  </p>
                  <p className="text-muted-foreground/30 mt-1 text-xs">
                    Replace with actual image
                  </p>
                </div>
                {/* Uncomment when you have real screenshots:
                <Image
                  src={screenshot.src}
                  alt={screenshot.alt}
                  fill
                  className="object-cover transition-transform group-hover:scale-105"
                />
                */}
              </div>
            ))}
          </div>
          <p className="text-muted-foreground mt-4 text-center text-sm">
            Add screenshots to{" "}
            <code className="bg-muted rounded px-2 py-1">
              public/images/screenshots/
            </code>
          </p>
        </section>

        {/* Features Section */}
        <section id="features" className="bg-muted/30 border-t py-24">
          <div className="container mx-auto px-4">
            <div className="mb-16 text-center">
              <h2 className="text-3xl font-bold tracking-tight md:text-4xl">
                Built for Scientific Rigor
              </h2>
              <p className="text-muted-foreground mx-auto mt-4 max-w-2xl text-lg">
                Everything you need to conduct reproducible Wizard-of-Oz
                studies, from experiment design to data analysis.
              </p>
            </div>

            <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
              <FeatureCard
                icon={LayoutTemplate}
                title="Visual Experiment Designer"
                description="Build complex branching narratives with drag-and-drop blocks. No coding required — just drag, configure, and run."
                color="blue"
              />
              <FeatureCard
                icon={PlayCircle}
                title="Guided Wizard Interface"
                description="Step-by-step protocol execution keeps wizards on track. Every action is logged with timestamps."
                color="violet"
              />
              <FeatureCard
                icon={Bot}
                title="Robot Agnostic"
                description="Design experiments once, run on any robot. NAO, Pepper, TurtleBot — your logic stays the same."
                color="green"
              />
              <FeatureCard
                icon={Users}
                title="Role-Based Collaboration"
                description="Invite PIs, wizards, and observers. Each role sees exactly what they need — nothing more."
                color="orange"
              />
              <FeatureCard
                icon={Database}
                title="Automatic Data Logging"
                description="Every action, timestamp, and sensor reading is captured. Export to CSV for analysis."
                color="rose"
              />
              <FeatureCard
                icon={Lock}
                title="Built-in Reproducibility"
                description="Protocol/trial separation, deviation logging, and comprehensive audit trails make replication trivial."
                color="cyan"
              />
            </div>
          </div>
        </section>

        {/* How It Works */}
        <section id="how-it-works" className="container mx-auto px-4 py-24">
          <div className="mb-16 text-center">
            <h2 className="text-3xl font-bold tracking-tight md:text-4xl">
              How It Works
            </h2>
            <p className="text-muted-foreground mt-4 text-lg">
              From design to publication in one unified workflow.
            </p>
          </div>

          <div className="relative">
            {/* Connection line */}
            <div className="bg-border absolute top-0 left-1/2 hidden h-full w-px -translate-x-1/2 lg:block" />

            <div className="space-y-12 lg:space-y-0">
              <WorkflowStep
                number={1}
                title="Design"
                description="Use the visual editor to build your experiment protocol with drag-and-drop blocks. Add speech, gestures, conditions, and branching logic — no code required."
                icon={LayoutTemplate}
              />
              <WorkflowStep
                number={2}
                title="Configure"
                description="Set up your study, invite team members with appropriate roles, and configure your robot platform."
                icon={Settings2}
              />
              <WorkflowStep
                number={3}
                title="Execute"
                description="Run trials with the wizard interface. Real-time updates keep everyone in sync. Every action is automatically logged."
                icon={PlayCircle}
              />
              <WorkflowStep
                number={4}
                title="Analyze"
                description="Review trial data, export responses, and compare across participants. Everything is timestamped and synchronized."
                icon={Share2}
              />
            </div>
          </div>
        </section>

        {/* Architecture Section */}
        <section id="architecture" className="bg-muted/30 border-t py-24">
          <div className="container mx-auto px-4">
            <div className="grid items-center gap-12 lg:grid-cols-2">
              <div>
                <h2 className="text-3xl font-bold tracking-tight">
                  Modern Architecture
                </h2>
                <p className="text-muted-foreground mt-4 text-lg">
                  Built on proven technologies for reliability, type safety, and
                  real-time collaboration.
                </p>

                <div className="mt-8 space-y-4">
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Network className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">3-Layer Design</h3>
                      <p className="text-muted-foreground text-sm">
                        UI, application logic, and hardware layers are strictly
                        separated for stability.
                      </p>
                    </div>
                  </div>
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Share2 className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">Real-Time Sync</h3>
                      <p className="text-muted-foreground text-sm">
                        WebSocket updates keep wizard and observer views
                        perfectly synchronized.
                      </p>
                    </div>
                  </div>
                  <div className="flex gap-4">
                    <div className="bg-background flex h-10 w-10 shrink-0 items-center justify-center rounded-lg border shadow-sm">
                      <Beaker className="text-primary h-5 w-5" />
                    </div>
                    <div>
                      <h3 className="font-semibold">Plugin System</h3>
                      <p className="text-muted-foreground text-sm">
                        Extend with custom robot integrations and actions
                        through a simple JSON configuration.
                      </p>
                    </div>
                  </div>
                </div>
              </div>

              <div className="relative space-y-4">
                <Card className="border-blue-500/20 bg-blue-500/5">
                  <CardHeader className="pb-2">
                    <CardTitle className="font-mono text-sm text-blue-600 dark:text-blue-400">
                      APP LAYER
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <p className="text-sm font-medium">
                      Next.js + React + tRPC
                    </p>
                    <p className="text-muted-foreground text-xs">
                      Type-safe full-stack
                    </p>
                  </CardContent>
                </Card>
                <Card className="border-violet-500/20 bg-violet-500/5">
                  <CardHeader className="pb-2">
                    <CardTitle className="font-mono text-sm text-violet-600 dark:text-violet-400">
                      DATA LAYER
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <p className="text-sm font-medium">
                      PostgreSQL + MinIO + WebSocket
                    </p>
                    <p className="text-muted-foreground text-xs">
                      Persistent storage + real-time
                    </p>
                  </CardContent>
                </Card>
                <Card className="border-green-500/20 bg-green-500/5">
                  <CardHeader className="pb-2">
                    <CardTitle className="font-mono text-sm text-green-600 dark:text-green-400">
                      ROBOT LAYER
                    </CardTitle>
                  </CardHeader>
                  <CardContent>
                    <p className="text-sm font-medium">
                      ROS2 Bridge + Plugin Config
                    </p>
                    <p className="text-muted-foreground text-xs">
                      Platform agnostic
                    </p>
                  </CardContent>
                </Card>
              </div>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className="container mx-auto px-4 py-24 text-center">
          <div className="mx-auto max-w-2xl">
            <h2 className="text-3xl font-bold tracking-tight md:text-4xl">
              Ready to upgrade your research?
            </h2>
            <p className="text-muted-foreground mt-4 text-lg">
              Join researchers building reproducible HRI studies with
              open-source tools.
            </p>
            <div className="mt-8 flex flex-col gap-4 sm:flex-row sm:justify-center">
              <Button
                size="lg"
                className="shadow-primary/20 h-12 px-8 text-base shadow-lg"
                asChild
              >
                <Link href="/auth/signup">Get Started Free</Link>
              </Button>
              <Button
                size="lg"
                variant="outline"
                className="h-12 px-8 text-base"
                asChild
              >
                <Link href="/docs" target="_blank">
                  Read the Docs
                </Link>
              </Button>
            </div>
          </div>
        </section>
      </main>

      <footer className="bg-muted/40 border-t py-12">
        <div className="container mx-auto flex flex-col items-center justify-between gap-6 px-4 text-center md:flex-row md:text-left">
          <div className="flex flex-col gap-2">
            <Logo iconSize="sm" showText={true} />
            <p className="text-muted-foreground text-sm">
              &copy; {new Date().getFullYear()} HRIStudio. Open source under MIT
              License.
            </p>
          </div>
          <div className="text-muted-foreground flex gap-6 text-sm">
            <Link href="/docs" className="hover:text-foreground">
              Docs
            </Link>
            <Link
              href="https://github.com/robolab/hristudio"
              className="hover:text-foreground"
              target="_blank"
            >
              GitHub
            </Link>
            <Link href="#" className="hover:text-foreground">
              Privacy
            </Link>
            <Link href="#" className="hover:text-foreground">
              Terms
            </Link>
          </div>
        </div>
      </footer>
    </div>
  );
}

function FeatureCard({
  icon: Icon,
  title,
  description,
  color,
}: {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  description: string;
  color: "blue" | "violet" | "green" | "orange" | "rose" | "cyan";
}) {
  const colors = {
    blue: "text-blue-500 bg-blue-500/10",
    violet: "text-violet-500 bg-violet-500/10",
    green: "text-green-500 bg-green-500/10",
    orange: "text-orange-500 bg-orange-500/10",
    rose: "text-rose-500 bg-rose-500/10",
    cyan: "text-cyan-500 bg-cyan-500/10",
  };

  return (
    <Card>
      <CardHeader>
        <div
          className={`mb-2 inline-flex h-10 w-10 items-center justify-center rounded-lg ${colors[color]}`}
        >
          <Icon className="h-5 w-5" />
        </div>
        <CardTitle className="text-lg">{title}</CardTitle>
      </CardHeader>
      <CardContent>
        <p className="text-muted-foreground text-sm">{description}</p>
      </CardContent>
    </Card>
  );
}

function WorkflowStep({
  number,
  title,
  description,
  icon: Icon,
}: {
  number: number;
  title: string;
  description: string;
  icon: React.ComponentType<{ className?: string }>;
}) {
  return (
    <div className="relative flex flex-col items-center gap-4 lg:flex-row lg:gap-8">
      <div className="border-primary bg-background text-primary z-10 flex h-12 w-12 shrink-0 items-center justify-center rounded-full border-2 font-bold">
        {number}
      </div>
      <Card className="flex-1">
        <CardHeader className="flex flex-row items-center gap-4 pb-2">
          <Icon className="text-primary h-5 w-5" />
          <CardTitle>{title}</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-muted-foreground">{description}</p>
        </CardContent>
      </Card>
    </div>
  );
}
