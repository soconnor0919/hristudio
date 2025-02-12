import { getServerAuthSession } from "~/server/auth";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { BotIcon, ArrowRight, Sparkles, Brain, Microscope } from "lucide-react";
import { Logo } from "~/components/logo";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";

export default async function Home() {
  const session = await getServerAuthSession();

  return (
    <div className="min-h-screen bg-background relative">
      {/* Background Gradients */}
      <div className="pointer-events-none fixed inset-0 flex items-center justify-center opacity-40">
        <div className="h-[800px] w-[800px] rounded-full bg-gradient-to-r from-primary/20 via-secondary/20 to-background blur-3xl" />
      </div>

      {/* Navigation Bar */}
      <nav className="sticky top-0 z-50 border-b bg-background/50 backdrop-blur supports-[backdrop-filter]:bg-background/50">
        <div className="container mx-auto px-4 h-16 flex items-center justify-between">
          <div className="flex items-center space-x-2">
            <Logo />
          </div>
          <div className="flex items-center space-x-4">
            {!session && (
              <>
                <Button variant="ghost" asChild>
                  <Link href="/auth/signin">Sign In</Link>
                </Button>
                <Button asChild>
                  <Link href="/auth/signup">Sign Up</Link>
                </Button>
              </>
            )}
            {session && (
              <Button asChild>
                <Link href="/dashboard">Dashboard</Link>
              </Button>
            )}
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="container mx-auto px-4 py-24">
        <div className="grid lg:grid-cols-2 gap-12 items-center">
          <div className="space-y-6">
            <div className="inline-flex rounded-lg bg-gradient-to-br from-primary/20 via-secondary/20 to-background p-1 mb-8">
              <span className="rounded-md bg-background/95 px-3 py-1 text-sm backdrop-blur">
                Now with Visual Experiment Designer
              </span>
            </div>
            <h1 className="text-4xl font-bold tracking-tight lg:text-6xl bg-gradient-to-br from-foreground via-foreground/90 to-foreground/70 bg-clip-text text-transparent">
              Streamline Your HRI Research
            </h1>
            <p className="text-xl text-muted-foreground">
              A comprehensive platform for designing, executing, and analyzing Wizard-of-Oz experiments in human-robot interaction studies.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 pt-4">
              {!session ? (
                <Button size="lg" className="w-full sm:w-auto group bg-gradient-to-r from-primary to-primary hover:from-primary/90 hover:to-primary" asChild>
                  <Link href="/auth/signup">
                    Get Started
                    <ArrowRight className="ml-2 h-4 w-4 transition-transform group-hover:translate-x-1" />
                  </Link>
                </Button>
              ) : (
                <Button size="lg" className="w-full sm:w-auto group bg-gradient-to-r from-primary to-primary hover:from-primary/90 hover:to-primary" asChild>
                  <Link href="/dashboard">
                    Go to Dashboard
                    <ArrowRight className="ml-2 h-4 w-4 transition-transform group-hover:translate-x-1" />
                  </Link>
                </Button>
              )}
              <Button size="lg" variant="outline" className="w-full sm:w-auto" asChild>
                <Link href="https://github.com/soconnor0919/hristudio" target="_blank">
                  View on GitHub
                </Link>
              </Button>
            </div>
          </div>
          <div className="relative aspect-square lg:aspect-video">
            <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-secondary/20 to-background rounded-lg border shadow-xl" />
            <div className="absolute inset-0 flex items-center justify-center">
              <BotIcon className="h-32 w-32 text-primary/40" />
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="container mx-auto px-4 py-24 space-y-12">
        <div className="text-center space-y-4">
          <h2 className="text-3xl font-bold tracking-tight bg-gradient-to-br from-foreground to-foreground/70 bg-clip-text text-transparent inline-block">
            Powerful Features for HRI Research
          </h2>
          <p className="text-muted-foreground max-w-[600px] mx-auto">
            Everything you need to design, execute, and analyze your human-robot interaction experiments.
          </p>
        </div>
        
        <div className="grid md:grid-cols-3 gap-8">
          <Card className="group relative overflow-hidden border bg-background/60 backdrop-blur supports-[backdrop-filter]:bg-background/60 hover:shadow-lg transition-all">
            <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary/10 via-primary/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
            <CardHeader>
              <div className="size-12 rounded-lg bg-gradient-to-br from-primary/20 to-primary/10 flex items-center justify-center mb-4">
                <Sparkles className="size-6 text-primary" />
              </div>
              <CardTitle>Visual Experiment Design</CardTitle>
              <CardDescription>
                Create and configure experiments using an intuitive drag-and-drop interface without extensive coding.
              </CardDescription>
            </CardHeader>
          </Card>

          <Card className="group relative overflow-hidden border bg-background/60 backdrop-blur supports-[backdrop-filter]:bg-background/60 hover:shadow-lg transition-all">
            <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary/10 via-primary/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
            <CardHeader>
              <div className="size-12 rounded-lg bg-gradient-to-br from-primary/20 to-primary/10 flex items-center justify-center mb-4">
                <Brain className="size-6 text-primary" />
              </div>
              <CardTitle>Real-time Control</CardTitle>
              <CardDescription>
                Execute experiments with synchronized views for wizards and observers, enabling seamless collaboration.
              </CardDescription>
            </CardHeader>
          </Card>

          <Card className="group relative overflow-hidden border bg-background/60 backdrop-blur supports-[backdrop-filter]:bg-background/60 hover:shadow-lg transition-all">
            <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary/10 via-primary/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
            <CardHeader>
              <div className="size-12 rounded-lg bg-gradient-to-br from-primary/20 to-primary/10 flex items-center justify-center mb-4">
                <Microscope className="size-6 text-primary" />
              </div>
              <CardTitle>Comprehensive Analysis</CardTitle>
              <CardDescription>
                Record, playback, and analyze experimental data with built-in annotation and export tools.
              </CardDescription>
            </CardHeader>
          </Card>
        </div>
      </section>

      {/* CTA Section */}
      <section className="container mx-auto px-4 py-24">
        <Card className="relative overflow-hidden">
          <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary via-primary to-secondary" />
          <div className="pointer-events-none absolute inset-0 bg-[radial-gradient(circle_at_50%_120%,rgba(0,0,0,0)_30%,rgba(0,0,0,0.15)_100%)]" />
          <CardContent className="relative p-12 flex flex-col items-center text-center space-y-6 text-primary-foreground">
            <BotIcon className="size-12 mb-4" />
            <h2 className="text-3xl font-bold tracking-tight">
              Ready to Transform Your Research?
            </h2>
            <p className="text-primary-foreground/90 max-w-[600px]">
              Join the growing community of researchers using HRIStudio to advance human-robot interaction studies.
            </p>
            {!session ? (
              <Button size="lg" variant="secondary" asChild className="mt-4 bg-background/20 hover:bg-background/30">
                <Link href="/auth/signup">Start Your Journey</Link>
              </Button>
            ) : (
              <Button size="lg" variant="secondary" asChild className="mt-4 bg-background/20 hover:bg-background/30">
                <Link href="/dashboard">Go to Dashboard</Link>
              </Button>
            )}
          </CardContent>
        </Card>
      </section>
    </div>
  );
}
