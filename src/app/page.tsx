import { getServerAuthSession } from "~/server/auth";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { BotIcon } from "lucide-react";
import { Logo } from "~/components/logo";

export default async function Home() {
  const session = await getServerAuthSession();

  return (
    <div className="min-h-screen bg-background">
      {/* Navigation Bar */}
      <nav className="border-b bg-card/50 backdrop-blur supports-[backdrop-filter]:bg-card/50">
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
      <section className="container mx-auto px-4 py-24 grid lg:grid-cols-2 gap-12 items-center">
        <div>
          <h1 className="text-4xl font-bold tracking-tight lg:text-6xl">
            Streamline Your HRI Research
          </h1>
          <p className="mt-6 text-xl text-muted-foreground">
            A comprehensive platform for designing, executing, and analyzing Wizard-of-Oz experiments in human-robot interaction studies.
          </p>
          <div className="mt-8 flex flex-col sm:flex-row gap-4">
            {!session ? (
              <Button size="lg" className="w-full sm:w-auto" asChild>
                <Link href="/auth/signup">Get Started</Link>
              </Button>
            ) : (
              <Button size="lg" className="w-full sm:w-auto" asChild>
                <Link href="/dashboard">Go to Dashboard</Link>
              </Button>
            )}
            <Button size="lg" variant="outline" className="w-full sm:w-auto" asChild>
              <Link href="https://github.com/soconnor0919/hristudio" target="_blank">
                View on GitHub
              </Link>
            </Button>
          </div>
        </div>
        <div className="relative aspect-video">
          <div className="absolute inset-0 bg-gradient-to-br from-primary/20 to-secondary/20 rounded-lg" />
          <div className="absolute inset-0 flex items-center justify-center">
            <BotIcon className="h-32 w-32 text-primary/40" />
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="container mx-auto px-4 py-24">
        <div className="grid md:grid-cols-3 gap-8">
          <div className="space-y-4">
            <h3 className="text-xl font-semibold">Visual Experiment Design</h3>
            <p className="text-muted-foreground">
              Create and configure experiments using an intuitive drag-and-drop interface without extensive coding.
            </p>
          </div>
          <div className="space-y-4">
            <h3 className="text-xl font-semibold">Real-time Control</h3>
            <p className="text-muted-foreground">
              Execute experiments with synchronized views for wizards and observers, enabling seamless collaboration.
            </p>
          </div>
          <div className="space-y-4">
            <h3 className="text-xl font-semibold">Comprehensive Analysis</h3>
            <p className="text-muted-foreground">
              Record, playback, and analyze experimental data with built-in annotation and export tools.
            </p>
          </div>
        </div>
      </section>
    </div>
  );
}
