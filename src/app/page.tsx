import { getServerAuthSession } from "~/server/auth";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { Logo } from "~/components/logo";
import { HeroSection } from "~/components/home/hero-section";
import { FeaturesSection } from "~/components/home/features-section";
import { CTASection } from "~/components/home/cta-section";

export default async function Home() {
  const session = await getServerAuthSession();
  const isLoggedIn = !!session;

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

      {/* Content Sections */}
      <div className="relative">
        <HeroSection isLoggedIn={isLoggedIn} />

        {/* Dotted pattern for content sections */}
        <div className="relative">
          <div 
            className="absolute inset-0 pointer-events-none opacity-30"
            style={{
              backgroundImage: `
                radial-gradient(circle at 1px 1px, hsl(var(--muted-foreground)) 1px, transparent 0),
                linear-gradient(to bottom, transparent, hsl(var(--background)))
              `,
              backgroundSize: '32px 32px, 100% 100%',
              maskImage: 'linear-gradient(to bottom, transparent, black 10%, black 90%, transparent)',
            }}
          />

          <FeaturesSection />
          <CTASection isLoggedIn={isLoggedIn} />
        </div>
      </div>
    </div>
  );
}
