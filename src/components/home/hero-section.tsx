"use client";

import { motion } from "framer-motion";
import { BotIcon, ArrowRight } from "lucide-react";
import Link from "next/link";
import { Button } from "~/components/ui/button";

interface HeroSectionProps {
  isLoggedIn: boolean;
}

export function HeroSection({ isLoggedIn }: HeroSectionProps) {
  return (
    <section className="relative">
      {/* Hero gradient background */}
      <div className="absolute inset-0 bg-gradient-to-b from-background via-primary/5 to-background">
        <div className="absolute inset-0" 
          style={{
            backgroundImage: `radial-gradient(circle at 50% 50%, hsl(var(--primary)/.08) 0%, transparent 50%)`,
          }}
        />
      </div>

      <div className="container mx-auto px-4 py-24 relative">
        <motion.div 
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          className="grid lg:grid-cols-2 gap-12 items-center"
        >
          <div className="space-y-6">
            <motion.div 
              initial={{ opacity: 0, y: 10 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2, duration: 0.5 }}
              className="inline-flex rounded-lg bg-gradient-to-br from-primary/20 via-secondary/20 to-background p-1 mb-8"
            >
              <span className="rounded-md bg-background/95 px-3 py-1 text-sm backdrop-blur">
                Now with Visual Experiment Designer
              </span>
            </motion.div>
            <motion.h1 
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.3, duration: 0.5 }}
              className="text-4xl font-bold tracking-tight lg:text-6xl bg-gradient-to-br from-foreground via-foreground/90 to-foreground/70 bg-clip-text text-transparent"
            >
              Streamline Your HRI Research
            </motion.h1>
            <motion.p 
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.4, duration: 0.5 }}
              className="text-xl text-muted-foreground"
            >
              A comprehensive platform for designing, executing, and analyzing Wizard-of-Oz experiments in human-robot interaction studies.
            </motion.p>
            <motion.div 
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.5, duration: 0.5 }}
              className="flex flex-col sm:flex-row gap-4 pt-4"
            >
              {!isLoggedIn ? (
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
            </motion.div>
          </div>
          <motion.div 
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ delay: 0.4, duration: 0.5 }}
            className="relative aspect-square lg:aspect-video"
          >
            <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-secondary/20 to-background rounded-lg border shadow-xl" />
            <div className="absolute inset-0 flex items-center justify-center">
              <BotIcon className="h-32 w-32 text-primary/40" />
            </div>
          </motion.div>
        </motion.div>
      </div>
    </section>
  );
} 