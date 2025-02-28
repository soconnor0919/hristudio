"use client";

import { motion } from "framer-motion";
import { BotIcon } from "lucide-react";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import { Card, CardContent } from "~/components/ui/card";

interface CTASectionProps {
  isLoggedIn: boolean;
}

export function CTASection({ isLoggedIn }: CTASectionProps) {
  return (
    <section className="container mx-auto px-4 py-24">
      <motion.div
        initial={{ opacity: 0, y: 20 }}
        whileInView={{ opacity: 1, y: 0 }}
        viewport={{ once: true }}
        transition={{ duration: 0.5 }}
      >
        <Card className="relative overflow-hidden">
          <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary via-primary to-secondary" />
          <div className="pointer-events-none absolute inset-0 bg-[radial-gradient(circle_at_50%_120%,rgba(0,0,0,0)_30%,rgba(0,0,0,0.15)_100%)]" />
          <CardContent className="relative p-12 flex flex-col items-center text-center space-y-6 text-primary-foreground">
            <motion.div
              initial={{ scale: 0.8, opacity: 0 }}
              whileInView={{ scale: 1, opacity: 1 }}
              viewport={{ once: true }}
              transition={{ delay: 0.2, duration: 0.5 }}
            >
              <BotIcon className="size-12 mb-4" />
            </motion.div>
            <motion.h2
              initial={{ y: 20, opacity: 0 }}
              whileInView={{ y: 0, opacity: 1 }}
              viewport={{ once: true }}
              transition={{ delay: 0.3, duration: 0.5 }}
              className="text-3xl font-bold tracking-tight"
            >
              Ready to Transform Your Research?
            </motion.h2>
            <motion.p
              initial={{ y: 20, opacity: 0 }}
              whileInView={{ y: 0, opacity: 1 }}
              viewport={{ once: true }}
              transition={{ delay: 0.4, duration: 0.5 }}
              className="text-primary-foreground/90 max-w-[600px]"
            >
              Join the growing community of researchers using HRIStudio to advance human-robot interaction studies.
            </motion.p>
            <motion.div
              initial={{ y: 20, opacity: 0 }}
              whileInView={{ y: 0, opacity: 1 }}
              viewport={{ once: true }}
              transition={{ delay: 0.5, duration: 0.5 }}
            >
              {!isLoggedIn ? (
                <Button size="lg" variant="secondary" asChild className="mt-4 bg-background/20 hover:bg-background/30">
                  <Link href="/auth/signup">Start Your Journey</Link>
                </Button>
              ) : (
                <Button size="lg" variant="secondary" asChild className="mt-4 bg-background/20 hover:bg-background/30">
                  <Link href="/dashboard">Go to Dashboard</Link>
                </Button>
              )}
            </motion.div>
          </CardContent>
        </Card>
      </motion.div>
    </section>
  );
}