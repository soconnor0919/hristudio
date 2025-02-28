"use client";

import { motion } from "framer-motion";
import { Sparkles, Brain, Microscope } from "lucide-react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";

const features = [
  {
    icon: <Sparkles className="size-6 text-primary" />,
    title: "Visual Experiment Design",
    description: "Create and configure experiments using an intuitive drag-and-drop interface without extensive coding."
  },
  {
    icon: <Brain className="size-6 text-primary" />,
    title: "Real-time Control",
    description: "Execute experiments with synchronized views for wizards and observers, enabling seamless collaboration."
  },
  {
    icon: <Microscope className="size-6 text-primary" />,
    title: "Comprehensive Analysis",
    description: "Record, playback, and analyze experimental data with built-in annotation and export tools."
  }
];

export function FeaturesSection() {
  return (
    <section className="container mx-auto px-4 py-24 space-y-12">
      <motion.div 
        initial={{ opacity: 0, y: 20 }}
        whileInView={{ opacity: 1, y: 0 }}
        viewport={{ once: true }}
        transition={{ duration: 0.5 }}
        className="text-center space-y-4"
      >
        <h2 className="text-3xl font-bold tracking-tight bg-gradient-to-br from-foreground to-foreground/70 bg-clip-text text-transparent inline-block">
          Powerful Features for HRI Research
        </h2>
        <p className="text-muted-foreground max-w-[600px] mx-auto">
          Everything you need to design, execute, and analyze your human-robot interaction experiments.
        </p>
      </motion.div>
      
      <div className="grid md:grid-cols-3 gap-8">
        {features.map((feature, index) => (
          <motion.div
            key={feature.title}
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ delay: index * 0.1, duration: 0.5 }}
          >
            <Card className="group relative overflow-hidden border bg-background/60 backdrop-blur supports-[backdrop-filter]:bg-background/60 hover:shadow-lg transition-all">
              <div className="pointer-events-none absolute inset-0 bg-gradient-to-br from-primary/10 via-primary/5 to-transparent opacity-0 group-hover:opacity-100 transition-opacity" />
              <CardHeader>
                <div className="size-12 rounded-lg bg-gradient-to-br from-primary/20 to-primary/10 flex items-center justify-center mb-4">
                  {feature.icon}
                </div>
                <CardTitle>{feature.title}</CardTitle>
                <CardDescription>{feature.description}</CardDescription>
              </CardHeader>
            </Card>
          </motion.div>
        ))}
      </div>
    </section>
  );
} 