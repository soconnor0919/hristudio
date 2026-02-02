"use client";

import { DesignerRoot } from "~/components/experiments/designer/DesignerRoot";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import type { ExperimentStep } from "~/lib/experiment-designer/types";

interface DesignerPageClientProps {
  experiment: {
    id: string;
    name: string;
    description: string | null;
    study: {
      id: string;
      name: string;
    };
  };
  initialDesign?: {
    id: string;
    name: string;
    description: string;
    steps: ExperimentStep[];
    version: number;
    lastSaved: Date;
  };
}

export function DesignerPageClient({
  experiment,
  initialDesign,
}: DesignerPageClientProps) {
  // Set breadcrumbs
  useBreadcrumbsEffect([
    {
      label: "Dashboard",
      href: "/",
    },
    {
      label: "Studies",
      href: "/studies",
    },
    {
      label: experiment.study.name,
      href: `/studies/${experiment.study.id}`,
    },
    {
      label: "Experiments",
      href: `/studies/${experiment.study.id}/experiments`,
    },
    {
      label: experiment.name,
      href: `/studies/${experiment.study.id}/experiments/${experiment.id}`,
    },
    {
      label: "Designer",
    },
  ]);

  return (
    <div className="h-[calc(100vh-4rem-2rem)] w-full overflow-hidden border rounded-lg bg-background">
      <DesignerRoot experimentId={experiment.id} initialDesign={initialDesign} />
    </div>
  );
}
