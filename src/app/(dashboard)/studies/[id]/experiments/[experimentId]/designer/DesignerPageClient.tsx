"use client";

import { useMemo } from "react";
import { DesignerRoot } from "~/components/experiments/designer/DesignerRoot";
import { useActionRegistry } from "~/components/experiments/designer/ActionRegistry";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import type { ExperimentStep } from "~/lib/experiment-designer/types";

interface DesignerPageClientProps {
  experiment: {
    id: string;
    name: string;
    description: string | null;
    status: string;
    studyId: string;
    createdAt: Date;
    updatedAt: Date;
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
  // Initialize action registry early to prevent CLS
  useActionRegistry();

  // Calculate design statistics
  const designStats = useMemo(() => {
    if (!initialDesign) return undefined;

    const stepCount = initialDesign.steps.length;
    const actionCount = initialDesign.steps.reduce(
      (sum, step) => sum + step.actions.length,
      0
    );

    return { stepCount, actionCount };
  }, [initialDesign]);

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
    <DesignerRoot
      experimentId={experiment.id}
      initialDesign={initialDesign}
      experiment={experiment}
      designStats={designStats}
    />
  );
}
