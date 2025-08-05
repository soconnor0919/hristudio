"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { X, ArrowLeft } from "lucide-react";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import {
  FlowDesigner,
  type FlowDesign,
  type FlowStep,
  type StepType,
} from "./FlowDesigner";

interface ExperimentDesignerClientProps {
  experiment: {
    id: string;
    name: string;
    description: string;
    studyId: string;
    study?: {
      name: string;
    };
  };
}

export function ExperimentDesignerClient({
  experiment,
}: ExperimentDesignerClientProps) {
  const [saveError, setSaveError] = useState<string | null>(null);
  const router = useRouter();

  // Set breadcrumbs for the designer
  useBreadcrumbsEffect([
    { label: "Studies", href: "/studies" },
    {
      label: experiment.study?.name ?? "Study",
      href: `/studies/${experiment.studyId}`,
    },
    {
      label: "Experiments",
      href: `/studies/${experiment.studyId}`,
    },
    {
      label: experiment.name,
      href: `/experiments/${experiment.id}`,
    },
    {
      label: "Designer",
      href: `/experiments/${experiment.id}/designer`,
    },
  ]);

  // Fetch the experiment's design data
  const { data: experimentSteps, isLoading } =
    api.experiments.getSteps.useQuery({
      experimentId: experiment.id,
    });

  const saveDesignMutation = api.experiments.saveDesign.useMutation({
    onSuccess: () => {
      setSaveError(null);
      toast.success("Experiment design saved successfully");
    },
    onError: (error) => {
      setSaveError(error.message);
      toast.error(`Failed to save design: ${error.message}`);
    },
  });

  const handleSave = async (design: FlowDesign) => {
    try {
      await saveDesignMutation.mutateAsync({
        experimentId: experiment.id,
        steps: design.steps
          .filter((step) => step.type !== "start" && step.type !== "end") // Filter out start/end nodes
          .map((step) => ({
            id: step.id,
            type: step.type as "wizard" | "robot" | "parallel" | "conditional",
            name: step.name,
            order: Math.floor(step.position.x / 250) + 1, // Calculate order from position
            parameters: step.parameters,
            description: step.description,
            duration: step.duration,
            actions: step.actions,
            expanded: false,
            children: [],
            parentId: undefined,
          })),
        version: design.version,
      });
    } catch (error) {
      console.error("Failed to save design:", error);
      throw error;
    }
  };

  if (isLoading) {
    return (
      <div className="flex min-h-[600px] items-center justify-center">
        <div className="text-center">
          <div className="border-primary mx-auto mb-4 h-8 w-8 animate-spin rounded-full border-b-2"></div>
          <p className="text-muted-foreground">
            Loading experiment designer...
          </p>
        </div>
      </div>
    );
  }

  // Convert backend steps to flow format
  const convertToFlowSteps = (steps: any[]): FlowStep[] => {
    return steps.map((step, index) => ({
      id: step.id,
      type: step.type as StepType,
      name: step.name,
      description: step.description ?? undefined,
      duration: step.duration ?? undefined,
      actions: [], // Actions will be loaded separately if needed
      parameters: step.parameters ?? {},
      position: {
        x: index * 250 + 100,
        y: 100,
      },
    }));
  };

  const initialDesign: FlowDesign = {
    id: experiment.id,
    name: experiment.name,
    description: experiment.description,
    steps: experimentSteps ? convertToFlowSteps(experimentSteps) : [],
    version: 1,
    lastSaved: new Date(),
  };

  return (
    <div className="bg-background flex h-screen flex-col">
      {/* Header */}
      <div className="bg-background/95 supports-[backdrop-filter]:bg-background/60 relative border-b backdrop-blur">
        <div className="from-primary/5 to-accent/5 absolute inset-0 bg-gradient-to-r" />
        <div className="relative flex items-center justify-between p-6">
          <div className="flex items-center space-x-4">
            <Button
              variant="ghost"
              size="sm"
              onClick={() => router.push(`/experiments/${experiment.id}`)}
            >
              <ArrowLeft className="mr-2 h-4 w-4" />
              Back to Experiment
            </Button>
            <div className="bg-border h-6 w-px" />
            <div className="bg-primary flex h-12 w-12 items-center justify-center rounded-xl shadow-lg">
              <span className="text-primary-foreground text-xl font-bold">
                F
              </span>
            </div>
            <div>
              <h1 className="text-2xl font-bold">{experiment.name}</h1>
              <p className="text-muted-foreground">
                {experiment.description || "Visual Flow Designer"}
              </p>
            </div>
          </div>
          <div className="flex items-center space-x-3">
            <span className="bg-muted rounded-lg px-3 py-1 text-sm">
              {experiment.study?.name ?? "Unknown Study"}
            </span>
            <Button
              variant="ghost"
              size="sm"
              onClick={() => router.push(`/experiments/${experiment.id}`)}
            >
              <X className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </div>

      {/* Error Display */}
      {saveError && (
        <div className="border-destructive/50 bg-destructive/10 mx-6 mt-4 rounded-lg border p-4">
          <div className="flex items-start">
            <div className="flex-1">
              <h4 className="text-destructive font-medium">Save Error</h4>
              <p className="text-destructive/90 mt-1 text-sm">{saveError}</p>
            </div>
          </div>
        </div>
      )}

      {/* Flow Designer */}
      <div className="flex-1 overflow-hidden">
        <FlowDesigner
          experimentId={experiment.id}
          initialDesign={initialDesign}
          onSave={handleSave}
          isSaving={saveDesignMutation.isPending}
        />
      </div>
    </div>
  );
}
