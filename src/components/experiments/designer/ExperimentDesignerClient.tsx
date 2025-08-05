"use client";

import { ArrowLeft } from "lucide-react";
import Link from "next/link";
import { useState } from "react";
import { api } from "~/trpc/react";
import {
  ExperimentDesigner,
  type ExperimentDesign,
} from "./ExperimentDesigner";

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

  // Fetch the experiment's design data
  const { data: experimentSteps, isLoading } =
    api.experiments.getSteps.useQuery({
      experimentId: experiment.id,
    });

  const saveDesignMutation = api.experiments.saveDesign.useMutation({
    onSuccess: () => {
      setSaveError(null);
    },
    onError: (error) => {
      setSaveError(error.message);
    },
  });

  const handleSave = async (design: ExperimentDesign) => {
    try {
      await saveDesignMutation.mutateAsync({
        experimentId: experiment.id,
        steps: design.steps,
        version: design.version,
      });
    } catch (error) {
      console.error("Failed to save design:", error);
      throw error;
    }
  };

  if (isLoading) {
    return (
      <div className="flex h-screen items-center justify-center">
        <div className="text-center">
          <div className="mx-auto mb-4 h-8 w-8 animate-spin rounded-full border-b-2 border-blue-600"></div>
          <p className="text-slate-600">Loading experiment designer...</p>
        </div>
      </div>
    );
  }

  const initialDesign: ExperimentDesign = {
    id: experiment.id,
    name: experiment.name,
    description: experiment.description,
    steps:
      experimentSteps?.map((step) => ({
        ...step,
        type: step.type as "wizard" | "robot" | "parallel" | "conditional",
        description: step.description ?? undefined,
        duration: step.duration ?? undefined,
        actions: [], // Initialize with empty actions array
        parameters: step.parameters || {},
        expanded: false,
      })) || [],
    version: 1,
    lastSaved: new Date(),
  };

  return (
    <div className="flex h-screen flex-col">
      {/* Header */}
      <div className="flex items-center justify-between border-b bg-white p-4">
        <div className="flex items-center space-x-4">
          <Link
            href={`/experiments/${experiment.id}`}
            className="flex items-center text-sm text-slate-600 hover:text-slate-900"
          >
            <ArrowLeft className="mr-1 h-4 w-4" />
            Back to Experiment
          </Link>
          <div className="h-4 w-px bg-slate-300" />
          <div>
            <h1 className="text-lg font-semibold text-slate-900">
              {experiment.name}
            </h1>
            <p className="text-sm text-slate-600">Visual Protocol Designer</p>
          </div>
        </div>

        <div className="flex items-center space-x-2 text-sm text-slate-500">
          <span>Study: </span>
          <Link
            href={`/studies/${experiment.studyId}`}
            className="font-medium text-blue-600 hover:text-blue-800"
          >
            {experiment.study?.name || "Unknown Study"}
          </Link>
        </div>
      </div>

      {/* Error Display */}
      {saveError && (
        <div className="border-l-4 border-red-400 bg-red-50 p-4">
          <div className="flex">
            <div className="ml-3">
              <p className="text-sm text-red-700">
                Failed to save experiment: {saveError}
              </p>
            </div>
          </div>
        </div>
      )}

      {/* Designer */}
      <div className="flex-1 overflow-hidden">
        <ExperimentDesigner
          experimentId={experiment.id}
          initialDesign={initialDesign}
          onSave={handleSave}
        />
      </div>
    </div>
  );
}
