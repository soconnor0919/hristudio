"use client";

import { useState } from "react";
import Link from "next/link";
import { ArrowLeft } from "lucide-react";
import { api } from "~/trpc/react";
import { ExperimentDesigner, ExperimentDesign } from "./ExperimentDesigner";

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

export function ExperimentDesignerClient({ experiment }: ExperimentDesignerClientProps) {
  const [saveError, setSaveError] = useState<string | null>(null);

  // Fetch the experiment's design data
  const { data: experimentSteps, isLoading } = api.experiments.getSteps.useQuery({
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
      <div className="h-screen flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-blue-600 mx-auto mb-4"></div>
          <p className="text-slate-600">Loading experiment designer...</p>
        </div>
      </div>
    );
  }

  const initialDesign: ExperimentDesign = {
    id: experiment.id,
    name: experiment.name,
    steps: experimentSteps || [],
    version: 1,
    lastSaved: new Date(),
  };

  return (
    <div className="h-screen flex flex-col">
      {/* Header */}
      <div className="flex items-center justify-between p-4 border-b bg-white">
        <div className="flex items-center space-x-4">
          <Link
            href={`/experiments/${experiment.id}`}
            className="flex items-center text-sm text-slate-600 hover:text-slate-900"
          >
            <ArrowLeft className="h-4 w-4 mr-1" />
            Back to Experiment
          </Link>
          <div className="h-4 w-px bg-slate-300" />
          <div>
            <h1 className="text-lg font-semibold text-slate-900">
              {experiment.name}
            </h1>
            <p className="text-sm text-slate-600">
              Visual Protocol Designer
            </p>
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
        <div className="bg-red-50 border-l-4 border-red-400 p-4">
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
