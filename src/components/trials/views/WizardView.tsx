"use client";

import React from "react";
import { WizardInterface } from "../wizard/WizardInterface";

interface WizardViewProps {
  trial: {
    id: string;
    status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
    scheduledAt: Date | null;
    startedAt: Date | null;
    completedAt: Date | null;
    duration: number | null;
    sessionNumber: number | null;
    notes: string | null;
    metadata: Record<string, unknown> | null;
    experimentId: string;
    participantId: string | null;
    wizardId: string | null;
    experiment: {
      id: string;
      name: string;
      description: string | null;
      studyId: string;
    };
    participant: {
      id: string;
      participantCode: string;
      demographics: Record<string, unknown> | null;
    };
  };
  userRole: string;
}

export function WizardView({ trial, userRole }: WizardViewProps) {
  return (
    <div className="h-full max-h-full w-full overflow-hidden">
      <WizardInterface trial={trial} userRole={userRole} />
    </div>
  );
}
