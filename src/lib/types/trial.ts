export interface StepData {
  id: string;
  name: string;
  description: string | null;
  type: "wizard_action" | "robot_action" | "parallel_steps" | "conditional";
  parameters: Record<string, unknown>;
  conditions?: {
    options?: {
      label: string;
      value: string;
      nextStepId?: string;
      nextStepIndex?: number;
      variant?:
        | "default"
        | "destructive"
        | "outline"
        | "secondary"
        | "ghost"
        | "link";
    }[];
  };
  order: number;
  actions?: ActionData[];
}

export interface ActionData {
  id: string;
  name: string;
  description: string | null;
  type: string;
  parameters: Record<string, unknown>;
  order: number;
  pluginId: string | null;
}

export interface TrialData {
  id: string;
  status: TrialStatus;
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  duration: number | null;
  sessionNumber: number | null;
  notes: string | null;
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
}

export type TrialStatus =
  | "scheduled"
  | "in_progress"
  | "completed"
  | "aborted"
  | "failed";

export interface TrialEvent {
  type: string;
  timestamp: Date;
  data?: unknown;
  message?: string;
}
