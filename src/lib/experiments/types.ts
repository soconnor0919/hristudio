export type ActionType = 
  | "move"      // Robot movement
  | "speak"     // Robot speech
  | "wait"      // Wait for a duration
  | "input"     // Wait for user input
  | "gesture"   // Robot gesture
  | "record"    // Start/stop recording
  | "condition" // Conditional branching
  | "loop";     // Repeat actions

export interface Action {
  id: string;
  type: ActionType;
  parameters: Record<string, any>;
  order: number;
}

export interface Step {
  id: string;
  title: string;
  description?: string;
  actions: Action[];
  order: number;
}

export interface Experiment {
  id: number;
  studyId: number;
  title: string;
  description?: string;
  version: number;
  status: "draft" | "active" | "archived";
  steps: Step[];
  createdAt: Date;
  updatedAt: Date;
}

// Action Parameters by Type
export interface MoveParameters {
  position: { x: number; y: number; z: number };
  speed?: number;
  easing?: "linear" | "easeIn" | "easeOut" | "easeInOut";
}

export interface SpeakParameters {
  text: string;
  voice?: string;
  speed?: number;
  pitch?: number;
  volume?: number;
}

export interface WaitParameters {
  duration: number; // in milliseconds
  showCountdown?: boolean;
}

export interface InputParameters {
  prompt?: string;
  type: "button" | "text" | "number" | "choice";
  options?: string[]; // for choice type
  timeout?: number; // optional timeout in milliseconds
}

export interface GestureParameters {
  name: string;
  speed?: number;
  intensity?: number;
}

export interface RecordParameters {
  type: "start" | "stop";
  streams: ("video" | "audio" | "sensors")[];
}

export interface ConditionParameters {
  condition: string; // JavaScript expression
  trueActions: Action[];
  falseActions?: Action[];
}

export interface LoopParameters {
  count: number;
  actions: Action[];
} 