"use client";

import { 
  Move, 
  MessageSquare, 
  Clock, 
  KeyboardIcon, 
  Pointer, 
  Video, 
  GitBranch, 
  Repeat 
} from "lucide-react";
import { type ActionType } from "./types";

interface ActionConfig {
  type: ActionType;
  title: string;
  description: string;
  icon: React.ReactNode;
  defaultParameters: Record<string, any>;
}

export const AVAILABLE_ACTIONS: ActionConfig[] = [
  {
    type: "move",
    title: "Move Robot",
    description: "Move the robot to a specific position",
    icon: <Move className="h-4 w-4" />,
    defaultParameters: {
      position: { x: 0, y: 0, z: 0 },
      speed: 1,
      easing: "linear",
    },
  },
  {
    type: "speak",
    title: "Robot Speech",
    description: "Make the robot say something",
    icon: <MessageSquare className="h-4 w-4" />,
    defaultParameters: {
      text: "",
      speed: 1,
      pitch: 1,
      volume: 1,
    },
  },
  {
    type: "wait",
    title: "Wait",
    description: "Pause for a specified duration",
    icon: <Clock className="h-4 w-4" />,
    defaultParameters: {
      duration: 1000,
      showCountdown: true,
    },
  },
  {
    type: "input",
    title: "User Input",
    description: "Wait for participant response",
    icon: <KeyboardIcon className="h-4 w-4" />,
    defaultParameters: {
      type: "button",
      prompt: "Please respond",
      timeout: null,
    },
  },
  {
    type: "gesture",
    title: "Gesture",
    description: "Perform a predefined gesture",
    icon: <Pointer className="h-4 w-4" />,
    defaultParameters: {
      name: "",
      speed: 1,
      intensity: 1,
    },
  },
  {
    type: "record",
    title: "Record",
    description: "Start or stop recording",
    icon: <Video className="h-4 w-4" />,
    defaultParameters: {
      type: "start",
      streams: ["video"],
    },
  },
  {
    type: "condition",
    title: "Condition",
    description: "Branch based on a condition",
    icon: <GitBranch className="h-4 w-4" />,
    defaultParameters: {
      condition: "",
      trueActions: [],
      falseActions: [],
    },
  },
  {
    type: "loop",
    title: "Loop",
    description: "Repeat a sequence of actions",
    icon: <Repeat className="h-4 w-4" />,
    defaultParameters: {
      count: 1,
      actions: [],
    },
  },
]; 