"use client";

import { type ReactNode } from "react";
import { type RobotPlugin } from "~/lib/plugin-store/types";
import {
    Move,
    MessageSquare,
    Clock,
    KeyboardIcon,
    Pointer,
    Video,
    GitBranch,
    Repeat,
    Navigation,
    type LucideIcon,
} from "lucide-react";

// Map of action types to their icons
const ACTION_ICONS: Record<string, LucideIcon> = {
    move: Move,
    speak: MessageSquare,
    wait: Clock,
    input: KeyboardIcon,
    gesture: Pointer,
    record: Video,
    condition: GitBranch,
    loop: Repeat,
    navigation: Navigation,
};

export interface ActionConfig {
    type: string;
    title: string;
    description: string;
    icon: ReactNode;
    defaultParameters: Record<string, any>;
    pluginId?: string;
    ros2Config?: {
        messageType: string;
        topic?: string;
        service?: string;
        action?: string;
        payloadMapping: {
            type: "direct" | "transform";
            transformFn?: string;
        };
        qos?: {
            reliability: "reliable" | "best_effort";
            durability: "volatile" | "transient_local";
            history: "keep_last" | "keep_all";
            depth?: number;
        };
    };
}

export function getActionIcon(iconName: string): ReactNode {
    const Icon = ACTION_ICONS[iconName.toLowerCase()] ?? Move;
    return <Icon className="h-4 w-4" />;
}

export function getDefaultParameters(parameters: {
    type: "object";
    properties: Record<string, {
        type: string;
        title: string;
        description?: string;
        default?: any;
        minimum?: number;
        maximum?: number;
        enum?: string[];
        unit?: string;
    }>;
    required: string[];
}): Record<string, any> {
    const defaults: Record<string, any> = {};

    for (const [key, prop] of Object.entries(parameters.properties)) {
        defaults[key] = prop.default ?? (
            prop.type === "number" ? 0 :
                prop.type === "string" ? "" :
                    prop.type === "boolean" ? false :
                        prop.type === "array" ? [] :
                            prop.type === "object" ? {} :
                                null
        );
    }

    return defaults;
}

export function getPluginActions(plugins: RobotPlugin[]): ActionConfig[] {
    return plugins.flatMap(plugin =>
        plugin.actions.map(action => ({
            type: `${plugin.robotId}:${action.type}`,
            title: action.title,
            description: action.description,
            icon: getActionIcon(action.icon ?? action.type),
            defaultParameters: getDefaultParameters(action.parameters),
            pluginId: plugin.robotId,
            ros2Config: action.ros2,
        }))
    );
}

// Built-in actions that are always available
export const BUILT_IN_ACTIONS: ActionConfig[] = [
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