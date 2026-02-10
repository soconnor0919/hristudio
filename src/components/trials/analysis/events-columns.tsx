"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { Badge } from "~/components/ui/badge";
import { cn } from "~/lib/utils";
import { CheckCircle, AlertTriangle, Info, Bot, User, Flag, MessageSquare, Activity } from "lucide-react";

// Define the shape of our data (matching schema)
export interface TrialEvent {
    id: string;
    trialId: string;
    eventType: string;
    timestamp: Date | string;
    data: any;
    createdBy: string | null;
}

// Helper to format timestamp relative to start
function formatRelativeTime(timestamp: Date | string, startTime?: Date) {
    if (!startTime) return "--:--";
    const date = new Date(timestamp);
    const diff = date.getTime() - startTime.getTime();
    if (diff < 0) return "0:00";

    const totalSeconds = Math.floor(diff / 1000);
    const m = Math.floor(totalSeconds / 60);
    const s = Math.floor(totalSeconds % 60);
    // Optional: extended formatting for longer durations
    const h = Math.floor(m / 60);

    if (h > 0) {
        return `${h}:${(m % 60).toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
    }
    return `${m}:${s.toString().padStart(2, "0")}`;
}

export const eventsColumns = (startTime?: Date): ColumnDef<TrialEvent>[] => [
    {
        id: "timestamp",
        header: "Time",
        accessorKey: "timestamp",
        cell: ({ row }) => {
            const date = new Date(row.original.timestamp);
            return (
                <div className="flex flex-col">
                    <span className="font-mono font-medium">
                        {formatRelativeTime(row.original.timestamp, startTime)}
                    </span>
                    <span className="text-[10px] text-muted-foreground hidden group-hover:block">
                        {date.toLocaleTimeString()}
                    </span>
                </div>
            );
        },
    },
    {
        accessorKey: "eventType",
        header: "Event Type",
        cell: ({ row }) => {
            const type = row.getValue("eventType") as string;
            const isError = type.includes("error");
            const isIntervention = type.includes("intervention");
            const isRobot = type.includes("robot");
            const isStep = type.includes("step");

            let Icon = Activity;
            if (isError) Icon = AlertTriangle;
            else if (isIntervention) Icon = User; // Wizard/Intervention often User
            else if (isRobot) Icon = Bot;
            else if (isStep) Icon = Flag;
            else if (type.includes("note")) Icon = MessageSquare;
            else if (type.includes("completed")) Icon = CheckCircle;

            return (
                <Badge variant="outline" className={cn(
                    "capitalize font-medium flex w-fit items-center gap-1.5 px-2 py-0.5",
                    isError && "border-red-200 bg-red-50 text-red-700 dark:border-red-900/50 dark:bg-red-900/20 dark:text-red-400",
                    isIntervention && "border-orange-200 bg-orange-50 text-orange-700 dark:border-orange-900/50 dark:bg-orange-900/20 dark:text-orange-400",
                    isRobot && "border-purple-200 bg-purple-50 text-purple-700 dark:border-purple-900/50 dark:bg-purple-900/20 dark:text-purple-400",
                    isStep && "border-blue-200 bg-blue-50 text-blue-700 dark:border-blue-900/50 dark:bg-blue-900/20 dark:text-blue-400"
                )}>
                    <Icon className="h-3 w-3" />
                    {type.replace(/_/g, " ")}
                </Badge>
            );
        },
        filterFn: (row, id, value) => {
            return value.includes(row.getValue(id));
        },
    },
    {
        accessorKey: "data",
        header: "Details",
        cell: ({ row }) => {
            const data = row.original.data;
            if (!data || Object.keys(data).length === 0) return <span className="text-muted-foreground text-xs">-</span>;

            // Simplistic view for now: JSON stringify but truncated?
            // Or meaningful extraction based on event type.
            return (
                <code className="text-[10px] font-mono text-muted-foreground bg-muted/50 px-1.5 py-0.5 rounded border block max-w-[400px] truncate">
                    {JSON.stringify(data).replace(/[{""}]/g, " ").trim()}
                </code>
            );
        },
    },
];
