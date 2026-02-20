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
        size: 90,
        meta: {
            style: { width: '90px', minWidth: '90px' }
        },
        cell: ({ row }) => {
            const date = new Date(row.original.timestamp);
            return (
                <div className="flex flex-col py-0.5">
                    <span className="font-mono font-medium text-xs">
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
        size: 160,
        meta: {
            style: { width: '160px', minWidth: '160px' }
        },
        cell: ({ row }) => {
            const type = row.getValue("eventType") as string;
            const isError = type.includes("error");
            const isIntervention = type.includes("intervention");
            const isRobot = type.includes("robot");
            const isStep = type.includes("step");

            const isObservation = type.includes("annotation") || type.includes("note");
            const isJump = type.includes("jump"); // intervention_step_jump
            const isActionComplete = type.includes("marked_complete");

            let Icon = Activity;
            if (isError) Icon = AlertTriangle;
            else if (isIntervention || isJump) Icon = User; // Jumps are interventions
            else if (isRobot) Icon = Bot;
            else if (isStep) Icon = Flag;
            else if (isObservation) Icon = MessageSquare;
            else if (type.includes("completed") || isActionComplete) Icon = CheckCircle;

            return (
                <div className="flex items-center py-0.5">
                    <Badge variant="outline" className={cn(
                        "capitalize font-medium flex w-fit items-center gap-1.5 px-2 py-0.5 text-[10px]",
                        isError && "border-red-200 bg-red-50 text-red-700 dark:border-red-900/50 dark:bg-red-900/20 dark:text-red-400",
                        (isIntervention || isJump) && "border-orange-200 bg-orange-50 text-orange-700 dark:border-orange-900/50 dark:bg-orange-900/20 dark:text-orange-400",
                        isRobot && "border-purple-200 bg-purple-50 text-purple-700 dark:border-purple-900/50 dark:bg-purple-900/20 dark:text-purple-400",
                        isStep && "border-blue-200 bg-blue-50 text-blue-700 dark:border-blue-900/50 dark:bg-blue-900/20 dark:text-blue-400",
                        isObservation && "border-yellow-200 bg-yellow-50 text-yellow-700 dark:border-yellow-900/50 dark:bg-yellow-900/20 dark:text-yellow-400",
                        isActionComplete && "border-green-200 bg-green-50 text-green-700 dark:border-green-900/50 dark:bg-green-900/20 dark:text-green-400"
                    )}>
                        <Icon className="h-3 w-3" />
                        {type.replace(/_/g, " ")}
                    </Badge>
                </div>
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
            const type = row.getValue("eventType") as string;

            // Wrapper for density and alignment
            const Wrapper = ({ children }: { children: React.ReactNode }) => (
                <div className="py-0.5 min-w-[300px] whitespace-normal break-words text-xs leading-normal">
                    {children}
                </div>
            );

            if (!data || Object.keys(data).length === 0) return <Wrapper><span className="text-muted-foreground">-</span></Wrapper>;

            // Smart Formatting
            if (type.includes("jump")) {
                return (
                    <Wrapper>
                        Jumped to step <strong>{data.stepName || (data.toIndex !== undefined ? data.toIndex + 1 : "?")}</strong>
                        <span className="text-muted-foreground ml-1">(Manual)</span>
                    </Wrapper>
                );
            }
            if (type.includes("skipped")) {
                return <Wrapper><span className="text-orange-600 dark:text-orange-400">Skipped: {data.actionId}</span></Wrapper>;
            }
            if (type.includes("marked_complete")) {
                return <Wrapper><span className="text-green-600 dark:text-green-400">Manually marked complete</span></Wrapper>;
            }
            if (type.includes("annotation") || type.includes("note")) {
                return <Wrapper><span className="italic text-foreground/80">{data.description || data.note || data.message || "No content"}</span></Wrapper>;
            }

            return (
                <Wrapper>
                    <code className="font-mono text-muted-foreground bg-muted/50 px-1.5 py-0.5 rounded border inline-block max-w-full truncate align-middle">
                        {JSON.stringify(data).replace(/[{""}]/g, " ").trim()}
                    </code>
                </Wrapper>
            );
        },
    },
];
