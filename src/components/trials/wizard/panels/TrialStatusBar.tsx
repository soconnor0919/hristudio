"use client";

import React, { useMemo } from "react";
import {
    GitBranch,
    Sparkles,
    CheckCircle2,
    Clock,
    Play,
    StickyNote,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";
import { Progress } from "~/components/ui/progress";

export interface TrialStatusBarProps {
    currentStepIndex: number;
    totalSteps: number;
    trialStatus: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
    rosConnected: boolean;
    eventsCount: number;
    completedActionsCount: number;
    totalActionsCount: number;
    onAddNote?: () => void;
    className?: string;
}

export function TrialStatusBar({
    currentStepIndex,
    totalSteps,
    trialStatus,
    rosConnected,
    eventsCount,
    completedActionsCount,
    totalActionsCount,
    onAddNote,
    className,
}: TrialStatusBarProps) {
    const progressPercentage = useMemo(
        () => (totalSteps > 0 ? ((currentStepIndex + 1) / totalSteps) * 100 : 0),
        [currentStepIndex, totalSteps],
    );

    const actionProgress = useMemo(
        () =>
            totalActionsCount > 0
                ? (completedActionsCount / totalActionsCount) * 100
                : 0,
        [completedActionsCount, totalActionsCount],
    );

    return (
        <div
            className={cn(
                "border-border/60 bg-muted/40 supports-[backdrop-filter]:bg-muted/30 backdrop-blur",
                "flex h-9 w-full flex-shrink-0 items-center gap-4 border-t px-3 text-xs font-medium",
                className,
            )}
        >
            {/* Step Progress */}
            <div className="flex items-center gap-2">
                <span className="flex items-center gap-1.5 text-muted-foreground">
                    <GitBranch className="h-3.5 w-3.5 opacity-70" />
                    Step {currentStepIndex + 1}/{totalSteps}
                </span>
                <div className="w-20">
                    <Progress value={progressPercentage} className="h-1.5" />
                </div>
                <span className="text-muted-foreground/70">{Math.round(progressPercentage)}%</span>
            </div>

            <Separator orientation="vertical" className="h-4 opacity-50" />

            {/* Action Progress */}
            {totalActionsCount > 0 && (
                <>
                    <div className="flex items-center gap-2">
                        <span className="flex items-center gap-1.5 text-muted-foreground">
                            <Sparkles className="h-3.5 w-3.5 opacity-70" />
                            {completedActionsCount}/{totalActionsCount} actions
                        </span>
                        <div className="w-16">
                            <Progress value={actionProgress} className="h-1.5" />
                        </div>
                    </div>
                    <Separator orientation="vertical" className="h-4 opacity-50" />
                </>
            )}

            {/* Trial Stats */}
            <div className="flex items-center gap-3 text-muted-foreground">
                <span className="flex items-center gap-1.5">
                    <Clock className="h-3.5 w-3.5 opacity-70" />
                    {eventsCount} events
                </span>
                {trialStatus === "in_progress" && (
                    <Badge variant="default" className="h-5 gap-1 bg-emerald-500 px-1.5 text-[10px] font-normal">
                        <Play className="h-2.5 w-2.5" />
                        Live
                    </Badge>
                )}
                {trialStatus === "completed" && (
                    <Badge variant="secondary" className="h-5 gap-1 px-1.5 text-[10px] font-normal">
                        <CheckCircle2 className="h-2.5 w-2.5" />
                        Completed
                    </Badge>
                )}
            </div>

            <div className="flex-1" />

            {/* Quick Actions */}
            <div className="flex items-center gap-1">
                {onAddNote && (
                    <Button
                        variant="ghost"
                        size="sm"
                        className="h-7 px-2 text-xs"
                        onClick={onAddNote}
                        title="Add Quick Note"
                    >
                        <StickyNote className="mr-1.5 h-3.5 w-3.5" />
                        Note
                    </Button>
                )}
            </div>
        </div>
    );
}

export default TrialStatusBar;
