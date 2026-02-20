import React, { useState, useCallback } from "react";
import {
    Play,
    CheckCircle,
    RotateCcw,
    Clock,
    Repeat,
    Split,
    Layers,
    ChevronRight,
    Loader2,
} from "lucide-react";
import { Button } from "~/components/ui/button";
import { cn } from "~/lib/utils";
import { Badge } from "~/components/ui/badge";

export interface ActionData {
    id: string;
    name: string;
    description: string | null;
    type: string;
    parameters: Record<string, unknown>;
    order: number;
    pluginId: string | null;
}

interface WizardActionItemProps {
    action: ActionData;
    index: number;
    isActive: boolean;
    isCompleted: boolean;
    onExecute: (actionId: string, parameters?: Record<string, unknown>) => void;
    onExecuteRobot: (
        pluginName: string,
        actionId: string,
        parameters: Record<string, unknown>,
        options?: { autoAdvance?: boolean }
    ) => Promise<void>;
    onSkip: (
        pluginName: string,
        actionId: string,
        parameters: Record<string, unknown>,
        options?: { autoAdvance?: boolean }
    ) => Promise<void>;
    onCompleted: () => void;
    readOnly?: boolean;
    isExecuting?: boolean;
    depth?: number;
    isRobotConnected?: boolean;
    onLogEvent?: (type: string, data?: any) => void;
}

export function WizardActionItem({
    action,
    index,
    isActive,
    isCompleted,
    onExecute,
    onExecuteRobot,
    onSkip,
    onCompleted,
    readOnly,
    isExecuting,
    depth = 0,
    isRobotConnected = false,
    onLogEvent,
}: WizardActionItemProps): React.JSX.Element {
    // Local state for container children completion
    const [completedChildren, setCompletedChildren] = useState<Set<number>>(new Set());
    // Local state for loop iterations
    const [currentIteration, setCurrentIteration] = useState(1);
    // Local state to track execution of this specific item
    const [isRunningLocal, setIsRunningLocal] = useState(false);
    // Local state for wait countdown
    const [countdown, setCountdown] = useState<number | null>(null);

    const isContainer =
        action.type === "hristudio-core.sequence" ||
        action.type === "hristudio-core.parallel" ||
        action.type === "hristudio-core.loop" ||
        action.type === "sequence" ||
        action.type === "parallel" ||
        action.type === "loop";

    // Branch support
    const isBranch = action.type === "hristudio-core.branch" || action.type === "branch";
    const isWait = action.type === "hristudio-core.wait" || action.type === "wait";

    // Helper to get children
    const children = (action.parameters.children as ActionData[]) || [];
    const iterations = (action.parameters.iterations as number) || 1;

    // Recursive helper to check for robot actions
    const hasRobotActions = useCallback((item: ActionData): boolean => {
        if (item.type === "robot_action" || !!item.pluginId) return true;
        if (item.parameters?.children && Array.isArray(item.parameters.children)) {
            return (item.parameters.children as ActionData[]).some(hasRobotActions);
        }
        return false;
    }, []);

    const containsRobotActions = hasRobotActions(action);

    // Countdown effect
    React.useEffect(() => {
        let interval: NodeJS.Timeout;
        if (isRunningLocal && countdown !== null && countdown > 0) {
            interval = setInterval(() => {
                setCountdown((prev) => (prev !== null && prev > 0 ? prev - 1 : 0));
            }, 1000);
        }
        return () => clearInterval(interval);
    }, [isRunningLocal, countdown]);

    // Derived state for disabled button
    const isButtonDisabled =
        isExecuting ||
        isRunningLocal ||
        (!isWait && !isRobotConnected && (action.type === 'robot_action' || !!action.pluginId || (isContainer && containsRobotActions)));


    // Handler for child completion
    const handleChildCompleted = useCallback((childIndex: number) => {
        setCompletedChildren(prev => {
            const next = new Set(prev);
            next.add(childIndex);
            return next;
        });
    }, []);

    // Handler for next loop iteration
    const handleNextIteration = useCallback(() => {
        if (currentIteration < iterations) {
            setCompletedChildren(new Set());
            setCurrentIteration(prev => prev + 1);
        } else {
            // Loop finished - allow manual completion of the loop action
        }
    }, [currentIteration, iterations]);

    // Check if current iteration is complete (all children done)
    const isIterationComplete = children.length > 0 && children.every((_, idx) => completedChildren.has(idx));
    const isLoopComplete = isIterationComplete && currentIteration >= iterations;

    return (
        <div
            className={cn(
                "relative pb-2 last:pb-0 transition-all duration-300",
                depth > 0 && "ml-4 mt-2 border-l pl-4 border-l-border/30"
            )}
        >
            {/* Visual Connection Line for Root items is handled by parent list, 
          but for nested items we handle it via border-l above */}

            <div
                className={cn(
                    "rounded-lg border transition-all duration-300",
                    isActive
                        ? "bg-card border-primary/50 shadow-md p-4"
                        : "bg-muted/5 border-transparent p-3 opacity-80 hover:opacity-100",
                    isContainer && "bg-muted/10 border-border/50"
                )}
            >
                <div className="space-y-2">
                    {/* Header Row */}
                    <div className="flex items-start justify-between gap-4">
                        <div className="flex items-center gap-2">
                            {/* Icon based on type */}
                            {isContainer && action.type.includes("loop") && <Repeat className="h-4 w-4 text-blue-500 dark:text-blue-400" />}
                            {isContainer && action.type.includes("parallel") && <Layers className="h-4 w-4 text-purple-500 dark:text-purple-400" />}
                            {isBranch && <Split className="h-4 w-4 text-orange-500 dark:text-orange-400" />}
                            {isWait && <Clock className="h-4 w-4 text-amber-500 dark:text-amber-400" />}

                            <div
                                className={cn(
                                    "text-base font-medium leading-none",
                                    isCompleted && "line-through text-muted-foreground"
                                )}
                            >
                                {action.name}
                            </div>
                        </div>

                        {/* Completion Badge */}
                        {isCompleted && <CheckCircle className="h-4 w-4 text-green-500 dark:text-green-400" />}
                    </div>

                    {action.description && (
                        <div className="text-sm text-muted-foreground">
                            {action.description}
                        </div>
                    )}

                    {/* Details for Control Flow */}
                    {isWait && (
                        <div className="flex items-center gap-2 text-xs text-amber-700 bg-amber-50/80 dark:text-amber-300 dark:bg-amber-900/30 w-fit px-2 py-1 rounded border border-amber-100 dark:border-amber-800/50">
                            <Clock className="h-3 w-3" />
                            Wait {String(action.parameters.duration || 1)}s
                        </div>
                    )}

                    {action.type.includes("loop") && (
                        <div className="flex items-center gap-2 text-xs text-blue-700 bg-blue-50/80 dark:text-blue-300 dark:bg-blue-900/30 w-fit px-2 py-1 rounded border border-blue-100 dark:border-blue-800/50">
                            <Repeat className="h-3 w-3" />
                            {String(action.parameters.iterations || 1)} Iterations
                        </div>
                    )}


                    {((!!isContainer && children.length > 0) ? (
                        <div className="mt-4 space-y-2">
                            {/* Loop Iteration Status & Controls */}
                            {action.type.includes("loop") && (
                                <div className="flex items-center justify-between bg-blue-50/50 dark:bg-blue-900/20 p-2 rounded mb-2 border border-blue-100 dark:border-blue-800/50">
                                    <div className="flex items-center gap-2">
                                        <Badge variant="outline" className="bg-white dark:bg-zinc-900 dark:text-zinc-100 border-zinc-200 dark:border-zinc-700">
                                            Iteration {currentIteration} of {iterations}
                                        </Badge>
                                        {isIterationComplete && currentIteration < iterations && (
                                            <span className="text-xs text-blue-600 dark:text-blue-400 font-medium animate-pulse">
                                                All actions complete. Ready for next iteration.
                                            </span>
                                        )}
                                        {isLoopComplete && (
                                            <span className="text-xs text-green-600 dark:text-green-400 font-medium">
                                                Loop complete!
                                            </span>
                                        )}
                                    </div>

                                    {isLoopComplete ? (
                                        <Button
                                            size="sm"
                                            onClick={(e) => {
                                                e.preventDefault();
                                                onCompleted();
                                            }}
                                            className="h-7 text-xs bg-green-600 hover:bg-green-700 text-white dark:bg-green-600 dark:hover:bg-green-500"
                                        >
                                            <CheckCircle className="mr-1 h-3 w-3" />
                                            Finish Loop
                                        </Button>
                                    ) : (
                                        isIterationComplete && currentIteration < iterations && !readOnly && (
                                            <div className="flex items-center gap-2">
                                                <Button
                                                    size="sm"
                                                    variant="secondary"
                                                    onClick={(e) => {
                                                        e.preventDefault();
                                                        onCompleted();
                                                    }}
                                                    className="h-7 text-xs"
                                                >
                                                    <ChevronRight className="mr-1 h-3 w-3" />
                                                    Exit Loop
                                                </Button>
                                                <Button
                                                    size="sm"
                                                    onClick={(e) => {
                                                        e.preventDefault();
                                                        handleNextIteration();
                                                    }}
                                                    className="h-7 text-xs"
                                                >
                                                    <Repeat className="mr-1 h-3 w-3" />
                                                    Next Iteration
                                                </Button>
                                            </div>
                                        )
                                    )}
                                </div>
                            )}

                            <div className="text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-2">
                                {action.type.includes("loop") ? "Loop Body" : "Actions"}
                            </div>

                            {children.map((child, idx) => (
                                <WizardActionItem
                                    key={`${child.id || idx}-${currentIteration}`}
                                    action={child as ActionData}
                                    index={idx}
                                    isActive={isActive && !isCompleted && !completedChildren.has(idx)}
                                    isCompleted={isCompleted || completedChildren.has(idx)}
                                    onExecute={onExecute}
                                    onExecuteRobot={onExecuteRobot}
                                    onSkip={onSkip}
                                    onCompleted={() => handleChildCompleted(idx)}
                                    readOnly={readOnly || isCompleted || completedChildren.has(idx) || (action.type.includes("parallel") && true)}
                                    isExecuting={isExecuting}
                                    depth={depth + 1}
                                    isRobotConnected={isRobotConnected}
                                    onLogEvent={onLogEvent}
                                />
                            ))}
                        </div>
                    ) : null) as any}

                    {/* Active Action Controls */}
                    {(isActive || (isCompleted && !readOnly)) && (
                        <div className="pt-3 flex flex-wrap items-center gap-3">
                            {/* Parallel Container Controls */}
                            {isContainer && action.type.includes("parallel") ? (
                                <>
                                    <Button
                                        size="sm"
                                        className={cn(
                                            "shadow-sm min-w-[100px]",
                                            isButtonDisabled && "opacity-50 cursor-not-allowed"
                                        )}
                                        onClick={async (e) => {
                                            e.preventDefault();
                                            // Run all child robot actions
                                            const children = (action.parameters.children as ActionData[]) || [];
                                            for (const child of children) {
                                                if (child.pluginId) {
                                                    // Fire and forget - don't await sequentially
                                                    onExecuteRobot(
                                                        child.pluginId,
                                                        child.type.includes(".") ? child.type.split(".").pop()! : child.type,
                                                        child.parameters || {},
                                                        { autoAdvance: false }
                                                    ).catch(console.error);
                                                }
                                            }
                                        }}
                                        disabled={isButtonDisabled}
                                        title={isButtonDisabled && !isExecuting ? "Robot disconnected" : undefined}
                                    >
                                        <Play className="mr-2 h-3.5 w-3.5" />
                                        {isCompleted ? "Rerun All" : "Run All"}
                                    </Button>
                                    {!isCompleted && (
                                        <Button
                                            size="sm"
                                            variant="outline"
                                            onClick={(e) => {
                                                e.preventDefault();
                                                onCompleted();
                                            }}
                                            disabled={isExecuting}
                                        >
                                            <CheckCircle className="mr-2 h-3.5 w-3.5" />
                                            Mark Group Complete
                                        </Button>
                                    )}
                                </>
                            ) : (
                                /* Standard Single Action Controls */
                                (action.pluginId && !["hristudio-woz"].includes(action.pluginId!) && (action.pluginId !== "hristudio-core" || isWait)) ? (
                                    <>
                                        <Button
                                            size="sm"
                                            className={cn(
                                                "shadow-sm min-w-[100px]",
                                                isButtonDisabled && "opacity-50 cursor-not-allowed"
                                            )}
                                            onClick={async (e) => {
                                                e.preventDefault();
                                                setIsRunningLocal(true);

                                                if (isWait) {
                                                    const duration = Number(action.parameters.duration || 1);
                                                    setCountdown(Math.ceil(duration));
                                                }

                                                try {
                                                    await onExecuteRobot(
                                                        action.pluginId!,
                                                        action.type.includes(".") ? action.type.split(".").pop()! : action.type,
                                                        action.parameters || {},
                                                        { autoAdvance: false }
                                                    );
                                                    if (!isCompleted) onCompleted();
                                                } catch (error) {
                                                    console.error("Action execution error:", error);
                                                } finally {
                                                    setIsRunningLocal(false);
                                                    setCountdown(null);
                                                }
                                            }}
                                            disabled={isExecuting || isRunningLocal || (!isWait && !isRobotConnected)}
                                            title={!isWait && !isRobotConnected ? "Robot disconnected" : undefined}
                                        >
                                            {isRunningLocal ? (
                                                <>
                                                    <Loader2 className="mr-2 h-3.5 w-3.5 animate-spin" />
                                                    {isWait ? (countdown !== null && countdown > 0 ? `Wait (${countdown}s)...` : "Finishing...") : "Running..."}
                                                </>
                                            ) : (
                                                <>
                                                    <Play className="mr-2 h-3.5 w-3.5" />
                                                    {isCompleted ? "Rerun" : "Run"}
                                                </>
                                            )}
                                        </Button>
                                        {!isCompleted && (
                                            <Button
                                                size="sm"
                                                variant="outline"
                                                onClick={(e) => {
                                                    e.preventDefault();
                                                    // Log manual completion
                                                    if (onLogEvent) {
                                                        onLogEvent("action_marked_complete", {
                                                            actionId: action.id,
                                                            formatted: "Action manually marked complete"
                                                        });
                                                    }
                                                    onCompleted();
                                                }}
                                                disabled={isExecuting}
                                            >
                                                <CheckCircle className="mr-2 h-3.5 w-3.5" />
                                                Mark Complete
                                            </Button>
                                        )}
                                        {!isCompleted && (
                                            <Button
                                                size="sm"
                                                variant="ghost"
                                                onClick={(e) => {
                                                    e.preventDefault();
                                                    if (onSkip) {
                                                        onSkip(action.pluginId!, action.type.includes(".") ? action.type.split(".").pop()! : action.type, action.parameters || {}, { autoAdvance: false });
                                                    }
                                                    onCompleted();
                                                }}
                                            >
                                                Skip
                                            </Button>
                                        )}
                                    </>
                                ) : (
                                    // Manual/Wizard Actions (Leaf nodes)
                                    !isContainer && action.type !== "wizard_wait_for_response" && !isCompleted && (
                                        <Button
                                            size="sm"
                                            onClick={(e) => {
                                                e.preventDefault();
                                                onCompleted();
                                            }}
                                            disabled={isExecuting}
                                        >
                                            <CheckCircle className="mr-2 h-3.5 w-3.5" />
                                            Mark Complete
                                        </Button>
                                    )
                                )
                            )}
                        </div>
                    )}

                    {/* Branching / Choice UI */}
                    {(isActive || (isCompleted && !readOnly)) &&
                        (action.type === "wizard_wait_for_response" || isBranch) &&
                        action.parameters?.options &&
                        Array.isArray(action.parameters.options) && (
                            <div className="pt-3 grid grid-cols-1 sm:grid-cols-2 gap-2">
                                {(action.parameters.options as any[]).map((opt, optIdx) => {
                                    const label = typeof opt === "string" ? opt : opt.label;
                                    const value = typeof opt === "string" ? opt : opt.value;
                                    const nextStepId = typeof opt === "object" ? opt.nextStepId : undefined;

                                    return (
                                        <Button
                                            key={optIdx}
                                            variant="outline"
                                            className="justify-start h-auto py-3 px-4 text-left hover:border-primary hover:bg-primary/5"
                                            onClick={(e) => {
                                                e.preventDefault();
                                                onExecute(action.id, { value, label, nextStepId });
                                                onCompleted();
                                            }}
                                            disabled={readOnly || isExecuting}
                                        >
                                            <div className="flex flex-col items-start gap-1">
                                                <span className="font-medium">{String(label)}</span>
                                            </div>
                                        </Button>
                                    );
                                })}
                            </div>
                        )}

                    {/* Retry for failed/completed robot actions */}
                    {isCompleted && action.pluginId && !isContainer && (
                        <div className="pt-1 flex items-center gap-1">
                            <Button
                                size="sm"
                                variant="ghost"
                                className="h-7 px-2 text-xs text-muted-foreground hover:text-primary"
                                onClick={(e) => {
                                    e.preventDefault();
                                    onExecuteRobot(
                                        action.pluginId!,
                                        action.type.includes(".") ? action.type.split(".").pop()! : action.type,
                                        action.parameters || {},
                                        { autoAdvance: false }
                                    );
                                }}
                                disabled={isExecuting}
                            >
                                <RotateCcw className="mr-1.5 h-3 w-3" />
                                Retry
                            </Button>
                        </div>
                    )}
                </div>
            </div>
        </div>
    );
}
