"use client";

import React, { useMemo, useRef, useState } from "react";
import { usePlayback } from "./PlaybackContext";
import { cn } from "~/lib/utils";
import {
    AlertTriangle,
    CheckCircle,
    Flag,
    MessageSquare,
    Zap,
    Circle,
    Bot,
    User,
    Activity
} from "lucide-react";
import {
    Tooltip,
    TooltipContent,
    TooltipProvider,
    TooltipTrigger
} from "~/components/ui/tooltip";

function formatTime(seconds: number) {
    const min = Math.floor(seconds / 60);
    const sec = Math.floor(seconds % 60);
    return `${min}:${sec.toString().padStart(2, "0")}`;
}

export function EventTimeline() {
    const {
        duration,
        currentTime,
        events,
        seekTo,
        startTime: contextStartTime
    } = usePlayback();

    // Determine effective time range
    const sortedEvents = useMemo(() => {
        return [...events].sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());
    }, [events]);

    const startTime = useMemo(() => {
        if (contextStartTime) return new Date(contextStartTime).getTime();
        if (sortedEvents.length > 0) return new Date(sortedEvents[0]!.timestamp).getTime();
        return 0;
    }, [contextStartTime, sortedEvents]);

    const effectiveDuration = useMemo(() => {
        if (duration > 0) return duration * 1000;
        if (sortedEvents.length === 0) return 60000; // 1 min default
        const end = new Date(sortedEvents[sortedEvents.length - 1]!.timestamp).getTime();
        return Math.max(end - startTime, 1000);
    }, [duration, sortedEvents, startTime]);

    // Dimensions
    const containerRef = useRef<HTMLDivElement>(null);

    // Helpers
    const getPercentage = (timestampMs: number) => {
        const offset = timestampMs - startTime;
        return Math.max(0, Math.min(100, (offset / effectiveDuration) * 100));
    };

    const handleSeek = (e: React.MouseEvent<HTMLDivElement>) => {
        if (!containerRef.current) return;
        const rect = containerRef.current.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const pct = Math.max(0, Math.min(1, x / rect.width));
        seekTo(pct * (effectiveDuration / 1000));
    };

    const currentProgress = (currentTime * 1000 / effectiveDuration) * 100;

    // Generate ticks for "number line" look
    // We want a major tick every ~10% or meaningful time interval
    const ticks = useMemo(() => {
        const count = 10;
        return Array.from({ length: count + 1 }).map((_, i) => ({
            pct: (i / count) * 100,
            label: formatTime((effectiveDuration / 1000) * (i / count))
        }));
    }, [effectiveDuration]);

    const getEventIcon = (type: string) => {
        if (type.includes("intervention") || type.includes("wizard")) return <User className="h-3 w-3" />;
        if (type.includes("robot") || type.includes("action")) return <Bot className="h-3 w-3" />;
        if (type.includes("completed")) return <CheckCircle className="h-3 w-3" />;
        if (type.includes("start")) return <Flag className="h-3 w-3" />;
        if (type.includes("note")) return <MessageSquare className="h-3 w-3" />;
        if (type.includes("error")) return <AlertTriangle className="h-3 w-3" />;
        return <Activity className="h-3 w-3" />;
    };

    const getEventColor = (type: string) => {
        if (type.includes("intervention") || type.includes("wizard")) return "text-orange-500 border-orange-200 bg-orange-50";
        if (type.includes("robot") || type.includes("action")) return "text-purple-500 border-purple-200 bg-purple-50";
        if (type.includes("completed")) return "text-green-500 border-green-200 bg-green-50";
        if (type.includes("start")) return "text-blue-500 border-blue-200 bg-blue-50";
        if (type.includes("error")) return "text-red-500 border-red-200 bg-red-50";
        return "text-slate-500 border-slate-200 bg-slate-50";
    };

    return (
        <div className="w-full h-full flex flex-col select-none py-2">
            <TooltipProvider>
                {/* Timeline Track Container */}
                <div
                    ref={containerRef}
                    className="relative w-full flex-1 min-h-[80px] group cursor-crosshair border-b border-border/50"
                    onClick={handleSeek}
                >
                    {/* Background Grid/Ticks */}
                    <div className="absolute inset-0 pointer-events-none">
                        {/* Major Ticks */}
                        {ticks.map((tick, i) => (
                            <div
                                key={i}
                                className="absolute top-0 bottom-0 border-l border-border/30 flex flex-col justify-end"
                                style={{ left: `${tick.pct}%` }}
                            >
                                <span className="text-[10px] font-mono text-muted-foreground -ml-3 mb-1 bg-background/80 px-1 rounded">
                                    {tick.label}
                                </span>
                            </div>
                        ))}
                    </div>

                    {/* Central Axis Line */}
                    <div className="absolute top-1/2 left-0 right-0 h-px bg-border z-0" />

                    {/* Progress Fill (Subtle) */}
                    <div
                        className="absolute top-0 bottom-0 left-0 bg-primary/5 z-0 pointer-events-none"
                        style={{ width: `${currentProgress}%` }}
                    />

                    {/* Playhead */}
                    <div
                        className="absolute top-0 bottom-0 w-px bg-red-500 z-30 pointer-events-none transition-all duration-75"
                        style={{ left: `${currentProgress}%` }}
                    >
                        <div className="absolute -top-1 -ml-1.5 p-0.5 bg-red-500 rounded text-[8px] font-bold text-white w-3 h-3 flex items-center justify-center">
                            ▼
                        </div>
                    </div>

                    {/* Events "Lollipops" */}
                    {sortedEvents.map((event, i) => {
                        const pct = getPercentage(new Date(event.timestamp).getTime());
                        const isTop = i % 2 === 0; // Stagger events top/bottom

                        return (
                            <Tooltip key={i}>
                                <TooltipTrigger asChild>
                                    <div
                                        className="absolute z-20 flex flex-col items-center group/event"
                                        style={{
                                            left: `${pct}%`,
                                            top: '50%',
                                            transform: 'translate(-50%, -50%)',
                                            height: '100%'
                                        }}
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            seekTo((new Date(event.timestamp).getTime() - startTime) / 1000);
                                        }}
                                    >
                                        {/* The Stem */}
                                        <div className={cn(
                                            "w-px transition-all duration-200 bg-border group-hover/event:bg-primary group-hover/event:h-full",
                                            isTop ? "h-8 mb-auto" : "h-8 mt-auto"
                                        )} />

                                        {/* The Node */}
                                        <div className={cn(
                                            "absolute w-6 h-6 rounded-full border shadow-sm flex items-center justify-center transition-transform hover:scale-110 cursor-pointer bg-background z-10",
                                            getEventColor(event.eventType),
                                            isTop ? "-top-2" : "-bottom-2"
                                        )}>
                                            {getEventIcon(event.eventType)}
                                        </div>
                                    </div>
                                </TooltipTrigger>
                                <TooltipContent side={isTop ? "top" : "bottom"}>
                                    <div className="text-xs font-semibold uppercase tracking-wider mb-0.5">{event.eventType.replace(/_/g, " ")}</div>
                                    <div className="text-[10px] font-mono opacity-70 mb-1">
                                        {new Date(event.timestamp).toLocaleTimeString()}
                                    </div>
                                    {event.data && (
                                        <div className="bg-muted/50 p-1 rounded font-mono text-[9px] max-w-[200px] break-all">
                                            {JSON.stringify(event.data as object).slice(0, 100)}
                                        </div>
                                    )}
                                </TooltipContent>
                            </Tooltip>
                        );
                    })}
                </div>
            </TooltipProvider>
        </div>
    );
}

