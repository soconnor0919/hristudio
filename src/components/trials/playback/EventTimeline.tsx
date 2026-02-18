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

    // Generate ticks
    const ticks = useMemo(() => {
        const count = 10;
        return Array.from({ length: count + 1 }).map((_, i) => ({
            pct: (i / count) * 100,
            label: formatTime((effectiveDuration / 1000) * (i / count))
        }));
    }, [effectiveDuration]);

    const getEventIcon = (type: string) => {
        if (type.includes("intervention") || type.includes("wizard")) return <User className="h-4 w-4" />;
        if (type.includes("robot") || type.includes("action")) return <Bot className="h-4 w-4" />;
        if (type.includes("completed")) return <CheckCircle className="h-4 w-4" />;
        if (type.includes("start")) return <Flag className="h-4 w-4" />;
        if (type.includes("note")) return <MessageSquare className="h-4 w-4" />;
        if (type.includes("error")) return <AlertTriangle className="h-4 w-4" />;
        return <Activity className="h-4 w-4" />;
    };

    const getEventColor = (type: string) => {
        if (type.includes("intervention") || type.includes("wizard")) return "bg-orange-100 text-orange-600 border-orange-200";
        if (type.includes("robot") || type.includes("action")) return "bg-purple-100 text-purple-600 border-purple-200";
        if (type.includes("completed")) return "bg-green-100 text-green-600 border-green-200";
        if (type.includes("start")) return "bg-blue-100 text-blue-600 border-blue-200";
        if (type.includes("error")) return "bg-red-100 text-red-600 border-red-200";
        return "bg-slate-100 text-slate-600 border-slate-200";
    };

    return (
        <div className="w-full h-28 flex flex-col justify-center px-8 select-none">
            <TooltipProvider delayDuration={0}>
                {/* Main Interactive Area */}
                <div
                    ref={containerRef}
                    className="relative w-full h-16 flex items-center cursor-pointer group"
                    onClick={handleSeek}
                >
                    {/* The Timeline Line (Horizontal) */}
                    <div className="absolute left-0 right-0 h-0.5 top-1/2 -mt-px bg-border group-hover:bg-border/80 transition-colors" />

                    {/* Progress Fill */}
                    <div
                        className="absolute left-0 h-0.5 bg-primary/30 pointer-events-none"
                        style={{ width: `${currentProgress}%`, top: '50%', marginTop: '-1px' }}
                    />

                    {/* Playhead (Scanner) */}
                    <div
                        className="absolute h-16 w-px bg-red-500 z-30 pointer-events-none transition-all duration-75"
                        style={{ left: `${currentProgress}%`, top: '50%', transform: 'translateY(-50%)' }}
                    >
                        {/* Knob */}
                        <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-3 h-3 bg-red-500 rounded-full shadow border border-white" />
                    </div>

                    {/* Events (Avatars/Dots) */}
                    {sortedEvents.map((event, i) => {
                        const pct = getPercentage(new Date(event.timestamp).getTime());

                        return (
                            <Tooltip key={i}>
                                <TooltipTrigger asChild>
                                    <div
                                        className="absolute z-20 top-1/2 left-0 transform -translate-x-1/2 -translate-y-1/2 flex flex-col items-center group/event"
                                        style={{ left: `${pct}%` }}
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            seekTo((new Date(event.timestamp).getTime() - startTime) / 1000);
                                        }}
                                    >
                                        <div className={cn(
                                            "flex h-8 w-8 items-center justify-center rounded-full border shadow-sm transition-transform hover:scale-125 hover:z-50 bg-background relative z-20",
                                            getEventColor(event.eventType)
                                        )}>
                                            {getEventIcon(event.eventType)}
                                        </div>
                                    </div>
                                </TooltipTrigger>
                                <TooltipContent side="top">
                                    <div className="text-xs font-semibold uppercase tracking-wider mb-0.5">{event.eventType.replace(/_/g, " ")}</div>
                                    <div className="text-[10px] font-mono opacity-70 mb-1">
                                        {new Date(event.timestamp).toLocaleTimeString()}
                                    </div>
                                    {!!event.data && (
                                        <div className="bg-muted/50 p-1 rounded font-mono text-[9px] max-w-[200px] break-all">
                                            {JSON.stringify(event.data as object).slice(0, 100)}
                                        </div>
                                    )}
                                </TooltipContent>
                            </Tooltip>
                        );
                    })}

                    {/* Ticks (Below) */}
                    {ticks.map((tick, i) => (
                        <div
                            key={i}
                            className="absolute top-10 text-[10px] font-mono text-muted-foreground transform -translate-x-1/2 pointer-events-none flex flex-col items-center"
                            style={{ left: `${tick.pct}%` }}
                        >
                            {/* Tick Mark */}
                            <div className="w-px h-2 bg-border mb-1" />
                            {tick.label}
                        </div>
                    ))}
                </div>
            </TooltipProvider>
        </div>
    );
}
