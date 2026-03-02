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
  Activity,
} from "lucide-react";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
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
    startTime: contextStartTime,
  } = usePlayback();

  // Determine effective time range
  const sortedEvents = useMemo(() => {
    return [...events].sort(
      (a, b) =>
        new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime(),
    );
  }, [events]);

  const startTime = useMemo(() => {
    if (contextStartTime) return new Date(contextStartTime).getTime();
    return 0;
  }, [contextStartTime]);

  const effectiveDuration = useMemo(() => {
    if (duration > 0) return duration * 1000;
    return 60000; // 1 min default
  }, [duration]);

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

  const currentProgress = ((currentTime * 1000) / effectiveDuration) * 100;

  // Generate ticks
  const ticks = useMemo(() => {
    const count = 10;
    return Array.from({ length: count + 1 }).map((_, i) => ({
      pct: (i / count) * 100,
      label: formatTime((effectiveDuration / 1000) * (i / count)),
    }));
  }, [effectiveDuration]);

  const getEventIcon = (type: string) => {
    if (
      type.includes("intervention") ||
      type.includes("wizard") ||
      type.includes("jump")
    )
      return <User className="h-4 w-4" />;
    if (type.includes("robot") || type.includes("action"))
      return <Bot className="h-4 w-4" />;
    if (type.includes("completed")) return <CheckCircle className="h-4 w-4" />;
    if (type.includes("start")) return <Flag className="h-4 w-4" />;
    if (type.includes("note") || type.includes("annotation"))
      return <MessageSquare className="h-4 w-4" />;
    if (type.includes("error")) return <AlertTriangle className="h-4 w-4" />;
    return <Activity className="h-4 w-4" />;
  };

  const getEventColor = (type: string) => {
    if (
      type.includes("intervention") ||
      type.includes("wizard") ||
      type.includes("jump")
    )
      return "bg-orange-100 text-orange-600 border-orange-200";
    if (type.includes("robot") || type.includes("action"))
      return "bg-purple-100 text-purple-600 border-purple-200";
    if (type.includes("completed"))
      return "bg-green-100 text-green-600 border-green-200";
    if (type.includes("start"))
      return "bg-blue-100 text-blue-600 border-blue-200";
    if (type.includes("note") || type.includes("annotation"))
      return "bg-yellow-100 text-yellow-600 border-yellow-200";
    if (type.includes("error")) return "bg-red-100 text-red-600 border-red-200";
    return "bg-slate-100 text-slate-600 border-slate-200";
  };

  return (
    <div className="flex h-28 w-full flex-col justify-center px-8 select-none">
      <TooltipProvider delayDuration={0}>
        {/* Main Interactive Area */}
        <div
          ref={containerRef}
          className="group relative flex h-16 w-full cursor-pointer items-center"
          onClick={handleSeek}
        >
          {/* The Timeline Line (Horizontal) */}
          <div className="bg-border group-hover:bg-border/80 absolute top-1/2 right-0 left-0 -mt-px h-0.5 transition-colors" />

          {/* Progress Fill */}
          <div
            className="bg-primary/30 pointer-events-none absolute left-0 h-0.5"
            style={{
              width: `${currentProgress}%`,
              top: "50%",
              marginTop: "-1px",
            }}
          />

          {/* Playhead (Scanner) */}
          <div
            className="pointer-events-none absolute z-30 h-16 w-px bg-red-500 transition-all duration-75"
            style={{
              left: `${currentProgress}%`,
              top: "50%",
              transform: "translateY(-50%)",
            }}
          >
            {/* Knob */}
            <div className="absolute top-1/2 left-1/2 h-3 w-3 -translate-x-1/2 -translate-y-1/2 rounded-full border border-white bg-red-500 shadow" />
          </div>

          {/* Events (Avatars/Dots) */}
          {sortedEvents.map((event, i) => {
            const pct = getPercentage(new Date(event.timestamp).getTime());

            // Smart Formatting Logic
            const details = (() => {
              const { eventType, data } = event;
              // eslint-disable-next-line @typescript-eslint/no-explicit-any
              const d = data as any;

              if (eventType.includes("jump"))
                return `Jumped to step ${d?.stepName || d?.toIndex + 1 || "?"} (Manual)`;
              if (eventType.includes("skipped"))
                return `Skipped: ${d?.actionId}`;
              if (eventType.includes("marked_complete"))
                return "Manually marked complete";
              if (
                eventType.includes("annotation") ||
                eventType.includes("note")
              )
                return d?.description || d?.note || d?.message || "Note";

              if (!d || Object.keys(d).length === 0) return null;
              return JSON.stringify(d)
                .slice(0, 100)
                .replace(/[{""}]/g, " ")
                .trim();
            })();

            return (
              <Tooltip key={i}>
                <TooltipTrigger asChild>
                  <div
                    className="group/event absolute top-1/2 left-0 z-20 flex -translate-x-1/2 -translate-y-1/2 transform cursor-pointer flex-col items-center p-2"
                    style={{ left: `${pct}%` }}
                    onClick={(e) => {
                      e.stopPropagation();
                      // startTime is in ms, timestamp is Date string or obj
                      const timeMs = new Date(event.timestamp).getTime();
                      const seekSeconds = (timeMs - startTime) / 1000;
                      seekTo(Math.max(0, seekSeconds));
                    }}
                  >
                    <div
                      className={cn(
                        "bg-background relative z-20 flex h-7 w-7 items-center justify-center rounded-full border shadow-sm transition-transform hover:z-50 hover:scale-125",
                        getEventColor(event.eventType),
                      )}
                    >
                      {getEventIcon(event.eventType)}
                    </div>
                  </div>
                </TooltipTrigger>
                <TooltipContent side="top">
                  <div className="mb-0.5 text-xs font-semibold tracking-wider uppercase">
                    {event.eventType.replace(/_/g, " ")}
                  </div>
                  <div className="mb-1 font-mono text-[10px] opacity-70">
                    {new Date(event.timestamp).toLocaleTimeString()}
                  </div>
                  {!!details && (
                    <div className="bg-muted/50 max-w-[220px] rounded border p-1.5 text-[10px] break-words whitespace-normal">
                      {details}
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
              className="text-muted-foreground pointer-events-none absolute top-10 flex -translate-x-1/2 transform flex-col items-center font-mono text-[10px]"
              style={{ left: `${tick.pct}%` }}
            >
              {/* Tick Mark */}
              <div className="bg-border mb-1 h-2 w-px" />
              {tick.label}
            </div>
          ))}
        </div>
      </TooltipProvider>
    </div>
  );
}
