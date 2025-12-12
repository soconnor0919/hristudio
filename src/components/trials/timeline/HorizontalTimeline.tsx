"use client";

import React, { useState } from "react";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent } from "~/components/ui/card";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Flag, CheckCircle, Bot, User, MessageSquare, AlertTriangle, Activity } from "lucide-react";

interface TimelineEvent {
    type: string;
    timestamp: Date;
    message?: string;
    data?: unknown;
}

interface HorizontalTimelineProps {
    events: TimelineEvent[];
    startTime?: Date;
    endTime?: Date;
}

export function HorizontalTimeline({ events, startTime, endTime }: HorizontalTimelineProps) {
    const [selectedEvent, setSelectedEvent] = useState<TimelineEvent | null>(null);

    if (events.length === 0) {
        return (
            <div className="text-center text-sm text-muted-foreground py-8">
                No events recorded yet
            </div>
        );
    }

    // Calculate time range
    const timestamps = events.map(e => e.timestamp.getTime());
    const minTime = startTime?.getTime() ?? Math.min(...timestamps);
    const maxTime = endTime?.getTime() ?? Math.max(...timestamps);
    const duration = maxTime - minTime;

    // Generate time markers (every 10 seconds or appropriate interval)
    const getTimeMarkers = () => {
        const markers: Date[] = [];
        const interval = duration > 300000 ? 60000 : duration > 60000 ? 30000 : 10000; // 1min, 30s, or 10s intervals

        for (let time = minTime; time <= maxTime; time += interval) {
            markers.push(new Date(time));
        }
        if (markers[markers.length - 1]?.getTime() !== maxTime) {
            markers.push(new Date(maxTime));
        }
        return markers;
    };

    const timeMarkers = getTimeMarkers();

    // Get position percentage for a timestamp
    const getPosition = (timestamp: Date) => {
        if (duration === 0) return 50;
        return ((timestamp.getTime() - minTime) / duration) * 100;
    };

    // Get color and icon for event type
    const getEventStyle = (eventType: string) => {
        if (eventType.includes("start") || eventType === "trial_started") {
            return { color: "bg-blue-500", Icon: Flag };
        } else if (eventType.includes("complete") || eventType === "trial_completed") {
            return { color: "bg-green-500", Icon: CheckCircle };
        } else if (eventType.includes("robot") || eventType.includes("action")) {
            return { color: "bg-purple-500", Icon: Bot };
        } else if (eventType.includes("wizard") || eventType.includes("intervention")) {
            return { color: "bg-orange-500", Icon: User };
        } else if (eventType.includes("note") || eventType.includes("annotation")) {
            return { color: "bg-yellow-500", Icon: MessageSquare };
        } else if (eventType.includes("error") || eventType.includes("issue")) {
            return { color: "bg-red-500", Icon: AlertTriangle };
        }
        return { color: "bg-gray-500", Icon: Activity };
    };

    return (
        <div className="space-y-4">
            {/* Timeline visualization */}
            <div className="relative">
                <ScrollArea className="w-full">
                    <div className="min-w-[800px] px-4 py-8">
                        {/* Time markers */}
                        <div className="relative h-20 mb-8">
                            {/* Main horizontal line */}
                            <div className="absolute top-1/2 left-0 right-0 h-0.5 bg-border" style={{ transform: 'translateY(-50%)' }} />

                            {/* Time labels */}
                            {timeMarkers.map((marker, i) => {
                                const pos = getPosition(marker);
                                return (
                                    <div
                                        key={i}
                                        className="absolute"
                                        style={{ left: `${pos}%`, top: '50%', transform: 'translate(-50%, -50%)' }}
                                    >
                                        <div className="flex flex-col items-center">
                                            <div className="text-[10px] font-mono text-muted-foreground mb-2">
                                                {marker.toLocaleTimeString([], {
                                                    hour12: false,
                                                    hour: '2-digit',
                                                    minute: '2-digit',
                                                    second: '2-digit'
                                                })}
                                            </div>
                                            <div className="w-px h-4 bg-border" />
                                        </div>
                                    </div>
                                );
                            })}
                        </div>

                        {/* Event markers */}
                        <div className="relative h-40">
                            {/* Timeline line for events */}
                            <div className="absolute top-20 left-0 right-0 h-0.5 bg-border" />

                            {events.map((event, i) => {
                                const pos = getPosition(event.timestamp);
                                const { color, Icon } = getEventStyle(event.type);
                                const isSelected = selectedEvent === event;

                                return (
                                    <div
                                        key={i}
                                        className="absolute"
                                        style={{
                                            left: `${pos}%`,
                                            top: '50%',
                                            transform: 'translate(-50%, -50%)'
                                        }}
                                    >
                                        {/* Clickable marker group */}
                                        <button
                                            onClick={() => setSelectedEvent(isSelected ? null : event)}
                                            className="flex flex-col items-center gap-1 cursor-pointer group"
                                            title={event.message || event.type}
                                        >
                                            {/* Vertical dash */}
                                            <div className={`
                                                w-1 h-20 ${color} rounded-full
                                                group-hover:w-1.5 transition-all
                                                ${isSelected ? 'w-1.5 ring-2 ring-offset-2 ring-offset-background ring-primary' : ''}
                                            `} />

                                            {/* Icon indicator */}
                                            <div className={`
                                                p-1.5 rounded-full ${color} bg-opacity-20
                                                group-hover:bg-opacity-30 transition-all
                                                ${isSelected ? 'ring-2 ring-primary bg-opacity-40' : ''}
                                            `}>
                                                <Icon className={`h-3.5 w-3.5 ${color.replace('bg-', 'text-')}`} />
                                            </div>
                                        </button>
                                    </div>
                                );
                            })}
                        </div>
                    </div>
                </ScrollArea>
            </div>

            {/* Selected event details */}
            {selectedEvent && (
                <Card>
                    <CardContent className="pt-4">
                        <div className="space-y-2">
                            <div className="flex items-center gap-2">
                                <Badge variant="outline" className="text-xs">
                                    {selectedEvent.type.replace(/_/g, " ")}
                                </Badge>
                                <span className="text-xs font-mono text-muted-foreground">
                                    {selectedEvent.timestamp.toLocaleTimeString([], {
                                        hour12: false,
                                        hour: '2-digit',
                                        minute: '2-digit',
                                        second: '2-digit',
                                        fractionalSecondDigits: 3
                                    })}
                                </span>
                            </div>
                            {selectedEvent.message && (
                                <p className="text-sm">{selectedEvent.message}</p>
                            )}
                            {selectedEvent.data !== undefined && selectedEvent.data !== null && (
                                <details className="text-xs">
                                    <summary className="cursor-pointer text-muted-foreground hover:text-foreground">
                                        Event data
                                    </summary>
                                    <pre className="mt-2 p-2 bg-muted rounded text-[10px] overflow-auto">
                                        {JSON.stringify(selectedEvent.data, null, 2)}
                                    </pre>
                                </details>
                            )}
                        </div>
                    </CardContent>
                </Card>
            )}
        </div>
    );
}
