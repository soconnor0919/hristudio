"use client";

import * as React from "react";
import {
    Table,
    TableBody,
    TableCell,
    TableHead,
    TableHeader,
    TableRow
} from "~/components/ui/table";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "~/components/ui/select";
import { usePlayback } from "../playback/PlaybackContext";
import { cn } from "~/lib/utils";
import {
    CheckCircle,
    AlertTriangle,
    Bot,
    User,
    Flag,
    MessageSquare,
    Activity,
    Video
} from "lucide-react";
import { type TrialEvent } from "./events-columns";

interface EventsDataTableProps {
    data: TrialEvent[];
    startTime?: Date;
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

export function EventsDataTable({ data, startTime }: EventsDataTableProps) {
    const { seekTo, events, currentEventIndex } = usePlayback();
    const [eventTypeFilter, setEventTypeFilter] = React.useState<string>("all");
    const [globalFilter, setGlobalFilter] = React.useState<string>("");

    // Enhanced filtering logic
    const filteredData = React.useMemo(() => {
        return data.filter(event => {
            // Type filter
            if (eventTypeFilter !== "all" && !event.eventType.includes(eventTypeFilter)) {
                return false;
            }

            // Global text search (checks type and data)
            if (globalFilter) {
                const searchLower = globalFilter.toLowerCase();
                const typeMatch = event.eventType.toLowerCase().includes(searchLower);
                // Safe JSON stringify check
                const dataString = event.data ? JSON.stringify(event.data).toLowerCase() : "";
                const dataMatch = dataString.includes(searchLower);

                return typeMatch || dataMatch;
            }

            return true;
        });
    }, [data, eventTypeFilter, globalFilter]);

    // Active Event Logic & Auto-scroll
    // Match filtered events with global playback "active event" via ID
    const activeEventId = React.useMemo(() => {
        if (currentEventIndex >= 0 && currentEventIndex < events.length) {
            // We need to match the type of ID used in data/events
            // Assuming events from context are TrialEvent compatible
            // eslint-disable-next-line @typescript-eslint/no-explicit-any
            const evt = events[currentEventIndex] as any;
            return evt?.id;
        }
        return null;
    }, [events, currentEventIndex]);

    const rowRefs = React.useRef<{ [key: string]: HTMLTableRowElement | null }>({});

    React.useEffect(() => {
        if (activeEventId && rowRefs.current[activeEventId]) {
            rowRefs.current[activeEventId]?.scrollIntoView({
                behavior: "smooth",
                block: "center",
            });
        }
    }, [activeEventId]);

    const handleRowClick = (event: TrialEvent) => {
        if (!startTime) return;
        const timeMs = new Date(event.timestamp).getTime();
        const seekSeconds = (timeMs - startTime.getTime()) / 1000;
        seekTo(Math.max(0, seekSeconds));
    };

    return (
        <div className="space-y-4">
            <div className="flex items-center justify-between">
                <div className="flex flex-1 items-center space-x-2">
                    <Input
                        placeholder="Search event data..."
                        value={globalFilter}
                        onChange={(e) => setGlobalFilter(e.target.value)}
                        className="h-8 w-[150px] lg:w-[250px]"
                    />
                    <Select value={eventTypeFilter} onValueChange={setEventTypeFilter}>
                        <SelectTrigger className="h-8 w-[160px]">
                            <SelectValue placeholder="All Events" />
                        </SelectTrigger>
                        <SelectContent>
                            <SelectItem value="all">All Events</SelectItem>
                            <SelectItem value="action_executed">Actions</SelectItem>
                            <SelectItem value="action_skipped">Skipped Actions</SelectItem>
                            <SelectItem value="intervention">Interventions</SelectItem>
                            <SelectItem value="robot">Robot Actions</SelectItem>
                            <SelectItem value="step">Step Changes</SelectItem>
                            <SelectItem value="error">Errors</SelectItem>
                            <SelectItem value="annotation">Notes</SelectItem>
                        </SelectContent>
                    </Select>
                </div>
                <div className="text-xs text-muted-foreground mr-2">
                    {filteredData.length} events
                </div>
            </div>

            <div className="rounded-md border bg-background">
                <div>
                    <Table className="w-full">
                        <TableHeader className="sticky top-0 bg-background z-10 shadow-sm">
                            <TableRow className="bg-muted/50 hover:bg-muted/50">
                                <TableHead className="w-[100px]">Time</TableHead>
                                <TableHead className="w-[180px]">Event Type</TableHead>
                                <TableHead className="w-auto">Details</TableHead>
                            </TableRow>
                        </TableHeader>
                        <TableBody>
                            {filteredData.length === 0 ? (
                                <TableRow>
                                    <TableCell colSpan={3} className="h-24 text-center">
                                        No results.
                                    </TableCell>
                                </TableRow>
                            ) : (
                                filteredData.map((event, index) => {
                                    const type = event.eventType;
                                    const data = event.data;

                                    // Type Logic
                                    const isError = type.includes("error");
                                    const isIntervention = type.includes("intervention");
                                    const isRobot = type.includes("robot");
                                    const isStep = type.includes("step");
                                    const isObservation = type.includes("annotation") || type.includes("note");
                                    const isJump = type.includes("jump");
                                    const isActionComplete = type.includes("marked_complete");
                                    const isCamera = type.includes("camera");

                                    let Icon = Activity;
                                    if (isError) Icon = AlertTriangle;
                                    else if (isIntervention || isJump) Icon = User;
                                    else if (isRobot) Icon = Bot;
                                    else if (isStep) Icon = Flag;
                                    else if (isObservation) Icon = MessageSquare;
                                    else if (isCamera) Icon = Video;
                                    else if (type.includes("completed") || isActionComplete) Icon = CheckCircle;

                                    // Details Logic
                                    let detailsContent;
                                    // eslint-disable-next-line @typescript-eslint/no-explicit-any
                                    const d = data as any; // Cast for easier access

                                    if (type.includes("jump")) {
                                        detailsContent = (
                                            <>Jumped to step <strong>{d?.stepName || (d?.toIndex !== undefined ? d.toIndex + 1 : "?")}</strong> <span className="text-muted-foreground ml-1">(Manual)</span></>
                                        );
                                    } else if (type.includes("skipped")) {
                                        detailsContent = <span className="text-orange-600 dark:text-orange-400">Skipped: {d?.actionId}</span>;
                                    } else if (type.includes("marked_complete")) {
                                        detailsContent = <span className="text-green-600 dark:text-green-400">Manually marked complete</span>;
                                    } else if (type.includes("annotation") || type.includes("note")) {
                                        detailsContent = <span className="italic text-foreground/80">{d?.description || d?.note || d?.message || "No content"}</span>;
                                    } else if (type.includes("step")) {
                                        detailsContent = <span>Step: <strong>{d?.stepName || d?.name || (d?.index !== undefined ? `Index ${d.index}` : "")}</strong></span>;
                                    } else if (type.includes("action_executed")) {
                                        const name = d?.actionName || d?.actionId;
                                        const meta = d?.actionType ? `(${d.actionType})` : d?.type ? `(${d.type})` : "";
                                        detailsContent = <span>Executed: <strong>{name}</strong> <span className="text-muted-foreground text-[10px] ml-1">{meta}</span></span>;
                                    } else if (type.includes("robot") || type.includes("say") || type.includes("speech")) {
                                        const text = d?.text || d?.message || d?.data?.text;
                                        detailsContent = (
                                            <span>
                                                Robot: <strong>{d?.command || d?.type || "Action"}</strong>
                                                {text && <span className="text-muted-foreground ml-1">"{text}"</span>}
                                            </span>
                                        );
                                    } else if (type.includes("intervention")) {
                                        detailsContent = <span className="text-orange-600 dark:text-orange-400">Intervention: {d?.type || "Manual Action"}</span>;
                                    } else if (type === "trial_started") {
                                        detailsContent = <span className="text-green-600 font-medium">Trial Started</span>;
                                    } else if (type === "trial_completed") {
                                        detailsContent = <span className="text-blue-600 font-medium">Trial Completed</span>;
                                    } else if (type === "trial_paused") {
                                        detailsContent = <span className="text-yellow-600 font-medium">Trial Paused</span>;
                                    } else if (isCamera) {
                                        detailsContent = <span className="font-medium text-teal-600 dark:text-teal-400">{type === "camera_started" ? "Recording Started" : type === "camera_stopped" ? "Recording Stopped" : "Camera Event"}</span>;
                                    } else {
                                        // Default
                                        if (d && Object.keys(d).length > 0) {
                                            detailsContent = (
                                                <code className="font-mono text-muted-foreground bg-muted/50 px-1 py-0.5 rounded border inline-block max-w-full truncate align-middle text-[10px]">
                                                    {JSON.stringify(d).replace(/[{"}]/g, " ").trim()}
                                                </code>
                                            );
                                        } else {
                                            detailsContent = <span className="text-muted-foreground text-xs">-</span>;
                                        }
                                    }

                                    const isActive = activeEventId === event.id;

                                    return (
                                        <TableRow
                                            key={event.id || index}
                                            ref={(el) => {
                                                if (event.id) rowRefs.current[event.id] = el;
                                            }}
                                            className={cn(
                                                "cursor-pointer h-auto border-l-2 border-transparent transition-colors",
                                                isActive
                                                    ? "bg-muted border-l-primary"
                                                    : "hover:bg-muted/50"
                                            )}
                                            onClick={() => handleRowClick(event)}
                                        >
                                            <TableCell className="py-1 align-top w-[100px]">
                                                <div className="flex flex-col">
                                                    <span className="font-mono font-medium text-xs">
                                                        {formatRelativeTime(event.timestamp, startTime)}
                                                    </span>
                                                    <span className="text-[10px] text-muted-foreground hidden group-hover:block">
                                                        {new Date(event.timestamp).toLocaleTimeString()}
                                                    </span>
                                                </div>
                                            </TableCell>
                                            <TableCell className="py-1 align-top w-[180px]">
                                                <div className="flex items-center">
                                                    <Badge variant="outline" className={cn(
                                                        "capitalize font-medium flex w-fit items-center gap-1.5 px-2 py-0.5 text-[10px]",
                                                        isError && "border-red-200 bg-red-50 text-red-700 dark:border-red-900/50 dark:bg-red-900/20 dark:text-red-400",
                                                        (isIntervention || isJump) && "border-orange-200 bg-orange-50 text-orange-700 dark:border-orange-900/50 dark:bg-orange-900/20 dark:text-orange-400",
                                                        isRobot && "border-purple-200 bg-purple-50 text-purple-700 dark:border-purple-900/50 dark:bg-purple-900/20 dark:text-purple-400",
                                                        isCamera && "border-teal-200 bg-teal-50 text-teal-700 dark:border-teal-900/50 dark:bg-teal-900/20 dark:text-teal-400",
                                                        isStep && "border-blue-200 bg-blue-50 text-blue-700 dark:border-blue-900/50 dark:bg-blue-900/20 dark:text-blue-400",
                                                        isObservation && "border-yellow-200 bg-yellow-50 text-yellow-700 dark:border-yellow-900/50 dark:bg-yellow-900/20 dark:text-yellow-400",
                                                        isActionComplete && "border-green-200 bg-green-50 text-green-700 dark:border-green-900/50 dark:bg-green-900/20 dark:text-green-400"
                                                    )}>
                                                        <Icon className="h-3 w-3" />
                                                        {type.replace(/_/g, " ")}
                                                    </Badge>
                                                </div>
                                            </TableCell>
                                            <TableCell className="py-1 align-top w-auto">
                                                <div className="text-xs break-words whitespace-normal leading-normal min-w-0">
                                                    {detailsContent}
                                                </div>
                                            </TableCell>
                                        </TableRow>
                                    );
                                })
                            )}
                        </TableBody>
                    </Table>
                </div>
            </div>
        </div>
    );
}
