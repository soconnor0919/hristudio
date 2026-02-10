"use client";

import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { LineChart, BarChart, Clock, Database, FileText, AlertTriangle, CheckCircle, VideoOff, Info, Bot, Activity, ArrowLeft } from "lucide-react";
import { PlaybackProvider } from "../playback/PlaybackContext";
import { PlaybackPlayer } from "../playback/PlaybackPlayer";
import { EventTimeline } from "../playback/EventTimeline";
import { api } from "~/trpc/react";
import { ScrollArea } from "~/components/ui/scroll-area";
import { cn } from "~/lib/utils";
import {
    ResizableHandle,
    ResizablePanel,
    ResizablePanelGroup,
} from "~/components/ui/resizable";
import { EventsDataTable } from "../analysis/events-data-table";

interface TrialAnalysisViewProps {
    trial: {
        id: string;
        status: string;
        startedAt: Date | null;
        completedAt: Date | null;
        duration: number | null;
        experiment: { name: string };
        participant: { participantCode: string };
        eventCount?: number;
        mediaCount?: number;
        media?: { url: string; contentType: string }[];
    };
    backHref: string;
}

export function TrialAnalysisView({ trial, backHref }: TrialAnalysisViewProps) {
    // Fetch events for timeline
    const { data: events = [] } = api.trials.getEvents.useQuery({
        trialId: trial.id,
        limit: 1000
    });

    const videoMedia = trial.media?.find(m => m.contentType.startsWith("video/"));
    const videoUrl = videoMedia?.url;

    // Metrics
    const interventionCount = events.filter(e => e.eventType.includes("intervention")).length;
    const errorCount = events.filter(e => e.eventType.includes("error")).length;
    const robotActionCount = events.filter(e => e.eventType.includes("robot_action")).length;

    return (
        <PlaybackProvider events={events} startTime={trial.startedAt ?? undefined}>
            <div className="flex h-full flex-col gap-4 p-4 text-sm">
                {/* Header Context */}
                <div className="flex items-center justify-between pb-2 border-b">
                    <div className="flex items-center gap-4">
                        <Button variant="ghost" size="icon" asChild className="-ml-2">
                            <Link href={backHref}>
                                <ArrowLeft className="h-4 w-4" />
                            </Link>
                        </Button>
                        <div className="flex flex-col">
                            <h1 className="text-lg font-semibold leading-none tracking-tight">
                                {trial.experiment.name}
                            </h1>
                            <div className="flex items-center gap-2 text-muted-foreground mt-1">
                                <span className="font-mono">{trial.participant.participantCode}</span>
                                <span>•</span>
                                <span>Session {trial.id.slice(0, 4)}</span>
                            </div>
                        </div>
                    </div>
                    <div className="flex items-center gap-4">
                        <div className="flex items-center gap-2 text-muted-foreground bg-muted/30 px-3 py-1 rounded-full border">
                            <Clock className="h-3.5 w-3.5" />
                            <span className="text-xs font-mono">
                                {trial.startedAt?.toLocaleDateString()} {trial.startedAt?.toLocaleTimeString()}
                            </span>
                        </div>
                    </div>
                </div>

                {/* Metrics Header */}
                <div className="grid grid-cols-2 gap-4 md:grid-cols-4">
                    <Card className="bg-gradient-to-br from-blue-50 to-transparent dark:from-blue-950/20">
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Duration</CardTitle>
                            <Clock className="h-4 w-4 text-blue-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">
                                {trial.duration ? (
                                    <span>{Math.floor(trial.duration / 60)}m {trial.duration % 60}s</span>
                                ) : (
                                    "--:--"
                                )}
                            </div>
                            <p className="text-xs text-muted-foreground">Total session time</p>
                        </CardContent>
                    </Card>

                    <Card className="bg-gradient-to-br from-purple-50 to-transparent dark:from-purple-950/20">
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Robot Actions</CardTitle>
                            <Bot className="h-4 w-4 text-purple-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">{robotActionCount}</div>
                            <p className="text-xs text-muted-foreground">Executed autonomous behaviors</p>
                        </CardContent>
                    </Card>

                    <Card className="bg-gradient-to-br from-orange-50 to-transparent dark:from-orange-950/20">
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Interventions</CardTitle>
                            <AlertTriangle className="h-4 w-4 text-orange-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">{interventionCount}</div>
                            <p className="text-xs text-muted-foreground">Manual wizard overrides</p>
                        </CardContent>
                    </Card>

                    <Card className="bg-gradient-to-br from-green-50 to-transparent dark:from-green-950/20">
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Completeness</CardTitle>
                            <Activity className="h-4 w-4 text-green-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">
                                {trial.status === 'completed' ? '100%' : 'Incomplete'}
                            </div>
                            <div className="flex items-center gap-2 text-xs text-muted-foreground">
                                <span className={cn(
                                    "inline-block h-2 w-2 rounded-full",
                                    trial.status === 'completed' ? "bg-green-500" : "bg-yellow-500"
                                )} />
                                {trial.status.charAt(0).toUpperCase() + trial.status.slice(1)}
                            </div>
                        </CardContent>
                    </Card>
                </div>

                {/* Main Workspace: Vertical Layout */}
                <div className="flex-1 min-h-0 rounded-xl border shadow-sm overflow-hidden bg-background">
                    <ResizablePanelGroup direction="vertical">

                        {/* TOP: Video & Timeline */}
                        <ResizablePanel defaultSize={50} minSize={30} className="flex flex-col min-h-0 bg-black/5 dark:bg-black/40">
                            <div className="relative flex-1 min-h-0 flex items-center justify-center">
                                {videoUrl ? (
                                    <div className="absolute inset-0">
                                        <PlaybackPlayer src={videoUrl} />
                                    </div>
                                ) : (
                                    <div className="flex flex-col items-center justify-center text-muted-foreground p-8 text-center">
                                        <div className="bg-muted rounded-full p-4 mb-4">
                                            <VideoOff className="h-8 w-8 opacity-50" />
                                        </div>
                                        <h3 className="font-semibold text-lg">No playback media available</h3>
                                        <p className="text-sm max-w-sm mt-2">
                                            There is no video recording associated with this trial session.
                                        </p>
                                    </div>
                                )}
                            </div>

                            {/* Timeline Control */}
                            <div className="shrink-0 border-t bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 p-4">
                                <EventTimeline />
                            </div>
                        </ResizablePanel>

                        <ResizableHandle withHandle className="bg-border/50" />

                        {/* BOTTOM: Events Table */}
                        <ResizablePanel defaultSize={50} minSize={20} className="flex flex-col min-h-0 bg-background">
                            <div className="flex items-center justify-between px-4 py-3 border-b">
                                <div className="flex items-center gap-2">
                                    <FileText className="h-4 w-4 text-primary" />
                                    <h3 className="font-semibold text-sm">Event Log</h3>
                                </div>
                                <Badge variant="secondary" className="text-xs">{events.length} Events</Badge>
                            </div>
                            <ScrollArea className="flex-1">
                                <div className="p-4">
                                    <EventsDataTable
                                        data={events.map(e => ({ ...e, timestamp: new Date(e.timestamp) }))}
                                        startTime={trial.startedAt ?? undefined}
                                    />
                                </div>
                            </ScrollArea>
                        </ResizablePanel>
                    </ResizablePanelGroup>
                </div>
            </div>
        </PlaybackProvider>
    );
}

function formatTime(ms: number) {
    if (ms < 0) return "0:00";
    const totalSeconds = Math.floor(ms / 1000);
    const m = Math.floor(totalSeconds / 60);
    const s = Math.floor(totalSeconds % 60);
    return `${m}:${s.toString().padStart(2, "0")}`;
}

