"use client";

import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { LineChart, BarChart, Clock, Database, FileText, AlertTriangle, CheckCircle, VideoOff, Info } from "lucide-react";
import { PlaybackProvider } from "../playback/PlaybackContext";
import { PlaybackPlayer } from "../playback/PlaybackPlayer";
import { EventTimeline } from "../playback/EventTimeline";
import { api } from "~/trpc/react";
import { ScrollArea } from "~/components/ui/scroll-area";
import {
    ResizableHandle,
    ResizablePanel,
    ResizablePanelGroup,
} from "~/components/ui/resizable";

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
}

export function TrialAnalysisView({ trial }: TrialAnalysisViewProps) {
    // Fetch events for timeline
    const { data: events = [] } = api.trials.getEvents.useQuery({
        trialId: trial.id,
        limit: 1000
    });

    const videoMedia = trial.media?.find(m => m.contentType.startsWith("video/"));
    const videoUrl = videoMedia?.url;

    return (
        <PlaybackProvider events={events} startTime={trial.startedAt ?? undefined}>
            <div className="h-[calc(100vh-8rem)] flex flex-col bg-background rounded-lg border shadow-sm overflow-hidden">
                {/* Header Context */}
                <div className="flex items-center justify-between p-3 border-b bg-muted/20 flex-none h-14">
                    <div className="flex items-center gap-4">
                        <div className="flex flex-col">
                            <h1 className="text-base font-semibold leading-none">
                                {trial.experiment.name}
                            </h1>
                            <p className="text-xs text-muted-foreground mt-1">
                                {trial.participant.participantCode} • Session {trial.id.slice(0, 4)}...
                            </p>
                        </div>
                        <div className="h-8 w-px bg-border" />
                        <div className="flex items-center gap-3 text-xs text-muted-foreground">
                            <div className="flex items-center gap-1.5">
                                <Clock className="h-3.5 w-3.5" />
                                <span>{trial.startedAt?.toLocaleDateString()} {trial.startedAt?.toLocaleTimeString()}</span>
                            </div>
                            {trial.duration && (
                                <Badge variant="secondary" className="text-[10px] font-mono">
                                    {Math.floor(trial.duration / 60)}m {trial.duration % 60}s
                                </Badge>
                            )}
                        </div>
                    </div>
                </div>

                {/* Main Resizable Workspace */}
                <div className="flex-1 min-h-0">
                    <ResizablePanelGroup direction="horizontal">

                        {/* LEFT: Video & Timeline */}
                        <ResizablePanel defaultSize={65} minSize={30} className="flex flex-col min-h-0">
                            <ResizablePanelGroup direction="vertical">
                                {/* Top: Video Player */}
                                <ResizablePanel defaultSize={75} minSize={20} className="bg-black relative">
                                    {videoUrl ? (
                                        <div className="absolute inset-0">
                                            <PlaybackPlayer src={videoUrl} />
                                        </div>
                                    ) : (
                                        <div className="h-full w-full flex flex-col items-center justify-center text-slate-500">
                                            <VideoOff className="h-12 w-12 mb-3 opacity-20" />
                                            <p className="text-sm">No recording available.</p>
                                        </div>
                                    )}
                                </ResizablePanel>

                                <ResizableHandle withHandle />

                                {/* Bottom: Timeline Track */}
                                <ResizablePanel defaultSize={25} minSize={10} className="bg-background flex flex-col min-h-0">
                                    <div className="p-2 border-b flex-none bg-muted/10 flex items-center gap-2">
                                        <Info className="h-3 w-3 text-muted-foreground" />
                                        <span className="text-[10px] uppercase font-bold text-muted-foreground tracking-wider">Timeline Track</span>
                                    </div>
                                    <div className="flex-1 min-h-0 relative">
                                        <div className="absolute inset-0 p-2 overflow-hidden">
                                            <EventTimeline />
                                        </div>
                                    </div>
                                </ResizablePanel>
                            </ResizablePanelGroup>
                        </ResizablePanel>

                        <ResizableHandle withHandle />

                        {/* RIGHT: Logs & Metrics */}
                        <ResizablePanel defaultSize={35} minSize={20} className="flex flex-col min-h-0 border-l bg-muted/5">
                            {/* Metrics Strip */}
                            <div className="grid grid-cols-2 gap-2 p-3 border-b bg-background flex-none">
                                <Card className="shadow-none border-dashed bg-transparent">
                                    <CardContent className="p-3 py-2">
                                        <div className="text-[10px] uppercase text-muted-foreground font-semibold mb-0.5">Interventions</div>
                                        <div className="text-xl font-mono font-bold flex items-center gap-2">
                                            {events.filter(e => e.eventType.includes("intervention")).length}
                                            <AlertTriangle className="h-3.5 w-3.5 text-yellow-500" />
                                        </div>
                                    </CardContent>
                                </Card>
                                <Card className="shadow-none border-dashed bg-transparent">
                                    <CardContent className="p-3 py-2">
                                        <div className="text-[10px] uppercase text-muted-foreground font-semibold mb-0.5">Status</div>
                                        <div className="text-xl font-mono font-bold flex items-center gap-2">
                                            {trial.status === 'completed' ? 'PASS' : 'INC'}
                                            <div className={`h-2 w-2 rounded-full ${trial.status === 'completed' ? 'bg-green-500' : 'bg-orange-500'}`} />
                                        </div>
                                    </CardContent>
                                </Card>
                            </div>

                            {/* Log Title */}
                            <div className="p-2 px-3 border-b bg-muted/20 flex items-center justify-between flex-none">
                                <span className="text-xs font-semibold flex items-center gap-2">
                                    <FileText className="h-3.5 w-3.5 text-primary" />
                                    Event Log
                                </span>
                                <Badge variant="outline" className="text-[10px] h-5">{events.length} Events</Badge>
                            </div>

                            {/* Scrollable Event List */}
                            <div className="flex-1 min-h-0 relative bg-background/50">
                                <ScrollArea className="h-full">
                                    <div className="divide-y divide-border/50">
                                        {events.map((event, i) => (
                                            <div key={i} className="p-3 py-2 text-sm hover:bg-accent/50 transition-colors cursor-pointer group flex gap-3 items-start">
                                                <div className="font-mono text-[10px] text-muted-foreground mt-0.5 min-w-[3rem]">
                                                    {formatTime(new Date(event.timestamp).getTime() - (trial.startedAt?.getTime() ?? 0))}
                                                </div>
                                                <div className="flex-1 min-w-0 space-y-1">
                                                    <div className="flex items-center justify-between">
                                                        <span className="font-medium text-xs text-foreground group-hover:text-primary transition-colors">
                                                            {event.eventType.replace(/_/g, " ")}
                                                        </span>
                                                    </div>
                                                    {event.data && (
                                                        <div className="text-[10px] text-muted-foreground bg-muted p-1.5 rounded border font-mono whitespace-pre-wrap break-all opacity-80 group-hover:opacity-100">
                                                            {JSON.stringify(event.data as object, null, 1).replace(/"/g, '').replace(/[{}]/g, '').trim()}
                                                        </div>
                                                    )}
                                                </div>
                                            </div>
                                        ))}
                                        {events.length === 0 && (
                                            <div className="p-8 text-center text-xs text-muted-foreground italic">
                                                No events found in log.
                                            </div>
                                        )}
                                    </div>
                                </ScrollArea>
                            </div>
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
