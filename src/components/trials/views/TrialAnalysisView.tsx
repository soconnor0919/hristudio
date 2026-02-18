
import { PageHeader } from "~/components/ui/page-header";

import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { LineChart, BarChart, Printer, Clock, Database, FileText, AlertTriangle, CheckCircle, VideoOff, Info, Bot, Activity, ArrowLeft } from "lucide-react";
import { useEffect } from "react";
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
        experiment: { name: string; studyId: string };
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

    // Auto-print effect
    useEffect(() => {
        const searchParams = new URLSearchParams(window.location.search);
        if (searchParams.get('export') === 'true') {
            // Small delay to ensure rendering
            setTimeout(() => {
                window.print();
            }, 1000);
        }
    }, []);

    const videoMedia = trial.media?.find(m => m.contentType.startsWith("video/"));
    const videoUrl = videoMedia?.url;

    // Metrics
    const interventionCount = events.filter(e => e.eventType.includes("intervention")).length;
    const errorCount = events.filter(e => e.eventType.includes("error")).length;
    const robotActionCount = events.filter(e => e.eventType.includes("robot_action")).length;

    return (
        <PlaybackProvider events={events} startTime={trial.startedAt ?? undefined}>
            <div id="trial-analysis-content" className="flex h-full flex-col gap-4 p-4 text-sm">
                {/* Header Context */}
                <PageHeader
                    title={trial.experiment.name}
                    description={`Session ${trial.id.slice(0, 8)} • ${trial.startedAt?.toLocaleDateString() ?? 'Unknown Date'} ${trial.startedAt?.toLocaleTimeString() ?? ''}`}
                    badges={[
                        {
                            label: trial.status.toUpperCase(),
                            variant: trial.status === 'completed' ? 'default' : 'secondary',
                            className: trial.status === 'completed' ? 'bg-green-500 hover:bg-green-600' : ''
                        }
                    ]}
                    actions={
                        <div className="flex items-center gap-2">
                            <style jsx global>{`
                                @media print {
                                    @page {
                                        size: auto;
                                        margin: 15mm;
                                    }
                                    body {
                                        background: white;
                                        color: black;
                                        -webkit-print-color-adjust: exact;
                                        print-color-adjust: exact;
                                    }
                                    /* Hide everything by default */
                                    body * {
                                        visibility: hidden;
                                    }
                                    /* Show only our content */
                                    #trial-analysis-content, #trial-analysis-content * {
                                        visibility: visible;
                                    }
                                    #trial-analysis-content {
                                        position: absolute;
                                        left: 0;
                                        top: 0;
                                        width: 100%;
                                        height: auto;
                                        overflow: visible;
                                        padding: 0;
                                        margin: 0;
                                        background: white;
                                    }
                                    
                                    /* Hide specific non-printable elements */
                                    #tour-trial-video, 
                                    button, 
                                    .no-print, 
                                    [role="dialog"],
                                    header, 
                                    nav {
                                        display: none !important;
                                    }

                                    /* Adjust Metrics for Print */
                                    #tour-trial-metrics {
                                        display: grid;
                                        grid-template-columns: repeat(4, 1fr);
                                        gap: 1rem;
                                        margin-bottom: 2rem;
                                        page-break-inside: avoid;
                                    }
                                    #tour-trial-metrics .rounded-xl {
                                        border: 1px solid #ddd;
                                        box-shadow: none;
                                    }

                                    /* Expand Timeline */
                                    .h-28 {
                                        height: 120px !important;
                                        page-break-inside: avoid;
                                        border-bottom: 1px solid #eee;
                                        margin-bottom: 1rem;
                                    }

                                    /* Remove Panel Resizing constraints */
                                    [data-panel-group-direction="vertical"] {
                                        flex-direction: column !important;
                                        display: block !important;
                                        height: auto !important;
                                    }
                                    [data-panel] {
                                        flex: none !important;
                                        height: auto !important;
                                        overflow: visible !important;
                                    }
                                    [data-panel-resize-handle] {
                                        display: none !important;
                                    }

                                    /* Table Styles: Clean & Full Width */
                                    #tour-trial-events {
                                        display: block !important;
                                        border: none !important;
                                        height: auto !important;
                                    }
                                    #tour-trial-events [data-radix-scroll-area-viewport] {
                                        overflow: visible !important;
                                        height: auto !important;
                                    }
                                    /* Hide "Filter" input wrapper if visible */
                                    #tour-trial-events .border-b {
                                        border-bottom: 2px solid #000 !important;
                                    }
                                }
                            `}</style>
                            <Button
                                variant="outline"
                                size="sm"
                                className="gap-2"
                                onClick={() => window.print()}
                            >
                                <Printer className="h-4 w-4" />
                                Export Report
                            </Button>
                        </div>
                    }
                />

                {/* Metrics Header */}
                <div className="grid grid-cols-2 gap-4 md:grid-cols-4" id="tour-trial-metrics">
                    <Card>
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

                    <Card>
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Robot Actions</CardTitle>
                            <Bot className="h-4 w-4 text-purple-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">{robotActionCount}</div>
                            <p className="text-xs text-muted-foreground">Executed autonomous behaviors</p>
                        </CardContent>
                    </Card>

                    <Card>
                        <CardHeader className="flex flex-row items-center justify-between pb-2 space-y-0">
                            <CardTitle className="text-sm font-medium text-muted-foreground">Interventions</CardTitle>
                            <AlertTriangle className="h-4 w-4 text-orange-500" />
                        </CardHeader>
                        <CardContent>
                            <div className="text-2xl font-bold">{interventionCount}</div>
                            <p className="text-xs text-muted-foreground">Manual wizard overrides</p>
                        </CardContent>
                    </Card>

                    <Card>
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
                <div className="flex-1 min-h-0 rounded-xl border shadow-sm overflow-hidden bg-background flex flex-col">

                    {/* FIXED TIMELINE: Always visible at top */}
                    <div className="shrink-0 border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 p-1">
                        <EventTimeline />
                    </div>

                    <ResizablePanelGroup direction="vertical">

                        {/* TOP: Video (Optional) */}
                        {videoUrl && (
                            <>
                                <ResizablePanel defaultSize={40} minSize={20} className="flex flex-col min-h-0 bg-black/5 dark:bg-black/40" id="tour-trial-video">
                                    <div className="relative flex-1 min-h-0 flex items-center justify-center">
                                        <div className="absolute inset-0">
                                            <PlaybackPlayer src={videoUrl} />
                                        </div>
                                    </div>
                                </ResizablePanel>
                                <ResizableHandle withHandle className="bg-border/50" />
                            </>
                        )}

                        {/* BOTTOM: Events Table */}
                        <ResizablePanel defaultSize={videoUrl ? 60 : 100} minSize={20} className="flex flex-col min-h-0 bg-background" id="tour-trial-events">
                            <div className="flex items-center justify-between px-4 py-3 border-b shrink-0">
                                <div className="flex items-center gap-2">
                                    <FileText className="h-4 w-4 text-primary" />
                                    <h3 className="font-semibold text-sm">Event Log</h3>
                                </div>
                                <div className="flex items-center gap-2">
                                    <Input
                                        placeholder="Filter events..."
                                        className="h-8 w-[200px]"
                                        disabled
                                        style={{ display: 'none' }}
                                    />
                                    <Badge variant="secondary" className="text-xs">{events.length} Events</Badge>
                                </div>
                            </div>
                            <div className="flex-1 min-h-0">
                                <ScrollArea className="h-full">
                                    <div className="p-4">
                                        <EventsDataTable
                                            data={events.map(e => ({ ...e, timestamp: new Date(e.timestamp) }))}
                                            startTime={trial.startedAt ?? undefined}
                                        />
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

// Helper specific to this file if needed, otherwise ignore.
import { Input } from "~/components/ui/input";

function formatTime(ms: number) {
    if (ms < 0) return "0:00";
    const totalSeconds = Math.floor(ms / 1000);
    const m = Math.floor(totalSeconds / 60);
    const s = Math.floor(totalSeconds % 60);
    return `${m}:${s.toString().padStart(2, "0")}`;
}

