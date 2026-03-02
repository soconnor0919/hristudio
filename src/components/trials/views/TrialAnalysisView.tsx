import { PageHeader } from "~/components/ui/page-header";

import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import Link from "next/link";
import {
  LineChart,
  BarChart,
  Printer,
  Clock,
  Database,
  FileText,
  AlertTriangle,
  CheckCircle,
  VideoOff,
  Info,
  Bot,
  Activity,
  ArrowLeft,
} from "lucide-react";
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
    media?: {
      url: string;
      mediaType: string;
      format?: string;
      contentType?: string;
    }[];
  };
  backHref: string;
}

export function TrialAnalysisView({ trial, backHref }: TrialAnalysisViewProps) {
  // Fetch events for timeline
  const { data: events = [] } = api.trials.getEvents.useQuery(
    {
      trialId: trial.id,
      limit: 1000,
    },
    {
      refetchInterval: 5000,
    },
  );

  // Auto-print effect
  useEffect(() => {
    const searchParams = new URLSearchParams(window.location.search);
    if (searchParams.get("export") === "true") {
      // Small delay to ensure rendering
      setTimeout(() => {
        window.print();
      }, 1000);
    }
  }, []);

  const videoMedia = trial.media?.find(
    (m) =>
      m.mediaType === "video" || (m as any).contentType?.startsWith("video/"),
  );
  const videoUrl = videoMedia?.url;

  // Metrics
  const interventionCount = events.filter((e) =>
    e.eventType.includes("intervention"),
  ).length;
  const errorCount = events.filter((e) => e.eventType.includes("error")).length;
  const robotActionCount = events.filter((e) =>
    e.eventType.includes("robot_action"),
  ).length;

  return (
    <PlaybackProvider events={events} startTime={trial.startedAt ?? undefined}>
      <div
        id="trial-analysis-content"
        className="flex h-full flex-col gap-2 p-3 text-sm"
      >
        {/* Header Context */}
        <PageHeader
          title={trial.experiment.name}
          description={`Session ${trial.id.slice(0, 8)} • ${trial.startedAt?.toLocaleDateString() ?? "Unknown Date"} ${trial.startedAt?.toLocaleTimeString() ?? ""}`}
          badges={[
            {
              label: trial.status.toUpperCase(),
              variant: trial.status === "completed" ? "default" : "secondary",
              className:
                trial.status === "completed"
                  ? "bg-green-500 hover:bg-green-600"
                  : "",
            },
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
                  #trial-analysis-content,
                  #trial-analysis-content * {
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

        {/* Top Section: Metrics & Optional Video Grid */}
        <div className="flex shrink-0 flex-col gap-3 xl:flex-row">
          <Card id="tour-trial-metrics" className="flex-1 shadow-sm">
            <CardContent className="h-full p-0">
              <div className="grid h-full grid-cols-2 grid-rows-2 divide-x divide-y">
                <div className="flex flex-col justify-center p-4 md:p-6">
                  <p className="text-muted-foreground mb-2 flex items-center gap-1.5 text-sm font-medium">
                    <Clock className="h-4 w-4 text-blue-500" /> Duration
                  </p>
                  <p className="text-2xl font-bold">
                    {trial.duration ? (
                      <span>
                        {Math.floor(trial.duration / 60)}m {trial.duration % 60}
                        s
                      </span>
                    ) : (
                      "--:--"
                    )}
                  </p>
                </div>
                <div className="flex flex-col justify-center border-t-0 p-4 md:p-6">
                  <p className="text-muted-foreground mb-2 flex items-center gap-1.5 text-sm font-medium">
                    <Bot className="h-4 w-4 text-purple-500" /> Robot Actions
                  </p>
                  <p className="text-2xl font-bold">{robotActionCount}</p>
                </div>
                <div className="flex flex-col justify-center p-4 md:p-6">
                  <p className="text-muted-foreground mb-2 flex items-center gap-1.5 text-sm font-medium">
                    <AlertTriangle className="h-4 w-4 text-orange-500" />{" "}
                    Interventions
                  </p>
                  <p className="text-2xl font-bold">{interventionCount}</p>
                </div>
                <div className="flex flex-col justify-center p-4 md:p-6">
                  <p className="text-muted-foreground mb-2 flex items-center gap-1.5 text-sm font-medium">
                    <Activity className="h-4 w-4 text-green-500" /> Completeness
                  </p>
                  <div className="flex items-center gap-2 text-2xl font-bold">
                    <span
                      className={cn(
                        "inline-block h-3 w-3 rounded-full",
                        trial.status === "completed"
                          ? "bg-green-500"
                          : "bg-yellow-500",
                      )}
                    />
                    {trial.status === "completed" ? "100%" : "Incomplete"}
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          {videoUrl && (
            <Card
              id="tour-trial-video"
              className="w-full shrink-0 overflow-hidden border bg-black/5 shadow-sm xl:w-[500px] dark:bg-black/40"
            >
              <div className="relative flex aspect-video h-full w-full items-center justify-center bg-black">
                <div className="absolute inset-0">
                  <PlaybackPlayer src={videoUrl} />
                </div>
              </div>
            </Card>
          )}
        </div>

        {/* Main Workspace: Vertical Layout */}
        <div className="bg-background flex min-h-0 flex-1 flex-col overflow-hidden rounded-xl border shadow-sm">
          {/* FIXED TIMELINE: Always visible at top */}
          <div
            id="tour-trial-timeline"
            className="bg-background/95 supports-[backdrop-filter]:bg-background/60 shrink-0 border-b p-1 backdrop-blur"
          >
            <EventTimeline />
          </div>

          {/* BOTTOM: Events Table */}
          <div
            className="bg-background flex min-h-0 flex-1 flex-col"
            id="tour-trial-events"
          >
            <Tabs defaultValue="events" className="flex h-full flex-col">
              <div className="bg-muted/10 flex shrink-0 items-center justify-between border-b px-3 py-2">
                <div className="flex items-center gap-2">
                  <TabsList className="h-8">
                    <TabsTrigger value="events" className="text-xs">
                      All Events
                    </TabsTrigger>
                    <TabsTrigger value="observations" className="text-xs">
                      Observations (
                      {
                        events.filter(
                          (e) =>
                            e.eventType.startsWith("annotation") ||
                            e.eventType === "wizard_note",
                        ).length
                      }
                      )
                    </TabsTrigger>
                  </TabsList>
                </div>
                <div className="flex items-center gap-2">
                  <Input
                    placeholder="Filter..."
                    className="h-7 w-[150px] text-xs"
                    disabled
                    style={{ display: "none" }}
                  />
                  <Badge variant="outline" className="text-[10px] font-normal">
                    {events.length} Total
                  </Badge>
                </div>
              </div>

              <TabsContent value="events" className="mt-0 min-h-0 flex-1">
                <ScrollArea className="h-full">
                  <div className="p-0">
                    <EventsDataTable
                      data={events.map((e) => ({
                        ...e,
                        timestamp: new Date(e.timestamp),
                      }))}
                      startTime={trial.startedAt ?? undefined}
                    />
                  </div>
                </ScrollArea>
              </TabsContent>

              <TabsContent
                value="observations"
                className="bg-muted/5 mt-0 min-h-0 flex-1"
              >
                <ScrollArea className="h-full">
                  <div className="mx-auto max-w-2xl space-y-3 p-4">
                    {events.filter(
                      (e) =>
                        e.eventType.startsWith("annotation") ||
                        e.eventType === "wizard_note",
                    ).length > 0 ? (
                      events
                        .filter(
                          (e) =>
                            e.eventType.startsWith("annotation") ||
                            e.eventType === "wizard_note",
                        )
                        .map((e, i) => {
                          const data = e.data as any;
                          return (
                            <Card key={i} className="border shadow-none">
                              <CardHeader className="flex flex-row items-center justify-between space-y-0 p-3 pb-0">
                                <div className="flex items-center gap-2">
                                  <Badge
                                    variant="outline"
                                    className="border-yellow-200 bg-yellow-50 text-yellow-700"
                                  >
                                    {data?.category || "Note"}
                                  </Badge>
                                  <span className="text-muted-foreground font-mono text-xs">
                                    {trial.startedAt
                                      ? formatTime(
                                          new Date(e.timestamp).getTime() -
                                            new Date(trial.startedAt).getTime(),
                                        )
                                      : "--:--"}
                                  </span>
                                </div>
                                <span className="text-muted-foreground text-[10px]">
                                  {new Date(e.timestamp).toLocaleTimeString()}
                                </span>
                              </CardHeader>
                              <CardContent className="p-3 pt-2">
                                <p className="text-sm">
                                  {data?.description ||
                                    data?.note ||
                                    data?.message ||
                                    "No content"}
                                </p>
                                {data?.tags && data.tags.length > 0 && (
                                  <div className="mt-2 flex gap-1">
                                    {data.tags.map((t: string, ti: number) => (
                                      <Badge
                                        key={ti}
                                        variant="secondary"
                                        className="h-5 px-1.5 text-[10px]"
                                      >
                                        {t}
                                      </Badge>
                                    ))}
                                  </div>
                                )}
                              </CardContent>
                            </Card>
                          );
                        })
                    ) : (
                      <div className="text-muted-foreground py-12 text-center text-sm">
                        <Info className="mx-auto mb-2 h-8 w-8 opacity-20" />
                        No observations recorded for this session.
                      </div>
                    )}
                  </div>
                </ScrollArea>
              </TabsContent>
            </Tabs>
          </div>
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
