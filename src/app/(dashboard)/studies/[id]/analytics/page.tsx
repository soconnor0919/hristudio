"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect, useState } from "react";
import {
  BarChart3,
  Search,
  Filter,
  PlayCircle,
  Calendar,
  Clock,
  ChevronRight,
  User,
  LayoutGrid
} from "lucide-react";

import { PageHeader } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";
import { api } from "~/trpc/react";
import { TrialAnalysisView } from "~/components/trials/views/TrialAnalysisView";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { ScrollArea } from "~/components/ui/scroll-area";
import { formatDistanceToNow } from "date-fns";

// -- Sub-Components --

function AnalyticsContent({
  selectedTrialId,
  setSelectedTrialId,
  trialsList,
  isLoadingList
}: {
  selectedTrialId: string | null;
  setSelectedTrialId: (id: string | null) => void;
  trialsList: any[];
  isLoadingList: boolean;
}) {

  // Fetch full details of selected trial
  const {
    data: selectedTrial,
    isLoading: isLoadingTrial,
    error: trialError
  } = api.trials.get.useQuery(
    { id: selectedTrialId! },
    { enabled: !!selectedTrialId }
  );

  // Transform trial data
  const trialData = selectedTrial ? {
    ...selectedTrial,
    startedAt: selectedTrial.startedAt ? new Date(selectedTrial.startedAt) : null,
    completedAt: selectedTrial.completedAt ? new Date(selectedTrial.completedAt) : null,
    eventCount: (selectedTrial as any).eventCount,
    mediaCount: (selectedTrial as any).mediaCount,
  } : null;

  return (
    <div className="h-[calc(100vh-140px)] flex flex-col">
      {selectedTrialId ? (
        isLoadingTrial ? (
          <div className="flex-1 flex items-center justify-center bg-background/50 rounded-lg border border-dashed">
            <div className="flex flex-col items-center gap-2 animate-pulse">
              <div className="h-8 w-8 rounded-full border-2 border-primary border-t-transparent animate-spin" />
              <span className="text-muted-foreground text-sm">Loading trial data...</span>
            </div>
          </div>
        ) : trialError ? (
          <div className="flex-1 flex items-center justify-center p-8 bg-background/50 rounded-lg border border-dashed text-destructive">
            <div className="max-w-md text-center">
              <h3 className="font-semibold mb-2">Error Loading Trial</h3>
              <p className="text-sm opacity-80">{trialError.message}</p>
              <Button variant="outline" className="mt-4" onClick={() => setSelectedTrialId(null)}>
                Return to Overview
              </Button>
            </div>
          </div>
        ) : trialData ? (
          <TrialAnalysisView trial={trialData} />
        ) : null
      ) : (
        <div className="flex-1 bg-background/50 rounded-lg border shadow-sm overflow-hidden">
          <StudyOverviewPlaceholder
            trials={trialsList ?? []}
            onSelect={(id) => setSelectedTrialId(id)}
          />
        </div>
      )}
    </div>
  );
}

function StudyOverviewPlaceholder({ trials, onSelect }: { trials: any[], onSelect: (id: string) => void }) {
  const recentTrials = [...trials].sort((a, b) =>
    new Date(b.startedAt || b.createdAt).getTime() - new Date(a.startedAt || a.createdAt).getTime()
  ).slice(0, 5);

  return (
    <div className="h-full p-8 grid place-items-center bg-muted/5">
      <div className="max-w-3xl w-full grid gap-8 md:grid-cols-2">
        {/* Left: Illustration / Prompt */}
        <div className="flex flex-col justify-center space-y-4">
          <div className="bg-primary/10 w-16 h-16 rounded-2xl flex items-center justify-center mb-2">
            <BarChart3 className="h-8 w-8 text-primary" />
          </div>
          <div>
            <h2 className="text-2xl font-semibold tracking-tight">Analytics & Playback</h2>
            <CardDescription className="text-base mt-2">
              Select a session from the top right to review video recordings, event logs, and metrics.
            </CardDescription>
          </div>
          <div className="flex gap-4 pt-4">
            <div className="flex items-center gap-2 text-sm text-muted-foreground">
              <PlayCircle className="h-4 w-4" />
              Feature-rich playback
            </div>
            <div className="flex items-center gap-2 text-sm text-muted-foreground">
              <Clock className="h-4 w-4" />
              Synchronized timeline
            </div>
          </div>
        </div>

        {/* Right: Recent Sessions */}
        <Card>
          <CardHeader className="pb-3">
            <CardTitle className="text-base">Recent Sessions</CardTitle>
          </CardHeader>
          <CardContent className="p-0">
            <ScrollArea className="h-[240px]">
              <div className="px-4 pb-4 space-y-1">
                {recentTrials.map(trial => (
                  <button
                    key={trial.id}
                    onClick={() => onSelect(trial.id)}
                    className="w-full flex items-center gap-3 p-3 rounded-md hover:bg-accent transition-colors text-left group"
                  >
                    <div className="h-8 w-8 rounded-full bg-primary/10 flex items-center justify-center text-xs font-mono font-medium text-primary">
                      {trial.sessionNumber}
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="flex items-center gap-2">
                        <span className="font-medium text-sm truncate">
                          {trial.participant?.participantCode ?? "Unknown"}
                        </span>
                        <span className={`text-[10px] px-1.5 py-0.5 rounded-full border capitalize ${trial.status === 'completed' ? 'bg-green-500/10 text-green-500 border-green-500/20' :
                          trial.status === 'in_progress' ? 'bg-blue-500/10 text-blue-500 border-blue-500/20' :
                            'bg-slate-500/10 text-slate-500 border-slate-500/20'
                          }`}>
                          {trial.status.replace('_', ' ')}
                        </span>
                      </div>
                      <div className="text-xs text-muted-foreground flex items-center gap-2 mt-0.5">
                        <Calendar className="h-3 w-3" />
                        {new Date(trial.createdAt).toLocaleDateString()}
                        <span className="text-muted-foreground top-[1px] relative text-[10px]">•</span>
                        {formatDistanceToNow(new Date(trial.createdAt), { addSuffix: true })}
                      </div>
                    </div>
                    <ChevronRight className="h-4 w-4 text-muted-foreground/30 group-hover:text-primary transition-colors" />
                  </button>
                ))}
                {recentTrials.length === 0 && (
                  <div className="py-8 text-center text-sm text-muted-foreground">
                    No sessions found.
                  </div>
                )}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}

// -- Main Page --

export default function StudyAnalyticsPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // State lifted up
  const [selectedTrialId, setSelectedTrialId] = useState<string | null>(null);

  // Fetch list of trials for the selector
  const { data: trialsList, isLoading: isLoadingList } = api.trials.list.useQuery(
    { studyId, limit: 100 },
    { enabled: !!studyId }
  );

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Analytics" },
  ]);

  // Set the active study if it doesn't match the current route
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return (
    <div className="h-[calc(100vh-64px)] flex flex-col p-6 gap-6">
      <div className="flex-none">
        <PageHeader
          title="Analytics"
          description="Analyze trial data and replay sessions"
          icon={BarChart3}
          actions={
            <div className="flex items-center gap-2">
              {/* Session Selector in Header */}
              <div className="w-[300px]">
                <Select
                  value={selectedTrialId ?? "overview"}
                  onValueChange={(val) => setSelectedTrialId(val === "overview" ? null : val)}
                >
                  <SelectTrigger className="w-full h-9 text-xs">
                    <LayoutGrid className="h-3.5 w-3.5 mr-2 text-muted-foreground" />
                    <SelectValue placeholder="Select Session" />
                  </SelectTrigger>
                  <SelectContent className="max-h-[400px]" align="end">
                    <SelectItem value="overview" className="border-b mb-1 pb-1 font-medium text-xs">
                      Show Study Overview
                    </SelectItem>
                    {trialsList?.map((trial) => (
                      <SelectItem key={trial.id} value={trial.id} className="text-xs">
                        <span className="font-mono mr-2 text-muted-foreground">#{trial.sessionNumber}</span>
                        {trial.participant?.participantCode ?? "Unknown"} <span className="text-muted-foreground ml-1">({new Date(trial.createdAt).toLocaleDateString()})</span>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
            </div>
          }
        />
      </div>

      <div className="flex-1 min-h-0 bg-transparent">
        <Suspense fallback={<div>Loading analytics...</div>}>
          <AnalyticsContent
            selectedTrialId={selectedTrialId}
            setSelectedTrialId={setSelectedTrialId}
            trialsList={trialsList ?? []}
            isLoadingList={isLoadingList}
            studyId={studyId}
          />
        </Suspense>
      </div>
    </div>
  );
}
