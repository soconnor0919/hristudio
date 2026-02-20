"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import Link from "next/link";
import { Play, Zap, ArrowLeft, User, FlaskConical, LineChart } from "lucide-react";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";
import { api } from "~/trpc/react";
import { formatDistanceToNow } from "date-fns";

function TrialDetailContent() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const trialId: string =
    typeof params.trialId === "string" ? params.trialId : "";

  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();

  // Get trial data
  const {
    data: trial,
    isLoading,
    error,
  } = api.trials.get.useQuery({ id: trialId }, { enabled: !!trialId });

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Trials", href: `/studies/${studyId}/trials` },
    { label: trial?.participant.participantCode ?? "Trial" },
  ]);

  // Sync selected study (unified study-context)
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  const getStatusBadgeVariant = (status: string) => {
    switch (status) {
      case "completed":
        return "default";
      case "in_progress":
        return "secondary";
      case "scheduled":
        return "outline";
      case "failed":
      case "aborted":
        return "destructive";
      default:
        return "outline";
    }
  };

  if (isLoading) {
    return (
      <div className="flex h-96 items-center justify-center">
        <div className="text-muted-foreground">Loading trial...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Trial Details"
          description="View trial information and execution data"
          icon={Play}
          actions={
            <Button asChild variant="outline">
              <a href={`/studies/${studyId}/trials`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </a>
            </Button>
          }
        />
        <div className="flex h-96 items-center justify-center">
          <div className="text-center">
            <h3 className="text-destructive mb-2 text-lg font-semibold">
              Error Loading Trial
            </h3>
            <p className="text-muted-foreground">
              {error.message || "Failed to load trial data"}
            </p>
          </div>
        </div>
      </div>
    );
  }

  if (!trial) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Trial Details"
          description="View trial information and execution data"
          icon={Play}
          actions={
            <Button asChild variant="outline">
              <a href={`/studies/${studyId}/trials`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </a>
            </Button>
          }
        />
        <div className="flex h-96 items-center justify-center">
          <div className="text-center">
            <h3 className="mb-2 text-lg font-semibold">Trial Not Found</h3>
            <p className="text-muted-foreground">
              The requested trial could not be found.
            </p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <PageHeader
        title={`Trial: ${trial.participant.participantCode}`}
        description={`${trial.experiment.name} - Session ${trial.sessionNumber}`}
        icon={Play}
        badges={[
          {
            label: trial.status.replace("_", " ").toUpperCase(),
            variant: getStatusBadgeVariant(trial.status),
          }
        ]}
        actions={
          <div className="flex gap-2">
            {trial.status === "scheduled" && (
              <Button>
                <Play className="mr-2 h-4 w-4" />
                Start Trial
              </Button>
            )}
            {(trial.status === "in_progress" ||
              trial.status === "scheduled") && (
                <Button asChild>
                  <Link href={`/studies/${studyId}/trials/${trialId}/wizard`}>
                    <Zap className="mr-2 h-4 w-4" />
                    Wizard Interface
                  </Link>
                </Button>
              )}
            {trial.status === "completed" && (
              <Button asChild>
                <Link href={`/studies/${studyId}/trials/${trialId}/analysis`}>
                  <LineChart className="mr-2 h-4 w-4" />
                  View Analysis
                </Link>
              </Button>
            )}
            <Button asChild variant="outline">
              <Link href={`/studies/${studyId}/trials`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </Link>
            </Button>
          </div>
        }
      />

      <div className="grid grid-cols-1 gap-6 lg:grid-cols-3">
        {/* Trial Overview */}
        <Card className="lg:col-span-2">
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Play className="h-5 w-5" />
              Trial Overview
            </CardTitle>
            <CardDescription>
              Basic information about this trial execution
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="text-muted-foreground text-sm font-medium">
                  Status
                </label>
                <div className="mt-1">
                  <Badge variant={getStatusBadgeVariant(trial.status)}>
                    {trial.status.replace("_", " ")}
                  </Badge>
                </div>
              </div>
              <div>
                <label className="text-muted-foreground text-sm font-medium">
                  Session Number
                </label>
                <div className="mt-1 text-sm">{trial.sessionNumber}</div>
              </div>
              {trial.scheduledAt && (
                <div>
                  <label className="text-muted-foreground text-sm font-medium">
                    Scheduled
                  </label>
                  <div className="mt-1 text-sm">
                    {formatDistanceToNow(new Date(trial.scheduledAt), {
                      addSuffix: true,
                    })}
                  </div>
                </div>
              )}
              {trial.startedAt && (
                <div>
                  <label className="text-muted-foreground text-sm font-medium">
                    Started
                  </label>
                  <div className="mt-1 text-sm">
                    {formatDistanceToNow(new Date(trial.startedAt), {
                      addSuffix: true,
                    })}
                  </div>
                </div>
              )}
              {trial.completedAt && (
                <div>
                  <label className="text-muted-foreground text-sm font-medium">
                    Completed
                  </label>
                  <div className="mt-1 text-sm">
                    {formatDistanceToNow(new Date(trial.completedAt), {
                      addSuffix: true,
                    })}
                  </div>
                </div>
              )}
              {trial.duration && (
                <div>
                  <label className="text-muted-foreground text-sm font-medium">
                    Duration
                  </label>
                  <div className="mt-1 text-sm">
                    {Math.round(trial.duration / 1000)}s
                  </div>
                </div>
              )}
            </div>
            {trial.notes && (
              <div>
                <label className="text-muted-foreground text-sm font-medium">
                  Notes
                </label>
                <div className="text-muted-foreground mt-1 text-sm">
                  {trial.notes}
                </div>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Quick Actions */}
        <div className="space-y-6">
          {/* Experiment Info */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <FlaskConical className="h-5 w-5" />
                Experiment
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <div>
                <label className="text-muted-foreground text-sm font-medium">
                  Name
                </label>
                <div className="mt-1 text-sm">{trial.experiment.name}</div>
              </div>
              {trial.experiment.description && (
                <div>
                  <label className="text-muted-foreground text-sm font-medium">
                    Description
                  </label>
                  <div className="text-muted-foreground mt-1 text-sm">
                    {trial.experiment.description}
                  </div>
                </div>
              )}
            </CardContent>
          </Card>

          {/* Participant Info */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <User className="h-5 w-5" />
                Participant
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <div>
                <label className="text-muted-foreground text-sm font-medium">
                  Code
                </label>
                <div className="mt-1 text-sm">
                  {trial.participant.participantCode}
                </div>
              </div>
              {(() => {
                const demographics = trial.participant.demographics as Record<
                  string,
                  unknown
                > | null;
                return (
                  demographics &&
                  typeof demographics === "object" && (
                    <div>
                      <label className="text-muted-foreground text-sm font-medium">
                        Demographics
                      </label>
                      <div className="text-muted-foreground mt-1 text-sm">
                        {Object.keys(demographics).length} fields recorded
                      </div>
                    </div>
                  )
                );
              })()}
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}

export default function TrialDetailPage() {
  return (
    <Suspense
      fallback={
        <div className="flex h-96 items-center justify-center">
          <div className="text-muted-foreground">Loading...</div>
        </div>
      }
    >
      <TrialDetailContent />
    </Suspense>
  );
}
