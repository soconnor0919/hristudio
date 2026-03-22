"use client";

import * as React from "react";
import Link from "next/link";
import { useSession } from "~/lib/auth-client";
import { formatDistanceToNow } from "date-fns";
import {
  Play,
  Plus,
  Activity,
  Clock,
  CheckCircle2,
  Users,
  FlaskConical,
  ChevronRight,
  Bot,
  Radio,
  Building,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { ScrollArea } from "~/components/ui/scroll-area";
import { PageHeader } from "~/components/ui/page-layout";
import { api } from "~/trpc/react";

export default function DashboardPage() {
  const { data: session } = useSession();
  const userName = session?.user?.name ?? "Researcher";

  const { data: userStudies } = api.studies.list.useQuery({
    memberOnly: true,
    limit: 10,
  });

  const { data: recentTrials } = api.trials.list.useQuery({
    limit: 5,
  });

  const { data: liveTrials } = api.dashboard.getLiveTrials.useQuery(
    {},
    { refetchInterval: 5000 },
  );

  const { data: stats } = api.dashboard.getStats.useQuery({});

  const greeting = (() => {
    const hour = new Date().getHours();
    if (hour < 12) return "Good morning";
    if (hour < 18) return "Good afternoon";
    return "Good evening";
  })();

  const firstStudy = userStudies?.studies?.[0];

  return (
    <div className="space-y-6">
      {/* Header */}
      <PageHeader
        title={`${greeting}, ${userName.split(" ")[0]}`}
        description="Ready to run your next session?"
        icon={Bot}
        actions={
          <div className="flex gap-2">
            <Button variant="outline" asChild>
              <Link href="/studies/new">
                <Plus className="mr-2 h-4 w-4" />
                New Study
              </Link>
            </Button>
            <Button asChild className="glow-teal">
              <Link href={firstStudy?.id ? `/studies/${firstStudy.id}/trials/new` : "/studies/new"}>
                <Play className="mr-2 h-4 w-4" />
                Start Trial
              </Link>
            </Button>
          </div>
        }
      />

      {/* Live Trials Banner */}
      {liveTrials && liveTrials.length > 0 && (
        <div className="bg-destructive/10 border-border flex items-center gap-4 rounded-lg border p-4">
          <div className="relative">
            <Radio className="h-8 w-8 text-destructive animate-pulse" />
          </div>
          <div className="flex-1">
            <p className="font-semibold">
              {liveTrials.length} Active Session{liveTrials.length > 1 ? "s" : ""}
            </p>
            <p className="text-muted-foreground text-sm">
              {liveTrials.map((t) => t.participantCode).join(", ")}
            </p>
          </div>
          <Button size="sm" variant="secondary" asChild>
            <Link href={`/studies/${liveTrials?.[0]?.studyId}/trials/${liveTrials?.[0]?.id}/wizard`}>
              View <ChevronRight className="ml-1 h-4 w-4" />
            </Link>
          </Button>
        </div>
      )}

      {/* Stats Row */}
      <div className="grid gap-4 md:grid-cols-4">
        <StatCard
          label="Active Trials"
          value={stats?.activeTrials ?? 0}
          icon={Activity}
          color="teal"
        />
        <StatCard
          label="Completed Today"
          value={stats?.completedToday ?? 0}
          icon={CheckCircle2}
          color="emerald"
        />
        <StatCard
          label="Total Studies"
          value={userStudies?.studies?.length ?? 0}
          icon={FlaskConical}
          color="blue"
        />
        <StatCard
          label="Scheduled"
          value={stats?.scheduledTrials ?? 0}
          icon={Clock}
          color="violet"
        />
      </div>

      {/* Main Grid */}
      <div className="grid gap-6 lg:grid-cols-3">
        {/* Studies List */}
        <div className="bg-card rounded-lg border lg:col-span-2">
          <div className="flex items-center justify-between border-b px-6 py-4">
            <div className="flex items-center gap-3">
              <FlaskConical className="text-primary h-5 w-5" />
              <h2 className="font-semibold">Your Studies</h2>
            </div>
            <Button variant="ghost" size="sm" asChild>
              <Link href="/studies">
                View all <ChevronRight className="ml-1 h-4 w-4" />
              </Link>
            </Button>
          </div>
          <div className="p-6">
            <div className="space-y-3">
              {userStudies?.studies?.slice(0, 5).map((study) => (
                <Link
                  key={study.id}
                  href={`/studies/${study.id}`}
                  className="hover:bg-accent/50 group flex items-center justify-between rounded-lg border p-4 transition-colors"
                >
                  <div className="flex items-center gap-4">
                    <div className="flex h-12 w-12 items-center justify-center rounded-full bg-primary/10">
                      <Bot className="h-6 w-6 text-primary" />
                    </div>
                    <div>
                      <p className="font-semibold group-hover:text-primary">
                        {study.name}
                      </p>
                      <p className="text-muted-foreground text-sm">
                        {study.status === "active" ? (
                          <span className="text-emerald-600 dark:text-emerald-400">Active</span>
                        ) : (
                          <span>{study.status}</span>
                        )}
                      </p>
                    </div>
                  </div>
                  <ChevronRight className="h-5 w-5 text-muted-foreground group-hover:text-primary" />
                </Link>
              ))}

              {!userStudies?.studies?.length && (
                <div className="flex flex-col items-center justify-center py-12 text-center">
                  <Bot className="text-muted-foreground/50 mb-4 h-16 w-16" />
                  <p className="font-medium">No studies yet</p>
                  <p className="text-muted-foreground mb-4 text-sm">
                    Create your first study to get started
                  </p>
                  <Button asChild>
                    <Link href="/studies/new">
                      <Plus className="mr-2 h-4 w-4" />
                      Create Study
                    </Link>
                  </Button>
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Quick Links & Recent */}
        <div className="space-y-6">
          {/* Quick Actions */}
          <div className="bg-card rounded-lg border">
            <div className="border-b px-6 py-4">
              <h2 className="font-semibold">Quick Actions</h2>
            </div>
            <div className="space-y-2 p-4">
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href="/studies/new">
                  <Plus className="mr-3 h-4 w-4" />
                  Create New Study
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={firstStudy?.id ? `/studies/${firstStudy.id}/experiments/new` : "/studies"}>
                  <FlaskConical className="mr-3 h-4 w-4" />
                  Design Experiment
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={firstStudy?.id ? `/studies/${firstStudy.id}/participants/new` : "/studies"}>
                  <Users className="mr-3 h-4 w-4" />
                  Add Participant
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={firstStudy?.id ? `/studies/${firstStudy.id}/trials/new` : "/studies"}>
                  <Play className="mr-3 h-4 w-4" />
                  Start Trial
                </Link>
              </Button>
            </div>
          </div>

          {/* Recent Trials */}
          <div className="bg-card rounded-lg border">
            <div className="flex items-center justify-between border-b px-6 py-4">
              <div className="flex items-center gap-2">
                <Clock className="h-4 w-4 text-muted-foreground" />
                <h2 className="font-semibold">Recent Trials</h2>
              </div>
            </div>
            <div className="p-4">
              <ScrollArea className="h-[200px]">
                <div className="space-y-3">
                  {recentTrials?.slice(0, 5).map((trial) => (
                    <Link
                      key={trial.id}
                      href={`/studies/${trial.experiment.studyId}/trials/${trial.id}`}
                      className="hover:bg-accent/50 block rounded-md border p-3 transition-colors"
                    >
                      <div className="flex items-center justify-between">
                        <span className="font-medium">{trial.participant.participantCode}</span>
                        <Badge
                          variant={
                            trial.status === "completed"
                              ? "default"
                              : trial.status === "in_progress"
                                ? "secondary"
                                : "outline"
                          }
                          className="text-xs"
                        >
                          {trial.status.replace("_", " ")}
                        </Badge>
                      </div>
                      <p className="text-muted-foreground mt-1 text-xs">
                        {trial.experiment.name}
                      </p>
                      {trial.completedAt && (
                        <p className="text-muted-foreground mt-1 text-xs">
                          {formatDistanceToNow(new Date(trial.completedAt), { addSuffix: true })}
                        </p>
                      )}
                    </Link>
                  ))}

                  {!recentTrials?.length && (
                    <p className="text-muted-foreground py-4 text-center text-sm">
                      No trials yet
                    </p>
                  )}
                </div>
              </ScrollArea>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

function StatCard({
  label,
  value,
  icon: Icon,
  color,
}: {
  label: string;
  value: number;
  icon: React.ElementType;
  color: "teal" | "emerald" | "blue" | "violet";
}) {
  const colorClasses = {
    teal: "bg-primary/10 text-primary",
    emerald: "bg-emerald-500/10 text-emerald-600 dark:text-emerald-400",
    blue: "bg-blue-500/10 text-blue-600 dark:text-blue-400",
    violet: "bg-violet-500/10 text-violet-600 dark:text-violet-400",
  };

  return (
    <div className="bg-card rounded-lg border p-6">
      <div className="flex items-center gap-4">
        <div className={`rounded-full p-3 ${colorClasses[color]}`}>
          <Icon className="h-6 w-6" />
        </div>
        <div>
          <p className="text-3xl font-bold">{value}</p>
          <p className="text-muted-foreground text-sm">{label}</p>
        </div>
      </div>
    </div>
  );
}
