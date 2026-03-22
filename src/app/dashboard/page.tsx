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
  BarChart3,
  Settings,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { ScrollArea } from "~/components/ui/scroll-area";
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
    status: undefined,
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

  return (
    <div className="min-h-screen bg-gradient-to-br from-background via-background to-primary/5 p-6 md:p-8">
      {/* Header */}
      <div className="mb-8 flex flex-col gap-4 md:flex-row md:items-center md:justify-between">
        <div>
          <h1 className="text-4xl font-bold tracking-tight">
            {greeting}, {userName.split(" ")[0]}
          </h1>
          <p className="text-muted-foreground mt-1">
            Ready to run your next session?
          </p>
        </div>

        <div className="flex gap-3">
          <Button variant="outline" asChild>
            <Link href="/studies/new">
              <FlaskConical className="mr-2 h-4 w-4" />
              New Study
            </Link>
          </Button>
          <Button asChild className="glow-teal">
            <Link href={userStudies?.studies?.[0]?.id ? `/studies/${userStudies.studies[0].id}/trials/new` : "/studies/new"}>
              <Play className="mr-2 h-4 w-4" />
              Start Trial
            </Link>
          </Button>
        </div>
      </div>

      {/* Live Trials Banner */}
      {liveTrials && liveTrials.length > 0 && (
        <Card className="border-red-200 bg-red-50/50 mb-6 dark:border-red-900 dark:bg-red-900/20">
          <CardContent className="flex items-center gap-4 py-4">
            <div className="relative">
              <Radio className="h-8 w-8 text-red-600 animate-pulse" />
              <span className="absolute -right-1 -top-1 h-3 w-3 rounded-full bg-red-500 animate-ping" />
            </div>
            <div className="flex-1">
              <p className="font-semibold text-red-600 dark:text-red-400">
                {liveTrials.length} Active Session{liveTrials.length > 1 ? "s" : ""}
              </p>
              <p className="text-sm text-red-600/70 dark:text-red-400/70">
                {liveTrials.map((t) => t.participantCode).join(", ")}
              </p>
            </div>
            <Button size="sm" variant="secondary" asChild>
              <Link href={`/studies/${liveTrials?.[0]?.studyId}/trials/${liveTrials?.[0]?.id}/wizard`}>
                View <ChevronRight className="ml-1 h-4 w-4" />
              </Link>
            </Button>
          </CardContent>
        </Card>
      )}

      {/* Stats Row */}
      <div className="mb-8 grid gap-4 md:grid-cols-4">
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
          label="Total Participants"
          value={stats?.scheduledTrials ?? 0}
          icon={Users}
          color="violet"
        />
      </div>

      {/* Main Grid */}
      <div className="grid gap-6 lg:grid-cols-3">
        {/* Studies List */}
        <Card className="lg:col-span-2">
          <CardHeader className="flex flex-row items-center justify-between">
            <div>
              <CardTitle className="flex items-center gap-2">
                <FlaskConical className="h-5 w-5 text-primary" />
                Your Studies
              </CardTitle>
              <CardDescription>Recent studies and quick access</CardDescription>
            </div>
            <Button variant="ghost" size="sm" asChild>
              <Link href="/studies">
                View all <ChevronRight className="ml-1 h-4 w-4" />
              </Link>
            </Button>
          </CardHeader>
          <CardContent>
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
          </CardContent>
        </Card>

        {/* Quick Links & Recent */}
        <div className="space-y-6">
          {/* Quick Actions */}
          <Card>
            <CardHeader>
              <CardTitle className="text-lg">Quick Actions</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href="/studies/new">
                  <Plus className="mr-3 h-4 w-4" />
                  Create New Study
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={userStudies?.studies?.[0]?.id ? `/studies/${userStudies.studies[0].id}/experiments/new` : "/studies"}>
                  <FlaskConical className="mr-3 h-4 w-4" />
                  Design Experiment
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={userStudies?.studies?.[0]?.id ? `/studies/${userStudies.studies[0].id}/participants/new` : "/studies"}>
                  <Users className="mr-3 h-4 w-4" />
                  Add Participant
                </Link>
              </Button>
              <Button variant="outline" className="w-full justify-start" asChild>
                <Link href={userStudies?.studies?.[0]?.id ? `/studies/${userStudies.studies[0].id}/trials/new` : "/studies"}>
                  <Play className="mr-3 h-4 w-4" />
                  Start Trial
                </Link>
              </Button>
            </CardContent>
          </Card>

          {/* Recent Trials */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2 text-lg">
                <Clock className="h-4 w-4" />
                Recent Trials
              </CardTitle>
            </CardHeader>
            <CardContent>
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
            </CardContent>
          </Card>
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
    teal: "bg-teal-500/10 text-teal-600 dark:text-teal-400",
    emerald: "bg-emerald-500/10 text-emerald-600 dark:text-emerald-400",
    blue: "bg-blue-500/10 text-blue-600 dark:text-blue-400",
    violet: "bg-violet-500/10 text-violet-600 dark:text-violet-400",
  };

  return (
    <Card>
      <CardContent className="flex items-center gap-4 p-6">
        <div className={`rounded-full p-3 ${colorClasses[color]}`}>
          <Icon className="h-6 w-6" />
        </div>
        <div>
          <p className="text-3xl font-bold">{value}</p>
          <p className="text-muted-foreground text-sm">{label}</p>
        </div>
      </CardContent>
    </Card>
  );
}
