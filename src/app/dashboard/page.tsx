"use client";

import * as React from "react";
import Link from "next/link";
import { format } from "date-fns";
import { formatDistanceToNow } from "date-fns";
import {
  Activity,
  ArrowRight,
  Bot,
  Calendar,
  CheckCircle2,
  Clock,
  LayoutDashboard,
  MoreHorizontal,
  Play,
  PlayCircle,
  Plus,
  Settings,
  Users,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { Progress } from "~/components/ui/progress";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Badge } from "~/components/ui/badge";
import { ScrollArea } from "~/components/ui/scroll-area";
import { api } from "~/trpc/react";

export default function DashboardPage() {
  const [studyFilter, setStudyFilter] = React.useState<string | null>(null);

  // --- Data Fetching ---
  const { data: userStudiesData } = api.studies.list.useQuery({
    memberOnly: true,
    limit: 100,
  });
  const userStudies = userStudiesData?.studies ?? [];

  const { data: stats } = api.dashboard.getStats.useQuery({
    studyId: studyFilter ?? undefined,
  });

  const { data: scheduledTrials } = api.trials.list.useQuery({
    studyId: studyFilter ?? undefined,
    status: "scheduled",
    limit: 5,
  });

  const { data: recentActivity } = api.dashboard.getRecentActivity.useQuery({
    limit: 10,
    studyId: studyFilter ?? undefined,
  });

  const { data: studyProgress } = api.dashboard.getStudyProgress.useQuery({
    limit: 5,
    studyId: studyFilter ?? undefined,
  });

  return (
    <div className="flex flex-col space-y-8 animate-in fade-in duration-500">
      {/* Header Section */}
      <div className="flex flex-col gap-4 md:flex-row md:items-center md:justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight text-foreground">Dashboard</h1>
          <p className="text-muted-foreground">
            Overview of your research activities and upcoming tasks.
          </p>
        </div>

        <div className="flex items-center gap-2">
          <Select
            value={studyFilter ?? "all"}
            onValueChange={(value) =>
              setStudyFilter(value === "all" ? null : value)
            }
          >
            <SelectTrigger className="w-[200px] bg-background">
              <SelectValue placeholder="All Studies" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Studies</SelectItem>
              {userStudies.map((study) => (
                <SelectItem key={study.id} value={study.id}>
                  {study.name}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>

          <Button asChild>
            <Link href="/studies/new">
              <Plus className="mr-2 h-4 w-4" /> New Study
            </Link>
          </Button>
        </div>
      </div>

      {/* Stats Cards */}
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        <StatsCard
          title="Total Participants"
          value={stats?.totalParticipants ?? 0}
          icon={Users}
          description="Across all studies"
          trend="+2 this week"
        />
        <StatsCard
          title="Active Trials"
          value={stats?.activeTrials ?? 0}
          icon={Activity}
          description="Currently in progress"

        />
        <StatsCard
          title="Completed Trials"
          value={stats?.completedToday ?? 0}
          icon={CheckCircle2}
          description="Completed today"
        />
        <StatsCard
          title="Scheduled"
          value={stats?.scheduledTrials ?? 0}
          icon={Calendar}
          description="Upcoming sessions"
        />
      </div>

      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-7">

        {/* Main Column: Scheduled Trials & Study Progress */}
        <div className="col-span-4 space-y-4">

          {/* Scheduled Trials */}
          <Card className="col-span-4 border-muted/40 shadow-sm">
            <CardHeader>
              <div className="flex items-center justify-between">
                <div>
                  <CardTitle>Upcoming Sessions</CardTitle>
                  <CardDescription>
                    You have {scheduledTrials?.length ?? 0} scheduled trials coming up.
                  </CardDescription>
                </div>
                <Button variant="ghost" size="sm" asChild>
                  <Link href="/trials?status=scheduled">View All <ArrowRight className="ml-2 h-4 w-4" /></Link>
                </Button>
              </div>
            </CardHeader>
            <CardContent>
              {!scheduledTrials?.length ? (
                <div className="flex h-[150px] flex-col items-center justify-center rounded-md border border-dashed text-center animate-in fade-in-50">
                  <Calendar className="h-8 w-8 text-muted-foreground/50" />
                  <p className="mt-2 text-sm text-muted-foreground">No scheduled trials found.</p>
                  <Button variant="link" size="sm" asChild className="mt-1">
                    <Link href="/trials/new">Schedule a Trial</Link>
                  </Button>
                </div>
              ) : (
                <div className="space-y-4">
                  {scheduledTrials.map((trial) => (
                    <div key={trial.id} className="flex items-center justify-between rounded-lg border p-3 hover:bg-muted/50 transition-colors">
                      <div className="flex items-center gap-4">
                        <div className="flex h-10 w-10 items-center justify-center rounded-full bg-blue-100 dark:bg-blue-900/30 text-blue-600 dark:text-blue-400">
                          <Calendar className="h-5 w-5" />
                        </div>
                        <div>
                          <p className="font-medium text-sm">
                            {trial.participant.participantCode}
                            <span className="ml-2 text-muted-foreground font-normal text-xs">• {trial.experiment.name}</span>
                          </p>
                          <div className="flex items-center gap-2 text-xs text-muted-foreground">
                            <Clock className="h-3 w-3" />
                            {trial.scheduledAt ? format(trial.scheduledAt, "MMM d, h:mm a") : "Unscheduled"}
                          </div>
                        </div>
                      </div>
                      <Button size="sm" className="gap-2" asChild>
                        <Link href={`/wizard/${trial.id}`}>
                          <Play className="h-3.5 w-3.5" /> Start
                        </Link>
                      </Button>
                    </div>
                  ))}
                </div>
              )}
            </CardContent>
          </Card>

          {/* Study Progress */}
          <Card className="border-muted/40 shadow-sm">
            <CardHeader>
              <CardTitle>Study Progress</CardTitle>
              <CardDescription>
                Completion tracking for active studies
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-6">
              {studyProgress?.map((study) => (
                <div key={study.id} className="space-y-2">
                  <div className="flex items-center justify-between text-sm">
                    <div className="font-medium">{study.name}</div>
                    <div className="text-muted-foreground">{study.participants} / {study.totalParticipants} Participants</div>
                  </div>
                  <Progress value={study.progress} className="h-2" />
                </div>
              ))}
              {!studyProgress?.length && (
                <p className="text-sm text-muted-foreground text-center py-4">No active studies to track.</p>
              )}
            </CardContent>
          </Card>

        </div>

        {/* Side Column: Recent Activity & Quick Actions */}
        <div className="col-span-3 space-y-4">
          {/* Quick Actions */}
          <div className="grid grid-cols-2 gap-4">
            <Button variant="outline" className="h-24 flex-col gap-2 hover:border-primary/50 hover:bg-primary/5" asChild>
              <Link href="/experiments/new">
                <Bot className="h-6 w-6 mb-1" />
                <span>New Experim.</span>
              </Link>
            </Button>
            <Button variant="outline" className="h-24 flex-col gap-2 hover:border-primary/50 hover:bg-primary/5" asChild>
              <Link href="/trials/new">
                <PlayCircle className="h-6 w-6 mb-1" />
                <span>Run Trial</span>
              </Link>
            </Button>
          </div>

          {/* Recent Activity */}
          <Card className="border-muted/40 shadow-sm h-full">
            <CardHeader>
              <CardTitle>Recent Activity</CardTitle>
            </CardHeader>
            <CardContent>
              <ScrollArea className="h-[400px] pr-4">
                <div className="space-y-4">
                  {recentActivity?.map((activity) => (
                    <div key={activity.id} className="relative pl-4 pb-1 border-l last:border-0 border-muted-foreground/20">
                      <span className="absolute left-[-5px] top-1 h-2.5 w-2.5 rounded-full bg-primary/30 ring-4 ring-background" />
                      <div className="mb-1 text-sm font-medium leading-none">{activity.title}</div>
                      <div className="text-xs text-muted-foreground mb-1">{activity.description}</div>
                      <div className="text-[10px] text-muted-foreground/70 uppercase">
                        {formatDistanceToNow(activity.time, { addSuffix: true })}
                      </div>
                    </div>
                  ))}
                  {!recentActivity?.length && (
                    <p className="text-sm text-muted-foreground text-center py-8">No recent activity.</p>
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

function StatsCard({
  title,
  value,
  icon: Icon,
  description,
  trend,
}: {
  title: string;
  value: string | number;
  icon: React.ElementType;
  description: string;
  trend?: string;
}) {
  return (
    <Card className="border-muted/40 shadow-sm">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium">{title}</CardTitle>
        <Icon className="h-4 w-4 text-muted-foreground" />
      </CardHeader>
      <CardContent>
        <div className="text-2xl font-bold">{value}</div>
        <p className="text-xs text-muted-foreground">
          {description}
          {trend && <span className="ml-1 text-green-600 dark:text-green-400 font-medium">{trend}</span>}
        </p>
      </CardContent>
    </Card>
  );
}
