"use client";

import * as React from "react";
import Link from "next/link";
import { format } from "date-fns";
import { formatDistanceToNow } from "date-fns";
import {
  Activity,
  ArrowRight,
  Calendar,
  CheckCircle,
  CheckCircle2,
  Clock,
  FlaskConical,
  HelpCircle,
  LayoutDashboard,
  MoreHorizontal,
  Play,
  PlayCircle,
  Plus,
  Search,
  Settings,
  Users,
  Radio,
  Gamepad2,
  AlertTriangle,
  Bot,
  User,
  MessageSquare,
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
import { useTour } from "~/components/onboarding/TourProvider";
import { useSession } from "next-auth/react";

export default function DashboardPage() {
  const { startTour } = useTour();
  const { data: session } = useSession();
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

  const { data: liveTrials } = api.dashboard.getLiveTrials.useQuery(
    { studyId: studyFilter ?? undefined },
    { refetchInterval: 5000 }
  );

  const { data: recentActivity } = api.dashboard.getRecentActivity.useQuery({
    limit: 15,
    studyId: studyFilter ?? undefined,
  });

  const { data: studyProgress } = api.dashboard.getStudyProgress.useQuery({
    limit: 5,
    studyId: studyFilter ?? undefined,
  });

  const userName = session?.user?.name ?? "Researcher";

  const getWelcomeMessage = () => {
    const hour = new Date().getHours();
    let greeting = "Good evening";
    if (hour < 12) greeting = "Good morning";
    else if (hour < 18) greeting = "Good afternoon";

    return `${greeting}, ${userName.split(" ")[0]}`;
  };

  return (
    <div className="space-y-8 animate-in fade-in duration-500">
      {/* Header Section */}
      <div id="dashboard-header" className="flex flex-col gap-4 md:flex-row md:items-center md:justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight text-foreground">
            {getWelcomeMessage()}
          </h1>
          <p className="text-muted-foreground">
            Here's what's happening with your research today.
          </p>
        </div>

        <div className="flex items-center gap-2">
          <Button variant="ghost" size="icon" onClick={() => startTour("dashboard")} title="Start Tour">
            <HelpCircle className="h-5 w-5" />
          </Button>
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

          <Button id="tour-new-study" asChild>
            <Link href="/studies/new">
              <Plus className="mr-2 h-4 w-4" /> New Study
            </Link>
          </Button>
        </div>
      </div>

      {/* Main Stats Grid */}
      <div id="tour-dashboard-stats" className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        <StatsCard
          title="Active Trials"
          value={stats?.activeTrials ?? 0}
          icon={Activity}
          description="Currently running sessions"
          iconColor="text-emerald-500"
        />
        <StatsCard
          title="Completed Today"
          value={stats?.completedToday ?? 0}
          icon={CheckCircle}
          description="Successful completions"
          iconColor="text-blue-500"
        />
        <StatsCard
          title="Scheduled"
          value={stats?.scheduledTrials ?? 0}
          icon={Calendar}
          description="Upcoming sessions"
          iconColor="text-violet-500"
        />
        <StatsCard
          title="Total Interventions"
          value={stats?.totalInterventions ?? 0}
          icon={Gamepad2}
          description="Wizard manual overrides"
          iconColor="text-orange-500"
        />
      </div>

      {/* Action Center & Recent Activity */}
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-7">

        {/* Quick Actions Card */}
        <Card className="col-span-3 bg-gradient-to-br from-primary/5 to-background border-primary/20 h-fit">
          <CardHeader>
            <CardTitle>Quick Actions</CardTitle>
            <CardDescription>Common tasks to get you started</CardDescription>
          </CardHeader>
          <CardContent className="grid gap-4">
            <Button
              variant="outline"
              className="justify-start h-auto py-4 px-4 border-primary/20 hover:border-primary/50 hover:bg-primary/5 group"
              asChild
            >
              <Link href="/studies/new">
                <div className="p-2 bg-primary/10 rounded-full mr-4 group-hover:bg-primary/20 transition-colors">
                  <FlaskConical className="h-5 w-5 text-primary" />
                </div>
                <div className="text-left">
                  <div className="font-semibold">Create New Study</div>
                  <div className="text-xs text-muted-foreground font-normal">
                    Design a new experiment protocol
                  </div>
                </div>
                <ArrowRight className="ml-auto h-4 w-4 text-muted-foreground group-hover:text-primary opacity-0 group-hover:opacity-100 transition-all" />
              </Link>
            </Button>

            <Button
              variant="outline"
              className="justify-start h-auto py-4 px-4 group"
              asChild
            >
              <Link href="/studies">
                <div className="p-2 bg-secondary rounded-full mr-4">
                  <Search className="h-5 w-5 text-foreground" />
                </div>
                <div className="text-left">
                  <div className="font-semibold">Browse Studies</div>
                  <div className="text-xs text-muted-foreground font-normal">
                    Find and manage existing studies
                  </div>
                </div>
              </Link>
            </Button>

            <Button
              variant="outline"
              className="justify-start h-auto py-4 px-4 group"
              asChild
            >
              <Link href="/trials">
                <div className="p-2 bg-emerald-500/10 rounded-full mr-4">
                  <Activity className="h-5 w-5 text-emerald-600" />
                </div>
                <div className="text-left">
                  <div className="font-semibold">Monitor Active Trials</div>
                  <div className="text-xs text-muted-foreground font-normal">
                    Jump into the Wizard Interface
                  </div>
                </div>
              </Link>
            </Button>
          </CardContent>
        </Card>

        {/* Recent Activity Card */}
        <Card id="tour-recent-activity" className="col-span-4 border-muted/40 shadow-sm">
          <CardHeader>
            <CardTitle>Recent Activity</CardTitle>
            <CardDescription>
              Your latest interactions across the platform
            </CardDescription>
          </CardHeader>
          <CardContent>
            <ScrollArea className="h-[400px] pr-4">
              <div className="space-y-4">
                {recentActivity?.map((activity) => {
                  let eventColor = "bg-primary/30 ring-background";
                  let Icon = Activity;
                  if (activity.type === "trial_started") {
                    eventColor = "bg-blue-500 ring-blue-100 dark:ring-blue-900";
                    Icon = PlayCircle;
                  } else if (activity.type === "trial_completed") {
                    eventColor = "bg-green-500 ring-green-100 dark:ring-green-900";
                    Icon = CheckCircle;
                  } else if (activity.type === "error") {
                    eventColor = "bg-red-500 ring-red-100 dark:ring-red-900";
                    Icon = AlertTriangle;
                  } else if (activity.type === "intervention") {
                    eventColor = "bg-orange-500 ring-orange-100 dark:ring-orange-900";
                    Icon = Gamepad2;
                  } else if (activity.type === "annotation") {
                    eventColor = "bg-yellow-500 ring-yellow-100 dark:ring-yellow-900";
                    Icon = MessageSquare;
                  }

                  return (
                    <div key={activity.id} className="relative pl-6 pb-4 border-l last:border-0 border-muted-foreground/20">
                      <span className={`absolute left-[-9px] top-0 h-4 w-4 rounded-full flex items-center justify-center ring-4 ${eventColor}`}>
                        <Icon className="h-2.5 w-2.5 text-white" />
                      </span>
                      <div className="mb-0.5 text-sm font-medium leading-none">{activity.title}</div>
                      <div className="text-xs text-muted-foreground mb-1">{activity.description}</div>
                      <div className="text-[10px] text-muted-foreground/70 uppercase font-mono">
                        {formatDistanceToNow(new Date(activity.time), { addSuffix: true })}
                      </div>
                    </div>
                  )
                })}
                {!recentActivity?.length && (
                  <div className="flex flex-col items-center justify-center py-8 text-center text-muted-foreground">
                    <Clock className="h-10 w-10 mb-3 opacity-20" />
                    <p>No recent activity recorded.</p>
                    <p className="text-xs mt-1">Start a trial to see experiment events stream here.</p>
                  </div>
                )}
              </div>
            </ScrollArea>
          </CardContent>
        </Card>
      </div>

      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-7">
        {/* Live Trials */}
        <Card id="tour-live-trials" className={`${liveTrials && liveTrials.length > 0 ? 'border-primary shadow-sm bg-primary/5' : 'border-muted/40'} col-span-4 transition-colors duration-500`}>
          <CardHeader>
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="flex items-center gap-2">
                  Live Sessions
                  {liveTrials && liveTrials.length > 0 && <span className="relative flex h-3 w-3">
                    <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-red-400 opacity-75"></span>
                    <span className="relative inline-flex rounded-full h-3 w-3 bg-red-500"></span>
                  </span>}
                </CardTitle>
                <CardDescription>
                  Currently running trials in the Wizard interface
                </CardDescription>
              </div>
              <Button variant="ghost" size="sm" asChild>
                <Link href="/trials">View All <ArrowRight className="ml-2 h-4 w-4" /></Link>
              </Button>
            </div>
          </CardHeader>
          <CardContent>
            {!liveTrials?.length ? (
              <div className="flex h-[150px] flex-col items-center justify-center rounded-md border border-dashed border-muted-foreground/30 text-center animate-in fade-in-50 bg-background/50">
                <Radio className="h-8 w-8 text-muted-foreground/50 mb-2" />
                <p className="text-sm text-muted-foreground">No trials are currently running.</p>
                <Button variant="link" size="sm" asChild className="mt-1">
                  <Link href="/trials">Start a Trial</Link>
                </Button>
              </div>
            ) : (
              <div className="space-y-4">
                {liveTrials.map((trial) => (
                  <div key={trial.id} className="flex items-center justify-between rounded-lg border border-primary/20 p-3 bg-background shadow-sm hover:shadow transition-all duration-200">
                    <div className="flex items-center gap-4">
                      <div className="flex h-10 w-10 items-center justify-center rounded-full bg-red-100 dark:bg-red-900/40 text-red-600 dark:text-red-400">
                        <Radio className="h-5 w-5 animate-pulse" />
                      </div>
                      <div>
                        <p className="font-medium text-sm">
                          {trial.participantCode}
                          <span className="ml-2 text-muted-foreground font-normal text-xs">• {trial.experimentName}</span>
                        </p>
                        <div className="flex items-center gap-2 text-xs text-muted-foreground">
                          <Clock className="h-3 w-3" />
                          Started {trial.startedAt ? formatDistanceToNow(new Date(trial.startedAt), { addSuffix: true }) : 'just now'}
                        </div>
                      </div>
                    </div>
                    <Button size="sm" className="gap-2 bg-primary hover:bg-primary/90" asChild>
                      <Link href={`/wizard/${trial.id}`}>
                        <Play className="h-3.5 w-3.5" /> Spectate / Jump In
                      </Link>
                    </Button>
                  </div>
                ))}
              </div>
            )}
          </CardContent>
        </Card>

        {/* Study Progress */}
        <Card className="col-span-3 border-muted/40 shadow-sm">
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
    </div>
  );
}

function StatsCard({
  title,
  value,
  icon: Icon,
  description,
  trend,
  iconColor,
}: {
  title: string;
  value: string | number;
  icon: React.ElementType;
  description: string;
  trend?: string;
  iconColor?: string;
}) {
  return (
    <Card className="border-muted/40 shadow-sm hover:shadow-md transition-all duration-200 hover:border-primary/20">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-sm font-medium">{title}</CardTitle>
        <Icon className={`h-4 w-4 ${iconColor || "text-muted-foreground"}`} />
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
