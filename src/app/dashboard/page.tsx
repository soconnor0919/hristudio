"use client";

import * as React from "react";
import Link from "next/link";
import {
  Building,
  FlaskConical,
  TestTube,
  Users,
  Calendar,
  Clock,
  AlertCircle,
  CheckCircle2,
} from "lucide-react";
import { formatDistanceToNow } from "date-fns";

import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Progress } from "~/components/ui/progress";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { api } from "~/trpc/react";

// Dashboard Overview Cards
function OverviewCards({ studyFilter }: { studyFilter: string | null }) {
  const { data: stats, isLoading } = api.dashboard.getStats.useQuery({
    studyId: studyFilter ?? undefined,
  });

  const cards = [
    {
      title: "Active Studies",
      value: stats?.totalStudies ?? 0,
      description: "Research studies you have access to",
      icon: Building,
      color: "text-blue-600",
      bg: "bg-blue-50",
    },
    {
      title: "Experiments",
      value: stats?.totalExperiments ?? 0,
      description: "Experiment protocols designed",
      icon: FlaskConical,
      color: "text-green-600",
      bg: "bg-green-50",
    },
    {
      title: "Participants",
      value: stats?.totalParticipants ?? 0,
      description: "Enrolled participants",
      icon: Users,
      color: "text-purple-600",
      bg: "bg-purple-50",
    },
    {
      title: "Trials",
      value: stats?.totalTrials ?? 0,
      description: "Total trials conducted",
      icon: TestTube,
      color: "text-orange-600",
      bg: "bg-orange-50",
    },
  ];

  if (isLoading) {
    return (
      <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
        {Array.from({ length: 4 }).map((_, i) => (
          <Card key={i}>
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
              <div className="bg-muted h-4 w-20 animate-pulse rounded" />
              <div className="bg-muted h-8 w-8 animate-pulse rounded" />
            </CardHeader>
            <CardContent>
              <div className="bg-muted h-8 w-12 animate-pulse rounded" />
              <div className="bg-muted mt-2 h-3 w-24 animate-pulse rounded" />
            </CardContent>
          </Card>
        ))}
      </div>
    );
  }

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {cards.map((card) => (
        <Card key={card.title}>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">{card.title}</CardTitle>
            <div className={`rounded-md p-2 ${card.bg}`}>
              <card.icon className={`h-4 w-4 ${card.color}`} />
            </div>
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{card.value}</div>
            <p className="text-muted-foreground text-xs">{card.description}</p>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}

// Recent Activity Component
function RecentActivity({ studyFilter }: { studyFilter: string | null }) {
  const { data: activities = [], isLoading } =
    api.dashboard.getRecentActivity.useQuery({
      limit: 8,
      studyId: studyFilter ?? undefined,
    });

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "success":
        return <CheckCircle2 className="h-4 w-4 text-green-600" />;
      case "pending":
        return <Clock className="h-4 w-4 text-yellow-600" />;
      case "error":
        return <AlertCircle className="h-4 w-4 text-red-600" />;
      default:
        return <AlertCircle className="h-4 w-4 text-blue-600" />;
    }
  };

  return (
    <Card className="col-span-4">
      <CardHeader>
        <CardTitle>Recent Activity</CardTitle>
        <CardDescription>
          Latest updates from your research platform
        </CardDescription>
      </CardHeader>
      <CardContent>
        {isLoading ? (
          <div className="space-y-4">
            {Array.from({ length: 4 }).map((_, i) => (
              <div key={i} className="flex items-center space-x-4">
                <div className="bg-muted h-4 w-4 animate-pulse rounded-full" />
                <div className="flex-1 space-y-2">
                  <div className="bg-muted h-4 w-3/4 animate-pulse rounded" />
                  <div className="bg-muted h-3 w-1/2 animate-pulse rounded" />
                </div>
                <div className="bg-muted h-3 w-16 animate-pulse rounded" />
              </div>
            ))}
          </div>
        ) : activities.length === 0 ? (
          <div className="py-8 text-center">
            <AlertCircle className="text-muted-foreground mx-auto h-8 w-8" />
            <p className="text-muted-foreground mt-2 text-sm">
              No recent activity
            </p>
          </div>
        ) : (
          <div className="space-y-4">
            {activities.map((activity) => (
              <div key={activity.id} className="flex items-center space-x-4">
                {getStatusIcon(activity.status)}
                <div className="flex-1 space-y-1">
                  <p className="text-sm leading-none font-medium">
                    {activity.title}
                  </p>
                  <p className="text-muted-foreground text-sm">
                    {activity.description}
                  </p>
                </div>
                <div className="text-muted-foreground text-sm">
                  {formatDistanceToNow(activity.time, { addSuffix: true })}
                </div>
              </div>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  );
}

// Quick Actions Component
function QuickActions() {
  const actions = [
    {
      title: "Create Study",
      description: "Start a new research study",
      href: "/studies/new",
      icon: Building,
      color: "bg-blue-500 hover:bg-blue-600",
    },
    {
      title: "Browse Studies",
      description: "View and manage your studies",
      href: "/studies",
      icon: Building,
      color: "bg-green-500 hover:bg-green-600",
    },
    {
      title: "Create Experiment",
      description: "Design new experiment protocol",
      href: "/experiments/new",
      icon: FlaskConical,
      color: "bg-purple-500 hover:bg-purple-600",
    },
    {
      title: "Browse Experiments",
      description: "View experiment templates",
      href: "/experiments",
      icon: FlaskConical,
      color: "bg-orange-500 hover:bg-orange-600",
    },
  ];

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
      {actions.map((action) => (
        <Card
          key={action.title}
          className="group cursor-pointer transition-all hover:shadow-md"
        >
          <CardContent className="p-6">
            <Button asChild className={`w-full ${action.color} text-white`}>
              <Link href={action.href}>
                <action.icon className="mr-2 h-4 w-4" />
                {action.title}
              </Link>
            </Button>
            <p className="text-muted-foreground mt-2 text-sm">
              {action.description}
            </p>
          </CardContent>
        </Card>
      ))}
    </div>
  );
}

// Study Progress Component
function StudyProgress({ studyFilter }: { studyFilter: string | null }) {
  const { data: studies = [], isLoading } =
    api.dashboard.getStudyProgress.useQuery({
      limit: 5,
      studyId: studyFilter ?? undefined,
    });

  return (
    <Card className="col-span-3">
      <CardHeader>
        <CardTitle>Study Progress</CardTitle>
        <CardDescription>
          Current status of active research studies
        </CardDescription>
      </CardHeader>
      <CardContent>
        {isLoading ? (
          <div className="space-y-6">
            {Array.from({ length: 3 }).map((_, i) => (
              <div key={i} className="space-y-2">
                <div className="flex items-center justify-between">
                  <div className="space-y-1">
                    <div className="bg-muted h-4 w-32 animate-pulse rounded" />
                    <div className="bg-muted h-3 w-24 animate-pulse rounded" />
                  </div>
                  <div className="bg-muted h-5 w-16 animate-pulse rounded" />
                </div>
                <div className="bg-muted h-2 w-full animate-pulse rounded" />
                <div className="bg-muted h-3 w-16 animate-pulse rounded" />
              </div>
            ))}
          </div>
        ) : studies.length === 0 ? (
          <div className="py-8 text-center">
            <Building className="text-muted-foreground mx-auto h-8 w-8" />
            <p className="text-muted-foreground mt-2 text-sm">
              No active studies found
            </p>
            <p className="text-muted-foreground text-xs">
              Create a study to get started
            </p>
          </div>
        ) : (
          <div className="space-y-6">
            {studies.map((study) => (
              <div key={study.id} className="space-y-2">
                <div className="flex items-center justify-between">
                  <div className="space-y-1">
                    <p className="text-sm leading-none font-medium">
                      {study.name}
                    </p>
                    <p className="text-muted-foreground text-sm">
                      {study.participants}/{study.totalParticipants} completed
                      trials
                    </p>
                  </div>
                  <Badge
                    variant={
                      study.status === "active" ? "default" : "secondary"
                    }
                  >
                    {study.status}
                  </Badge>
                </div>
                <Progress value={study.progress} className="h-2" />
                <p className="text-muted-foreground text-xs">
                  {study.progress}% complete
                </p>
              </div>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  );
}

export default function DashboardPage() {
  const [studyFilter, setStudyFilter] = React.useState<string | null>(null);

  // Get user studies for filter dropdown
  const { data: userStudiesData } = api.studies.list.useQuery({
    memberOnly: true,
    limit: 100,
  });

  const userStudies = userStudiesData?.studies ?? [];

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">
            Dashboard
            {studyFilter && (
              <Badge variant="secondary" className="ml-2">
                {userStudies.find((s) => s.id === studyFilter)?.name}
              </Badge>
            )}
          </h1>
          <p className="text-muted-foreground">
            {studyFilter
              ? "Study-specific dashboard view"
              : "Welcome to your HRI Studio research platform"}
          </p>
        </div>
        <div className="flex items-center space-x-4">
          <div className="flex items-center space-x-2">
            <span className="text-muted-foreground text-sm">
              Filter by study:
            </span>
            <Select
              value={studyFilter ?? "all"}
              onValueChange={(value) =>
                setStudyFilter(value === "all" ? null : value)
              }
            >
              <SelectTrigger className="w-[200px]">
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
          </div>
          <Badge variant="outline" className="text-xs">
            <Calendar className="mr-1 h-3 w-3" />
            {new Date().toLocaleDateString()}
          </Badge>
        </div>
      </div>

      {/* Overview Cards */}
      <OverviewCards studyFilter={studyFilter} />

      {/* Main Content Grid */}
      <div className="grid gap-4 lg:grid-cols-7">
        <StudyProgress studyFilter={studyFilter} />
        <div className="col-span-4 space-y-4">
          <RecentActivity studyFilter={studyFilter} />
        </div>
      </div>

      {/* Quick Actions */}
      <div className="space-y-4">
        <h2 className="text-xl font-semibold">Quick Actions</h2>
        <QuickActions />
      </div>
    </div>
  );
}
