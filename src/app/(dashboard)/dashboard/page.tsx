"use client";

import * as React from "react";
import Link from "next/link";
import {
  BarChart3,
  Building,
  FlaskConical,
  TestTube,
  Users,
  Calendar,
  Clock,
  AlertCircle,
  CheckCircle2,
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
import { Progress } from "~/components/ui/progress";
import { api } from "~/trpc/react";

// Dashboard Overview Cards
function OverviewCards() {
  const utils = api.useUtils();

  // Auto-refresh overview data when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void utils.studies.list.invalidate();
      void utils.experiments.getUserExperiments.invalidate();
      void utils.trials.getUserTrials.invalidate();
    }, 60000); // Refresh every minute

    return () => clearInterval(interval);
  }, [utils]);

  const { data: studiesData } = api.studies.list.useQuery(
    { page: 1, limit: 1 },
    {
      staleTime: 1000 * 60 * 2, // 2 minutes
      refetchOnWindowFocus: true,
    },
  );
  const { data: experimentsData } = api.experiments.getUserExperiments.useQuery(
    { page: 1, limit: 1 },
    {
      staleTime: 1000 * 60 * 2, // 2 minutes
      refetchOnWindowFocus: true,
    },
  );
  const { data: trialsData } = api.trials.getUserTrials.useQuery(
    { page: 1, limit: 1 },
    {
      staleTime: 1000 * 60 * 2, // 2 minutes
      refetchOnWindowFocus: true,
    },
  );
  // TODO: Fix participants API call - needs actual study ID
  const participantsData = { pagination: { total: 0 } };

  const cards = [
    {
      title: "Active Studies",
      value: studiesData?.pagination?.total ?? 0,
      description: "Research studies in progress",
      icon: Building,
      color: "text-blue-600",
      bg: "bg-blue-50",
    },
    {
      title: "Experiments",
      value: experimentsData?.pagination?.total ?? 0,
      description: "Experiment protocols designed",
      icon: FlaskConical,
      color: "text-green-600",
      bg: "bg-green-50",
    },
    {
      title: "Participants",
      value: participantsData?.pagination?.total ?? 0,
      description: "Enrolled participants",
      icon: Users,
      color: "text-purple-600",
      bg: "bg-purple-50",
    },
    {
      title: "Trials",
      value: trialsData?.pagination?.total ?? 0,
      description: "Completed trials",
      icon: TestTube,
      color: "text-orange-600",
      bg: "bg-orange-50",
    },
  ];

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
function RecentActivity() {
  // Mock data - replace with actual API calls
  const activities = [
    {
      id: "1",
      type: "trial_completed",
      title: "Trial #142 completed",
      description: "Memory retention study - Participant P001",
      time: "2 hours ago",
      status: "success",
    },
    {
      id: "2",
      type: "experiment_created",
      title: "New experiment protocol",
      description: "Social interaction study v2.1",
      time: "4 hours ago",
      status: "info",
    },
    {
      id: "3",
      type: "participant_enrolled",
      title: "New participant enrolled",
      description: "P045 added to cognitive study",
      time: "6 hours ago",
      status: "success",
    },
    {
      id: "4",
      type: "trial_started",
      title: "Trial #143 started",
      description: "Attention span experiment",
      time: "8 hours ago",
      status: "pending",
    },
  ];

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
                {activity.time}
              </div>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  );
}

// Quick Actions Component
function QuickActions() {
  const actions = [
    {
      title: "Start New Trial",
      description: "Begin a new experimental trial",
      href: "/dashboard/trials/new",
      icon: TestTube,
      color: "bg-blue-500 hover:bg-blue-600",
    },
    {
      title: "Add Participant",
      description: "Enroll a new participant",
      href: "/dashboard/participants/new",
      icon: Users,
      color: "bg-green-500 hover:bg-green-600",
    },
    {
      title: "Create Experiment",
      description: "Design new experiment protocol",
      href: "/dashboard/experiments/new",
      icon: FlaskConical,
      color: "bg-purple-500 hover:bg-purple-600",
    },
    {
      title: "View Analytics",
      description: "Analyze research data",
      href: "/dashboard/analytics",
      icon: BarChart3,
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
function StudyProgress() {
  // Mock data - replace with actual API calls
  const studies = [
    {
      id: "1",
      name: "Cognitive Load Study",
      progress: 75,
      participants: 24,
      totalParticipants: 30,
      status: "active",
    },
    {
      id: "2",
      name: "Social Interaction Research",
      progress: 45,
      participants: 18,
      totalParticipants: 40,
      status: "active",
    },
    {
      id: "3",
      name: "Memory Retention Analysis",
      progress: 90,
      participants: 45,
      totalParticipants: 50,
      status: "completing",
    },
  ];

  return (
    <Card className="col-span-3">
      <CardHeader>
        <CardTitle>Study Progress</CardTitle>
        <CardDescription>
          Current status of active research studies
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="space-y-6">
          {studies.map((study) => (
            <div key={study.id} className="space-y-2">
              <div className="flex items-center justify-between">
                <div className="space-y-1">
                  <p className="text-sm leading-none font-medium">
                    {study.name}
                  </p>
                  <p className="text-muted-foreground text-sm">
                    {study.participants}/{study.totalParticipants} participants
                  </p>
                </div>
                <Badge
                  variant={study.status === "active" ? "default" : "secondary"}
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
      </CardContent>
    </Card>
  );
}

export default function DashboardPage() {
  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h1 className="text-3xl font-bold tracking-tight">Dashboard</h1>
          <p className="text-muted-foreground">
            Welcome to your HRI Studio research platform
          </p>
        </div>
        <div className="flex items-center space-x-2">
          <Badge variant="outline" className="text-xs">
            <Calendar className="mr-1 h-3 w-3" />
            {new Date().toLocaleDateString()}
          </Badge>
        </div>
      </div>

      {/* Overview Cards */}
      <OverviewCards />

      {/* Main Content Grid */}
      <div className="grid gap-4 lg:grid-cols-7">
        <StudyProgress />
        <div className="col-span-4 space-y-4">
          <RecentActivity />
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
