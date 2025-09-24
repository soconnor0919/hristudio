"use client";

import { Activity, Calendar, CheckCircle, FlaskConical } from "lucide-react";
import { DashboardOverviewLayout } from "~/components/ui/page-layout";

interface DashboardContentProps {
  userName: string;
  userRole: string;
  totalStudies: number;
  activeTrials: number;
  scheduledTrials: number;
  completedToday: number;
  canControl: boolean;
  canManage: boolean;
  _recentTrials: unknown[];
}

export function DashboardContent({
  userName,
  userRole,
  totalStudies,
  activeTrials,
  scheduledTrials,
  completedToday,
  canControl,
  canManage,
  _recentTrials,
}: DashboardContentProps) {
  const getWelcomeMessage = () => {
    switch (userRole) {
      case "wizard":
        return "Ready to control trials";
      case "researcher":
        return "Your research platform awaits";
      case "administrator":
        return "System management dashboard";
      default:
        return "Welcome to HRIStudio";
    }
  };

  const quickActions = [
    ...(canManage
      ? [
          {
            title: "Create Study",
            description: "Start a new research study",
            icon: FlaskConical,
            href: "/studies/new",
            variant: "primary" as const,
          },
        ]
      : []),
    ...(canControl
      ? [
          {
            title: "Browse Studies",
            description: "View and manage studies",
            icon: Calendar,
            href: "/studies",
            variant: "default" as const,
          },
        ]
      : []),
  ];

  const stats = [
    {
      title: "Studies",
      value: totalStudies,
      description: "Research studies",
      icon: FlaskConical,
      variant: "primary" as const,
      action: {
        label: "View All",
        href: "/studies",
      },
    },
    {
      title: "Active Trials",
      value: activeTrials,
      description: "Currently running",
      icon: Activity,
      variant: "success" as const,
      ...(canControl && {
        action: {
          label: "View",
          href: "/studies",
        },
      }),
    },
    {
      title: "Scheduled",
      value: scheduledTrials,
      description: "Upcoming trials",
      icon: Calendar,
      variant: "default" as const,
    },
    {
      title: "Completed Today",
      value: completedToday,
      description: "Finished trials",
      icon: CheckCircle,
      variant: "success" as const,
    },
  ];

  const alerts: never[] = [];

  const recentActivity = null;

  return (
    <DashboardOverviewLayout
      title={`${getWelcomeMessage()}, ${userName}`}
      description="Monitor your HRI research activities and manage ongoing studies"
      userName={userName}
      userRole={userRole}
      breadcrumb={[{ label: "Dashboard" }]}
      quickActions={quickActions}
      stats={stats}
      alerts={alerts}
      recentActivity={recentActivity}
    />
  );
}
