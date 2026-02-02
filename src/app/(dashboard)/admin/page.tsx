"use client";

import * as React from "react";
import Link from "next/link";
import {
  Shield,
  Users,
  Database,
  Settings,
  Activity,
  AlertTriangle,
  CheckCircle2,
  Clock,
  BarChart3,
  FileText,
  UserCheck,
  Plus,
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

import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { PageHeader, ActionButton } from "~/components/ui/page-header";

// System Overview Cards
function SystemOverview() {
  // Mock data - replace with actual API calls when available
  const stats = {
    totalUsers: 0,
    activeStudies: 0,
    systemHealth: 100,
    pluginRepositories: 0,
  };

  const cards = [
    {
      title: "Total Users",
      value: stats.totalUsers,
      description: "Registered platform users",
      icon: Users,
      color: "text-blue-600",
      bg: "bg-blue-50",
    },
    {
      title: "Active Studies",
      value: stats.activeStudies,
      description: "Currently running studies",
      icon: Activity,
      color: "text-green-600",
      bg: "bg-green-50",
    },
    {
      title: "System Health",
      value: `${stats.systemHealth}%`,
      description: "Overall system status",
      icon: CheckCircle2,
      color: "text-emerald-600",
      bg: "bg-emerald-50",
    },
    {
      title: "Plugin Repos",
      value: stats.pluginRepositories,
      description: "Configured repositories",
      icon: Database,
      color: "text-purple-600",
      bg: "bg-purple-50",
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

// Recent Admin Activity
function RecentActivity() {
  // Mock data - replace with actual audit log API
  const activities = [
    {
      id: "1",
      type: "user_created",
      title: "New user registered",
      description: "researcher@university.edu joined the platform",
      time: "2 hours ago",
      status: "success",
    },
    {
      id: "2",
      type: "repository_added",
      title: "Plugin repository added",
      description: "Official TurtleBot3 repository configured",
      time: "4 hours ago",
      status: "info",
    },
    {
      id: "3",
      type: "role_updated",
      title: "User role modified",
      description: "john.doe@lab.edu promoted to researcher",
      time: "6 hours ago",
      status: "success",
    },
    {
      id: "4",
      type: "system_update",
      title: "System maintenance",
      description: "Database optimization completed",
      time: "1 day ago",
      status: "success",
    },
  ];

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "success":
        return <CheckCircle2 className="h-4 w-4 text-green-600" />;
      case "pending":
        return <Clock className="h-4 w-4 text-yellow-600" />;
      case "error":
        return <AlertTriangle className="h-4 w-4 text-red-600" />;
      default:
        return <Activity className="h-4 w-4 text-blue-600" />;
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Recent Activity</CardTitle>
        <CardDescription>
          Latest administrative actions and system events
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

// System Status
function SystemStatus() {
  // Mock data - replace with actual system health checks
  const services = [
    {
      name: "Database",
      status: "healthy",
      uptime: "99.9%",
      responseTime: "12ms",
    },
    {
      name: "Authentication",
      status: "healthy",
      uptime: "100%",
      responseTime: "8ms",
    },
    {
      name: "File Storage",
      status: "healthy",
      uptime: "99.8%",
      responseTime: "45ms",
    },
    {
      name: "Plugin System",
      status: "healthy",
      uptime: "99.5%",
      responseTime: "23ms",
    },
  ];

  const getStatusBadge = (status: string) => {
    switch (status) {
      case "healthy":
        return <Badge className="bg-green-100 text-green-800">Healthy</Badge>;
      case "warning":
        return <Badge className="bg-yellow-100 text-yellow-800">Warning</Badge>;
      case "error":
        return <Badge className="bg-red-100 text-red-800">Error</Badge>;
      default:
        return <Badge variant="secondary">Unknown</Badge>;
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>System Status</CardTitle>
        <CardDescription>
          Current status of core system services
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">
          {services.map((service) => (
            <div
              key={service.name}
              className="flex items-center justify-between"
            >
              <div className="space-y-1">
                <p className="text-sm font-medium">{service.name}</p>
                <p className="text-muted-foreground text-xs">
                  Uptime: {service.uptime} • Response: {service.responseTime}
                </p>
              </div>
              {getStatusBadge(service.status)}
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  );
}

// Quick Admin Actions
function QuickActions() {
  const actions = [
    {
      title: "Manage Users",
      description: "View and modify user accounts",
      href: "/admin/users",
      icon: Users,
      disabled: true, // Enable when route exists
    },
    {
      title: "Plugin Repositories",
      description: "Configure plugin sources",
      href: "/admin/repositories",
      icon: Database,
      disabled: false,
    },
    {
      title: "System Settings",
      description: "Configure platform settings",
      href: "/admin/settings",
      icon: Settings,
      disabled: true, // Enable when route exists
    },
    {
      title: "View Audit Logs",
      description: "Review system activity",
      href: "/admin/audit",
      icon: FileText,
      disabled: true, // Enable when route exists
    },
    {
      title: "Role Management",
      description: "Manage user permissions",
      href: "/admin/roles",
      icon: UserCheck,
      disabled: true, // Enable when route exists
    },
    {
      title: "Analytics",
      description: "Platform usage statistics",
      href: "/admin/analytics",
      icon: BarChart3,
      disabled: true, // Enable when route exists
    },
  ];

  return (
    <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
      {actions.map((action) => (
        <Card
          key={action.title}
          className={`group transition-all ${
            action.disabled
              ? "cursor-not-allowed opacity-50"
              : "cursor-pointer hover:shadow-md"
          }`}
        >
          <CardContent className="p-6">
            <div className="mb-3 flex items-center space-x-3">
              <div className="rounded-lg bg-slate-100 p-2">
                <action.icon className="h-5 w-5 text-slate-600" />
              </div>
              <div className="flex-1">
                <h3 className="text-sm font-medium">{action.title}</h3>
              </div>
            </div>
            <p className="text-muted-foreground mb-4 text-sm">
              {action.description}
            </p>
            {action.disabled ? (
              <Button disabled className="w-full" variant="outline">
                Coming Soon
              </Button>
            ) : (
              <Button asChild className="w-full" variant="outline">
                <Link href={action.href}>Access</Link>
              </Button>
            )}
          </CardContent>
        </Card>
      ))}
    </div>
  );
}

export default function AdminPage() {
  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Administration" },
  ]);

  return (
    <div className="space-y-6">
      {/* Header */}
      <PageHeader
        title="Administration"
        description="System administration and platform management"
        icon={Shield}
        actions={
          <div className="flex items-center space-x-2">
            <Badge variant="secondary" className="bg-red-100 text-red-800">
              <Shield className="mr-1 h-3 w-3" />
              Administrator
            </Badge>
            <ActionButton href="/admin/repositories">
              <Plus className="mr-2 h-4 w-4" />
              Add Repository
            </ActionButton>
          </div>
        }
      />

      {/* System Overview */}
      <SystemOverview />

      {/* Main Content Grid */}
      <div className="grid gap-6 lg:grid-cols-2">
        <div className="space-y-6">
          <RecentActivity />
        </div>
        <div className="space-y-6">
          <SystemStatus />
        </div>
      </div>

      {/* Quick Actions */}
      <div className="space-y-4">
        <h2 className="text-xl font-semibold">Administrative Tools</h2>
        <QuickActions />
      </div>

      {/* Security Notice */}
      <Card className="border-amber-200 bg-amber-50">
        <CardContent className="pt-6">
          <div className="flex items-start space-x-3">
            <div className="rounded-lg bg-amber-100 p-2">
              <AlertTriangle className="h-5 w-5 text-amber-600" />
            </div>
            <div className="flex-1">
              <h3 className="mb-1 font-semibold text-amber-900">
                Administrator Access
              </h3>
              <p className="text-sm text-amber-800">
                You have full administrative access to this system. All actions
                are logged for security purposes. Please use these privileges
                responsibly.
              </p>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
