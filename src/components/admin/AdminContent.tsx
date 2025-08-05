"use client";

import { Users } from "lucide-react";
import { AdminUserTable } from "~/components/admin/admin-user-table";
import { DashboardOverviewLayout } from "~/components/ui/page-layout";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

interface AdminContentProps {
  userName: string;
  userEmail: string;
}

export function AdminContent({ userName, userEmail }: AdminContentProps) {
  const quickActions = [
    {
      title: "Manage Users",
      description: "View and manage user accounts",
      icon: Users,
      href: "/admin/users",
      variant: "primary" as const,
    },
  ];

  const stats: any[] = [];

  const alerts: any[] = [];

  const recentActivity = (
    <div className="space-y-6">
      <Card>
        <CardHeader>
          <CardTitle>User Management</CardTitle>
          <CardDescription>
            Manage user accounts and role assignments
          </CardDescription>
        </CardHeader>
        <CardContent>
          <AdminUserTable />
        </CardContent>
      </Card>
    </div>
  );

  return (
    <DashboardOverviewLayout
      title="System Administration"
      description="Manage users, monitor system performance, and configure platform settings"
      userName={userName}
      userRole="administrator"
      breadcrumb={[
        { label: "Dashboard", href: "/dashboard" },
        { label: "Administration" },
      ]}
      quickActions={quickActions}
      stats={stats}
      alerts={alerts}
      recentActivity={recentActivity}
    />
  );
}
