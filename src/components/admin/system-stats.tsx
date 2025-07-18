"use client";

import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";

export function SystemStats() {
  // TODO: Implement admin.getSystemStats API endpoint
  // const { data: stats, isLoading } = api.admin.getSystemStats.useQuery({});
  const isLoading = false;

  if (isLoading) {
    return (
      <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-4">
        {Array.from({ length: 4 }).map((_, i) => (
          <Card key={i} className="animate-pulse">
            <CardHeader className="pb-2">
              <div className="h-4 w-20 rounded bg-slate-200"></div>
            </CardHeader>
            <CardContent>
              <div className="mb-2 h-8 w-12 rounded bg-slate-200"></div>
              <div className="h-3 w-24 rounded bg-slate-200"></div>
            </CardContent>
          </Card>
        ))}
      </div>
    );
  }

  // Mock data for now since we don't have the actual admin router implemented
  const mockStats = {
    totalUsers: 42,
    totalStudies: 15,
    totalExperiments: 38,
    totalTrials: 127,
    activeTrials: 3,
    systemHealth: "healthy",
    uptime: "7 days, 14 hours",
    storageUsed: "2.3 GB",
  };

  const displayStats = mockStats;

  return (
    <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-4">
      {/* Total Users */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Total Users
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-2xl font-bold">{displayStats.totalUsers}</div>
          <div className="mt-1 flex items-center gap-2">
            <Badge variant="secondary" className="text-xs">
              All roles
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* Total Studies */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Studies
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-2xl font-bold">{displayStats.totalStudies}</div>
          <div className="mt-1 flex items-center gap-2">
            <Badge variant="secondary" className="text-xs">
              Active
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* Total Experiments */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Experiments
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-2xl font-bold">
            {displayStats.totalExperiments}
          </div>
          <div className="mt-1 flex items-center gap-2">
            <Badge variant="secondary" className="text-xs">
              Published
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* Total Trials */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Trials
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-2xl font-bold">{displayStats.totalTrials}</div>
          <div className="mt-1 flex items-center gap-2">
            <Badge variant="outline" className="text-xs">
              {displayStats.activeTrials} running
            </Badge>
          </div>
        </CardContent>
      </Card>

      {/* System Health */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            System Health
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="flex items-center gap-2">
            <div className="flex h-3 w-3 items-center justify-center">
              <div className="h-2 w-2 rounded-full bg-green-500"></div>
            </div>
            <span className="text-sm font-medium text-green-700">
              {displayStats.systemHealth === "healthy" ? "Healthy" : "Issues"}
            </span>
          </div>
          <div className="mt-1 text-xs text-slate-500">
            All services operational
          </div>
        </CardContent>
      </Card>

      {/* Uptime */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Uptime
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-sm font-medium">{displayStats.uptime}</div>
          <div className="mt-1 text-xs text-slate-500">Since last restart</div>
        </CardContent>
      </Card>

      {/* Storage Usage */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Storage Used
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="text-sm font-medium">{displayStats.storageUsed}</div>
          <div className="mt-1 text-xs text-slate-500">Media & database</div>
        </CardContent>
      </Card>

      {/* Recent Activity */}
      <Card>
        <CardHeader className="pb-2">
          <CardTitle className="text-sm font-medium text-slate-600">
            Recent Activity
          </CardTitle>
        </CardHeader>
        <CardContent>
          <div className="space-y-1">
            <div className="text-xs text-slate-600">2 trials started today</div>
            <div className="text-xs text-slate-600">1 new user registered</div>
            <div className="text-xs text-slate-600">
              3 experiments published
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
