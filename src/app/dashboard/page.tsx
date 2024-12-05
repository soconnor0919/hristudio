'use client';

import { useCallback, useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { BookOpen, Settings2 } from "lucide-react";
import { useToast } from "~/hooks/use-toast";
import { getApiUrl } from "~/lib/fetch-utils";
import { Skeleton } from "~/components/ui/skeleton";
import { useActiveStudy } from "~/context/active-study";

interface DashboardStats {
  studyCount: number;
  activeInvitationCount: number;
}

export default function Dashboard() {
  const [stats, setStats] = useState<DashboardStats>({
    studyCount: 0,
    activeInvitationCount: 0,
  });
  const [isLoading, setIsLoading] = useState(true);
  const router = useRouter();
  const { toast } = useToast();
  const { studies, setActiveStudy } = useActiveStudy();

  const fetchStats = useCallback(async () => {
    try {
      const response = await fetch(getApiUrl('/api/studies'));
      if (!response.ok) throw new Error("Failed to fetch studies");
      const { data } = await response.json();
      setStats({
        studyCount: data.length,
        activeInvitationCount: 0
      });

      // If there's only one study and we're on the main dashboard, select it
      if (data.length === 1) {
        const study = {
          ...data[0],
          createdAt: new Date(data[0].createdAt),
          updatedAt: data[0].updatedAt ? new Date(data[0].updatedAt) : null
        };
        setActiveStudy(study);
        router.push(`/dashboard/studies/${study.id}`);
      }
    } catch (error) {
      console.error("Error fetching stats:", error);
      toast({
        title: "Error",
        description: "Failed to load dashboard statistics",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
    }
  }, [toast, router, setActiveStudy]);

  useEffect(() => {
    fetchStats();
  }, [fetchStats]);

  if (isLoading) {
    return (
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <div>
            <Skeleton className="h-8 w-[200px] mb-2" />
            <Skeleton className="h-4 w-[300px]" />
          </div>
          <Skeleton className="h-10 w-[140px]" />
        </div>

        <div className="grid gap-4 grid-cols-1 md:grid-cols-3">
          {[1, 2].map((i) => (
            <Card key={i}>
              <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <Skeleton className="h-4 w-[100px]" />
                <Skeleton className="h-4 w-4" />
              </CardHeader>
              <CardContent>
                <Skeleton className="h-7 w-[50px] mb-1" />
                <Skeleton className="h-3 w-[120px]" />
              </CardContent>
            </Card>
          ))}
        </div>

        <Card>
          <CardHeader>
            <Skeleton className="h-5 w-[120px] mb-2" />
            <Skeleton className="h-4 w-[200px]" />
          </CardHeader>
          <CardContent className="flex gap-4">
            <Skeleton className="h-10 w-[140px]" />
            <Skeleton className="h-10 w-[120px]" />
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold tracking-tight">Dashboard</h2>
          <p className="text-muted-foreground">
            Welcome back to your research dashboard
          </p>
        </div>
        <Button onClick={() => router.push('/dashboard/studies/new')}>
          Create New Study
        </Button>
      </div>

      <div className="grid gap-4 grid-cols-1 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              Total Studies
            </CardTitle>
            <BookOpen className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.studyCount}</div>
            <p className="text-xs text-muted-foreground">
              Active research studies
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              Pending Invitations
            </CardTitle>
            <Settings2 className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.activeInvitationCount}</div>
            <p className="text-xs text-muted-foreground">
              Awaiting responses
            </p>
          </CardContent>
        </Card>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Quick Actions</CardTitle>
          <CardDescription>Common tasks and actions</CardDescription>
        </CardHeader>
        <CardContent className="flex gap-4">
          <Button onClick={() => router.push('/dashboard/studies/new')}>
            Create New Study
          </Button>
          <Button
            variant="outline"
            onClick={() => router.push('/dashboard/settings')}
          >
            <Settings2 className="w-4 h-4 mr-2" />
            Settings
          </Button>
        </CardContent>
      </Card>
    </div>
  );
}
