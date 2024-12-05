'use client';

import { useCallback, useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useToast } from "~/hooks/use-toast";
import { Plus, Users, FileText, BarChart, PlayCircle } from "lucide-react";
import Link from "next/link";
import { useActiveStudy } from "~/context/active-study";
import { getApiUrl } from "~/lib/fetch-utils";
import { Skeleton } from "~/components/ui/skeleton";

interface StudyStats {
  participantCount: number;
  formCount: number;
  trialCount: number;
}

export default function StudyDashboard() {
  const [stats, setStats] = useState<StudyStats>({
    participantCount: 0,
    formCount: 0,
    trialCount: 0,
  });
  const [isLoading, setIsLoading] = useState(true);
  const { id } = useParams();
  const { toast } = useToast();
  const { activeStudy } = useActiveStudy();

  const fetchStats = useCallback(async () => {
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}/stats`));
      if (!response.ok) throw new Error("Failed to fetch stats");
      const { data } = await response.json();
      setStats({
        participantCount: data?.participantCount ?? 0,
        formCount: data?.formCount ?? 0,
        trialCount: data?.trialCount ?? 0
      });
    } catch (error) {
      console.error("Error fetching stats:", error);
      toast({
        title: "Error",
        description: "Failed to load study statistics",
        variant: "destructive",
      });
      // Set default values on error
      setStats({
        participantCount: 0,
        formCount: 0,
        trialCount: 0
      });
    } finally {
      setIsLoading(false);
    }
  }, [toast, id]);

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
        </div>

        <div className="grid gap-4 grid-cols-1 md:grid-cols-3">
          {[1, 2, 3].map((i) => (
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
          <h2 className="text-2xl font-bold tracking-tight">{activeStudy?.title}</h2>
          <p className="text-muted-foreground">
            Overview of your study&apos;s progress and statistics
          </p>
        </div>
      </div>

      <div className="grid gap-4 grid-cols-1 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              Participants
            </CardTitle>
            <Users className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.participantCount}</div>
            <p className="text-xs text-muted-foreground">
              Total enrolled participants
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              Forms
            </CardTitle>
            <FileText className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.formCount}</div>
            <p className="text-xs text-muted-foreground">
              Active study forms
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">
              Trials
            </CardTitle>
            <BarChart className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.trialCount}</div>
            <p className="text-xs text-muted-foreground">
              Completed trials
            </p>
          </CardContent>
        </Card>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Quick Actions</CardTitle>
          <CardDescription>Common tasks and actions for this study</CardDescription>
        </CardHeader>
        <CardContent className="flex gap-4">
          <Button asChild>
            <Link href={`/dashboard/studies/${id}/participants/new`}>
              <Plus className="w-4 h-4 mr-2" />
              Add Participant
            </Link>
          </Button>
          <Button variant="outline" asChild>
            <Link href={`/dashboard/studies/${id}/trials/new`}>
              <PlayCircle className="w-4 h-4 mr-2" />
              Start Trial
            </Link>
          </Button>
        </CardContent>
      </Card>
    </div>
  );
} 