'use client';

import { useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { 
  Users, 
  FileText, 
  BarChart, 
  PlayCircle,
  Plus,
  Settings2
} from "lucide-react";
import { useToast } from "~/hooks/use-toast";
import Link from "next/link";

interface StudyStats {
  participantCount: number;
  completedTrialsCount: number;
  pendingFormsCount: number;
}

export default function StudyDashboard() {
  const [stats, setStats] = useState<StudyStats>({
    participantCount: 0,
    completedTrialsCount: 0,
    pendingFormsCount: 0,
  });
  const [loading, setLoading] = useState(true);
  const { id } = useParams();
  const { toast } = useToast();

  useEffect(() => {
    fetchStudyStats();
  }, [id]);

  const fetchStudyStats = async () => {
    try {
      const response = await fetch(`/api/studies/${id}/stats`);
      if (!response.ok) throw new Error("Failed to fetch study statistics");
      const data = await response.json();
      setStats(data.data);
    } catch (error) {
      console.error("Error fetching study stats:", error);
      toast({
        title: "Error",
        description: "Failed to load study statistics",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="animate-spin h-8 w-8 border-4 border-primary border-t-transparent rounded-full" />
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="grid gap-4 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Participants</CardTitle>
            <Users className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.participantCount}</div>
            <p className="text-xs text-muted-foreground">Total participants enrolled</p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Completed Trials</CardTitle>
            <PlayCircle className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.completedTrialsCount}</div>
            <p className="text-xs text-muted-foreground">Successfully completed trials</p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Pending Forms</CardTitle>
            <FileText className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.pendingFormsCount}</div>
            <p className="text-xs text-muted-foreground">Forms awaiting completion</p>
          </CardContent>
        </Card>
      </div>

      <div className="grid gap-4 md:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>Quick Actions</CardTitle>
            <CardDescription>Common tasks for this study</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <Button
              variant="outline"
              className="w-full justify-start"
              asChild
            >
              <Link href={`/dashboard/studies/${id}/participants/new`}>
                <Plus className="mr-2 h-4 w-4" />
                Add Participant
              </Link>
            </Button>
            <Button
              variant="outline"
              className="w-full justify-start"
              asChild
            >
              <Link href={`/dashboard/studies/${id}/trials/new`}>
                <PlayCircle className="mr-2 h-4 w-4" />
                Start New Trial
              </Link>
            </Button>
            <Button
              variant="outline"
              className="w-full justify-start"
              asChild
            >
              <Link href={`/dashboard/studies/${id}/forms/new`}>
                <FileText className="mr-2 h-4 w-4" />
                Create Form
              </Link>
            </Button>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Recent Activity</CardTitle>
            <CardDescription>Latest updates and changes</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <Button
              variant="outline"
              className="w-full justify-start"
              asChild
            >
              <Link href={`/dashboard/studies/${id}/analysis`}>
                <BarChart className="mr-2 h-4 w-4" />
                View Analytics
              </Link>
            </Button>
            <Button
              variant="outline"
              className="w-full justify-start"
              asChild
            >
              <Link href={`/dashboard/studies/${id}/settings`}>
                <Settings2 className="mr-2 h-4 w-4" />
                Study Settings
              </Link>
            </Button>
          </CardContent>
        </Card>
      </div>
    </div>
  );
} 