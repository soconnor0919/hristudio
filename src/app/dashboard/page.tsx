'use client';

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Users, BookOpen, Settings2 } from "lucide-react";
import { useToast } from "~/hooks/use-toast";

interface DashboardStats {
  studyCount: number;
  participantCount: number;
  activeInvitationCount: number;
}

export default function Dashboard() {
  const [stats, setStats] = useState<DashboardStats>({
    studyCount: 0,
    participantCount: 0,
    activeInvitationCount: 0,
  });
  const [loading, setLoading] = useState(true);
  const router = useRouter();
  const { toast } = useToast();

  useEffect(() => {
    fetchDashboardStats();
  }, []);

  const fetchDashboardStats = async () => {
    try {
      const studiesRes = await fetch('/api/studies');
      const studies = await studiesRes.json();
      
      // For now, just show study count
      setStats({
        studyCount: studies.length,
        participantCount: 0,
        activeInvitationCount: 0,
      });
    } catch (error) {
      console.error('Error fetching dashboard stats:', error);
      toast({
        title: "Error",
        description: "Failed to load dashboard statistics",
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
    <div className="container py-6 space-y-6">
      <div>
        <h1 className="text-2xl font-bold">Dashboard</h1>
        <p className="text-muted-foreground">Overview of your research studies</p>
      </div>

      <div className="grid gap-4 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Studies</CardTitle>
            <BookOpen className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.studyCount ? stats.studyCount : 0}</div>
            <p className="text-xs text-muted-foreground">Active research studies</p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Participants</CardTitle>
            <Users className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.participantCount}</div>
            <p className="text-xs text-muted-foreground">Across all studies</p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Pending Invitations</CardTitle>
            <Settings2 className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{stats.activeInvitationCount}</div>
            <p className="text-xs text-muted-foreground">Awaiting acceptance</p>
          </CardContent>
        </Card>
      </div>

      <div className="grid gap-4 md:grid-cols-2">
        <Card>
          <CardHeader>
            <CardTitle>Quick Actions</CardTitle>
            <CardDescription>Common tasks and actions</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <Button
              variant="outline"
              className="w-full justify-start"
              onClick={() => router.push('/dashboard/studies')}
            >
              <BookOpen className="mr-2 h-4 w-4" />
              Manage Studies
            </Button>
            <Button
              variant="outline"
              className="w-full justify-start"
              onClick={() => router.push('/dashboard/participants')}
            >
              <Users className="mr-2 h-4 w-4" />
              Manage Participants
            </Button>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Getting Started</CardTitle>
            <CardDescription>Tips for using HRIStudio</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2">
            <p className="text-sm text-muted-foreground">
              • Create a new study from the Studies page
            </p>
            <p className="text-sm text-muted-foreground">
              • Invite collaborators using study settings
            </p>
            <p className="text-sm text-muted-foreground">
              • Add participants to begin collecting data
            </p>
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
