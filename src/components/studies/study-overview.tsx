"use client";

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { api } from "~/trpc/react";
import { Users, Calendar, Clock } from "lucide-react";

interface StudyOverviewProps {
  study: {
    id: number;
    title: string;
    description: string | null;
    role: string;
  };
}

export function StudyOverview({ study }: StudyOverviewProps) {
  const { data: participantCount } = api.participant.getCount.useQuery({ studyId: study.id });

  return (
    <div className="grid gap-4">
      {/* Basic Information */}
      <Card>
        <CardHeader>
          <CardTitle>Study Details</CardTitle>
          <CardDescription>Basic information about the study</CardDescription>
        </CardHeader>
        <CardContent>
          <dl className="grid gap-4 sm:grid-cols-2">
            <div>
              <dt className="text-sm font-medium text-muted-foreground">Your Role</dt>
              <dd className="mt-1 text-sm">{study.role}</dd>
            </div>
            <div>
              <dt className="text-sm font-medium text-muted-foreground">Description</dt>
              <dd className="mt-1 text-sm">{study.description || "No description provided"}</dd>
            </div>
          </dl>
        </CardContent>
      </Card>

      {/* Quick Stats */}
      <div className="grid gap-4 md:grid-cols-3">
        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Total Participants</CardTitle>
            <Users className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">{participantCount ?? 0}</div>
            <p className="text-xs text-muted-foreground">
              Active participants in study
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Last Activity</CardTitle>
            <Clock className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">—</div>
            <p className="text-xs text-muted-foreground">
              Most recent update
            </p>
          </CardContent>
        </Card>

        <Card>
          <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
            <CardTitle className="text-sm font-medium">Study Duration</CardTitle>
            <Calendar className="h-4 w-4 text-muted-foreground" />
          </CardHeader>
          <CardContent>
            <div className="text-2xl font-bold">—</div>
            <p className="text-xs text-muted-foreground">
              Days since creation
            </p>
          </CardContent>
        </Card>
      </div>
    </div>
  );
} 