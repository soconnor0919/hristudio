"use client";

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { api } from "~/trpc/react";
import { format } from "date-fns";
import { Activity, User, UserPlus, Settings, FileEdit } from "lucide-react";

interface StudyActivityProps {
  studyId: number;
  role: string;
}

interface ActivityItem {
  id: number;
  type: "member_added" | "member_role_changed" | "study_updated" | "participant_added" | "participant_updated";
  description: string;
  userId: string;
  userName: string;
  createdAt: Date;
}

function getActivityIcon(type: ActivityItem["type"]) {
  switch (type) {
    case "member_added":
      return <UserPlus className="h-4 w-4" />;
    case "member_role_changed":
      return <Settings className="h-4 w-4" />;
    case "study_updated":
      return <FileEdit className="h-4 w-4" />;
    case "participant_added":
    case "participant_updated":
      return <User className="h-4 w-4" />;
    default:
      return <Activity className="h-4 w-4" />;
  }
}

export function StudyActivity({ studyId, role }: StudyActivityProps) {
  const { data: activities } = api.study.getActivities.useQuery({ studyId });

  return (
    <Card>
      <CardHeader>
        <CardTitle>Activity Log</CardTitle>
        <CardDescription>Recent activity in this study</CardDescription>
      </CardHeader>
      <CardContent>
        {!activities || activities.length === 0 ? (
          <div className="text-center py-6 text-muted-foreground">
            No activity recorded yet
          </div>
        ) : (
          <div className="space-y-8">
            {activities.map((activity) => (
              <div key={activity.id} className="flex gap-4">
                <div className="mt-1">
                  <div className="flex h-8 w-8 items-center justify-center rounded-full bg-muted">
                    {getActivityIcon(activity.type)}
                  </div>
                </div>
                <div className="flex-1 space-y-1">
                  <p className="text-sm font-medium leading-none">
                    {activity.userName}
                  </p>
                  <p className="text-sm text-muted-foreground">
                    {activity.description}
                  </p>
                  <p className="text-xs text-muted-foreground">
                    {format(new Date(activity.createdAt), "PPpp")}
                  </p>
                </div>
              </div>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  );
} 