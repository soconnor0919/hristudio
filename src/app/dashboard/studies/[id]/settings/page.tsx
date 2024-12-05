'use client';

import { useState } from "react";
import { useParams, useRouter } from "next/navigation";
import { SettingsTab } from "~/components/studies/settings-tab";
import { ParticipantsTab } from "~/components/studies/participants-tab";
import { UsersTab } from "~/components/studies/users-tab";
import { useEffect } from "react";
import { PERMISSIONS } from "~/lib/permissions-client";
import { Button } from "~/components/ui/button";
import { Settings2Icon, UsersIcon, UserIcon } from "lucide-react";
import { cn } from "~/lib/utils";
import { getApiUrl } from "~/lib/fetch-utils";

interface Study {
  id: number;
  title: string;
  description: string | null;
  permissions: string[];
}

export default function StudySettings() {
  const [study, setStudy] = useState<Study | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [activeTab, setActiveTab] = useState<'settings' | 'participants' | 'users'>('settings');
  const { id } = useParams();
  const router = useRouter();

  useEffect(() => {
    const fetchStudy = async () => {
      try {
        const response = await fetch(getApiUrl(`/api/studies/${id}`));
        if (!response.ok) {
          if (response.status === 403) {
            router.push('/dashboard/studies');
            return;
          }
          throw new Error("Failed to fetch study");
        }
        const data = await response.json();
        
        // Check if user has any required permissions
        const requiredPermissions = [PERMISSIONS.EDIT_STUDY, PERMISSIONS.MANAGE_ROLES];
        const hasAccess = data.data.permissions.some(p => requiredPermissions.includes(p));

        if (!hasAccess) {
          router.push('/dashboard/studies');
          return;
        }

        setStudy(data.data);
      } catch (error) {
        console.error("Error fetching study:", error);
        setError(error instanceof Error ? error.message : "Failed to load study");
      } finally {
        setIsLoading(false);
      }
    };

    fetchStudy();
  }, [id, router]);

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (error || !study) {
    return <div>Error: {error}</div>;
  }

  return (
    <div className="container py-6">
      <div className="flex flex-col gap-2 mb-6">
        <h1 className="text-3xl font-bold tracking-tight">{study.title}</h1>
        <p className="text-muted-foreground">
          Manage study settings, participants, and team members
        </p>
      </div>

      <div className="flex gap-6">
        <div className="w-48 flex flex-col gap-2">
          <Button
            variant={activeTab === 'settings' ? 'secondary' : 'ghost'}
            className="justify-start"
            onClick={() => setActiveTab('settings')}
          >
            <Settings2Icon className="mr-2 h-4 w-4" />
            Settings
          </Button>
          <Button
            variant={activeTab === 'participants' ? 'secondary' : 'ghost'}
            className="justify-start"
            onClick={() => setActiveTab('participants')}
          >
            <UserIcon className="mr-2 h-4 w-4" />
            Participants
          </Button>
          <Button
            variant={activeTab === 'users' ? 'secondary' : 'ghost'}
            className="justify-start"
            onClick={() => setActiveTab('users')}
          >
            <UsersIcon className="mr-2 h-4 w-4" />
            Users
          </Button>
        </div>

        <div className="flex-1">
          <div className={cn(activeTab === 'settings' ? 'block' : 'hidden')}>
            <SettingsTab study={study} />
          </div>
          <div className={cn(activeTab === 'participants' ? 'block' : 'hidden')}>
            <ParticipantsTab studyId={study.id} permissions={study.permissions} />
          </div>
          <div className={cn(activeTab === 'users' ? 'block' : 'hidden')}>
            <UsersTab studyId={study.id} permissions={study.permissions} />
          </div>
        </div>
      </div>
    </div>
  );
} 