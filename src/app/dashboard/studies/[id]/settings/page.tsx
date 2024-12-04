'use client';

import { useState } from "react";
import { useParams, useRouter, useSearchParams } from "next/navigation";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { SettingsTab } from "~/components/studies/settings-tab";
import { ParticipantsTab } from "~/components/studies/participants-tab";
import { useEffect } from "react";

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
  const { id } = useParams();
  const router = useRouter();
  const searchParams = useSearchParams();
  const tab = searchParams.get('tab') || 'settings';

  useEffect(() => {
    const fetchStudy = async () => {
      try {
        const response = await fetch(`/api/studies/${id}`);
        if (!response.ok) throw new Error("Failed to fetch study");
        const data = await response.json();
        setStudy(data.data);
      } catch (error) {
        console.error("Error fetching study:", error);
        setError(error instanceof Error ? error.message : "Failed to load study");
      } finally {
        setIsLoading(false);
      }
    };

    fetchStudy();
  }, [id]);

  const handleTabChange = (value: string) => {
    router.push(`/dashboard/studies/${id}/settings?tab=${value}`);
  };

  if (isLoading) {
    return <div className="container py-6">Loading...</div>;
  }

  if (error || !study) {
    return <div className="container py-6 text-destructive">{error || "Study not found"}</div>;
  }

  return (
    <div className="container py-6 space-y-6">
      <div className="flex flex-col gap-2">
        <h1 className="text-3xl font-bold tracking-tight">{study.title}</h1>
        <p className="text-muted-foreground">
          Manage study settings and participants
        </p>
      </div>

      <Tabs value={tab} onValueChange={handleTabChange} className="space-y-6">
        <TabsList>
          <TabsTrigger value="settings">Settings</TabsTrigger>
          <TabsTrigger value="participants">Participants</TabsTrigger>
        </TabsList>

        <TabsContent value="settings" className="space-y-6">
          <SettingsTab study={study} />
        </TabsContent>

        <TabsContent value="participants" className="space-y-6">
          <ParticipantsTab studyId={study.id} permissions={study.permissions} />
        </TabsContent>
      </Tabs>
    </div>
  );
} 