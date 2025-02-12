"use client";

import { useRouter, useSearchParams } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Button } from "~/components/ui/button";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";
import { Pencil as PencilIcon } from "lucide-react";
import { use } from "react";
import { StudyOverview } from "~/components/studies/study-overview";
import { StudyParticipants } from "~/components/studies/study-participants";
import { StudyMembers } from "~/components/studies/study-members";
import { StudyMetadata } from "~/components/studies/study-metadata";
import { StudyActivity } from "~/components/studies/study-activity";
import { StudyDetailsSkeleton } from "~/components/ui/skeleton";

export default function StudyPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const searchParams = useSearchParams();
  const resolvedParams = use(params);
  const id = Number(resolvedParams.id);
  const activeTab = searchParams.get("tab") ?? "overview";

  const { data: study, isLoading: isLoadingStudy } = api.study.getById.useQuery({ id });

  if (isLoadingStudy) {
    return (
      <>
        <PageHeader
          title="Loading..."
          description="Please wait while we load the study details"
        />
        <PageContent>
          <Tabs defaultValue="overview" className="space-y-4">
            <TabsList>
              <TabsTrigger value="overview">Overview</TabsTrigger>
              <TabsTrigger value="participants">Participants</TabsTrigger>
              <TabsTrigger value="members">Members</TabsTrigger>
              <TabsTrigger value="metadata">Metadata</TabsTrigger>
              <TabsTrigger value="activity">Activity</TabsTrigger>
            </TabsList>
            <TabsContent value="overview">
              <StudyDetailsSkeleton />
            </TabsContent>
          </Tabs>
        </PageContent>
      </>
    );
  }

  if (!study) {
    return <div>Study not found</div>;
  }

  const canEdit = study.role === "admin";

  return (
    <>
      <PageHeader
        title={study.title}
        description={study.description ?? "No description provided"}
      >
        {canEdit && (
          <Button
            variant="outline"
            size="sm"
            onClick={() => router.push(`/dashboard/studies/${id}/edit`)}
          >
            <PencilIcon className="h-4 w-4 mr-2" />
            Edit Study
          </Button>
        )}
      </PageHeader>
      <PageContent>
        <Tabs defaultValue={activeTab} className="space-y-4">
          <TabsList>
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="participants">Participants</TabsTrigger>
            <TabsTrigger value="members">Members</TabsTrigger>
            <TabsTrigger value="metadata">Metadata</TabsTrigger>
            <TabsTrigger value="activity">Activity</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-4">
            <StudyOverview study={study} />
          </TabsContent>

          <TabsContent value="participants" className="space-y-4">
            <StudyParticipants studyId={id} role={study.role} />
          </TabsContent>

          <TabsContent value="members" className="space-y-4">
            <StudyMembers studyId={id} role={study.role} />
          </TabsContent>

          <TabsContent value="metadata" className="space-y-4">
            <StudyMetadata studyId={id} role={study.role} />
          </TabsContent>

          <TabsContent value="activity" className="space-y-4">
            <StudyActivity studyId={id} role={study.role} />
          </TabsContent>
        </Tabs>
      </PageContent>
    </>
  );
} 