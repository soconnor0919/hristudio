"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Pencil as PencilIcon, Play, Archive } from "lucide-react";
import { use } from "react";
import { ExperimentDesigner } from "~/components/experiments/experiment-designer";

export default function ExperimentDetailsPage({
  params,
}: {
  params: Promise<{ id: string; experimentId: string }>;
}) {
  const router = useRouter();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);
  const experimentId = Number(resolvedParams.experimentId);

  const { data: study } = api.study.getById.useQuery({ id: studyId });
  const { data: experiment, isLoading } = api.experiment.getById.useQuery({ id: experimentId });

  const { mutate: updateExperiment } = api.experiment.update.useMutation({
    onSuccess: () => {
      router.refresh();
    },
  });

  const canEdit = study && ["OWNER", "ADMIN", "PRINCIPAL_INVESTIGATOR"]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  if (isLoading) {
    return (
      <>
        <PageHeader
          title="Loading..."
          description="Please wait while we load the experiment details"
        />
        <PageContent>
          <div className="space-y-6">
            <Card className="animate-pulse">
              <CardHeader>
                <div className="h-6 w-1/3 bg-muted rounded" />
                <div className="h-4 w-1/2 bg-muted rounded mt-2" />
              </CardHeader>
              <CardContent>
                <div className="h-4 w-1/4 bg-muted rounded" />
              </CardContent>
            </Card>
          </div>
        </PageContent>
      </>
    );
  }

  if (!study || !experiment) {
    return <div>Not found</div>;
  }

  return (
    <>
      <PageHeader
        title={experiment.title}
        description={experiment.description ?? "No description provided"}
      >
        <div className="flex items-center gap-2">
          <Badge variant={
            experiment.status === "active" ? "default" :
            experiment.status === "archived" ? "secondary" :
            "outline"
          }>
            {experiment.status}
          </Badge>
          {canEdit && (
            <>
              <Button
                variant="outline"
                size="sm"
                onClick={() => router.push(`/dashboard/studies/${studyId}/experiments/${experimentId}/edit`)}
              >
                <PencilIcon className="h-4 w-4 mr-2" />
                Edit
              </Button>
              {experiment.status === "draft" ? (
                <Button
                  size="sm"
                  onClick={() => updateExperiment({
                    id: experimentId,
                    title: experiment.title,
                    description: experiment.description,
                    status: "active",
                  })}
                >
                  <Play className="h-4 w-4 mr-2" />
                  Activate
                </Button>
              ) : experiment.status === "active" ? (
                <Button
                  variant="secondary"
                  size="sm"
                  onClick={() => updateExperiment({
                    id: experimentId,
                    title: experiment.title,
                    description: experiment.description,
                    status: "archived",
                  })}
                >
                  <Archive className="h-4 w-4 mr-2" />
                  Archive
                </Button>
              ) : null}
            </>
          )}
        </div>
      </PageHeader>
      <PageContent>
        <div className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle>Experiment Flow</CardTitle>
              <CardDescription>
                View the steps and actions in this experiment.
              </CardDescription>
            </CardHeader>
            <CardContent className="p-0">
              <ExperimentDesigner
                defaultSteps={experiment.steps}
                onChange={canEdit ? (steps) => {
                  updateExperiment({
                    id: experimentId,
                    title: experiment.title,
                    description: experiment.description,
                    steps,
                  });
                } : undefined}
                readOnly={!canEdit}
              />
            </CardContent>
          </Card>
        </div>
      </PageContent>
    </>
  );
} 