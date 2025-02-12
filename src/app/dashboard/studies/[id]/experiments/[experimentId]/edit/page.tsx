"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { use } from "react";
import { ExperimentDesigner } from "~/components/experiments/experiment-designer";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { useToast } from "~/hooks/use-toast";
import { useState, useEffect } from "react";
import { type Step } from "~/lib/experiments/types";

export default function EditExperimentPage({
  params,
}: {
  params: Promise<{ id: string; experimentId: string }>;
}) {
  const router = useRouter();
  const { toast } = useToast();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);
  const experimentId = Number(resolvedParams.experimentId);

  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [steps, setSteps] = useState<Step[]>([]);

  const { data: study } = api.study.getById.useQuery({ id: studyId });
  const { data: experiment, isLoading } = api.experiment.getById.useQuery({ id: experimentId });

  useEffect(() => {
    if (experiment) {
      setTitle(experiment.title);
      setDescription(experiment.description ?? "");
      setSteps(experiment.steps);
    }
  }, [experiment]);

  const { mutate: updateExperiment, isPending: isUpdating } = api.experiment.update.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Experiment updated successfully",
      });
      router.push(`/dashboard/studies/${studyId}/experiments/${experimentId}`);
      router.refresh();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
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

  if (!canEdit) {
    return <div>You do not have permission to edit this experiment.</div>;
  }

  return (
    <>
      <PageHeader
        title="Edit Experiment"
        description={`Update experiment details for ${study.title}`}
      />
      <PageContent>
        <div className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle>Experiment Details</CardTitle>
              <CardDescription>
                Update the basic information for your experiment.
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="space-y-2">
                <label htmlFor="title" className="text-sm font-medium">
                  Title
                </label>
                <Input
                  id="title"
                  placeholder="Enter experiment title"
                  value={title}
                  onChange={(e) => setTitle(e.target.value)}
                />
              </div>
              <div className="space-y-2">
                <label htmlFor="description" className="text-sm font-medium">
                  Description
                </label>
                <Textarea
                  id="description"
                  placeholder="Enter experiment description"
                  value={description}
                  onChange={(e) => setDescription(e.target.value)}
                />
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader>
              <CardTitle>Design Experiment</CardTitle>
              <CardDescription>
                Use the designer below to update your experiment flow.
              </CardDescription>
            </CardHeader>
            <CardContent className="p-0">
              <ExperimentDesigner
                defaultSteps={steps}
                onChange={setSteps}
              />
            </CardContent>
          </Card>

          <div className="flex justify-end gap-4">
            <Button
              variant="outline"
              onClick={() => router.push(`/dashboard/studies/${studyId}/experiments/${experimentId}`)}
            >
              Cancel
            </Button>
            <Button
              onClick={() => {
                updateExperiment({
                  id: experimentId,
                  title,
                  description,
                  steps,
                });
              }}
              disabled={isUpdating || !title}
            >
              {isUpdating ? "Saving..." : "Save Changes"}
            </Button>
          </div>
        </div>
      </PageContent>
    </>
  );
} 