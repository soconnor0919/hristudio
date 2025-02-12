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
import { useState } from "react";
import { type Step } from "~/lib/experiments/types";

export default function NewExperimentPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const { toast } = useToast();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);

  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [steps, setSteps] = useState<Step[]>([]);

  const { data: study } = api.study.getById.useQuery({ id: studyId });

  const { mutate: createExperiment, isPending: isCreating } = api.experiment.create.useMutation({
    onSuccess: (data) => {
      toast({
        title: "Success",
        description: "Experiment created successfully",
      });
      router.push(`/dashboard/studies/${studyId}/experiments/${data.id}`);
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

  const canCreateExperiments = study && ["OWNER", "ADMIN", "PRINCIPAL_INVESTIGATOR"]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  if (!study) {
    return <div>Study not found</div>;
  }

  if (!canCreateExperiments) {
    return <div>You do not have permission to create experiments in this study.</div>;
  }

  return (
    <>
      <PageHeader
        title="Create Experiment"
        description={`Design a new experiment for ${study.title}`}
      />
      <PageContent>
        <div className="space-y-6">
          <Card>
            <CardHeader>
              <CardTitle>Experiment Details</CardTitle>
              <CardDescription>
                Enter the basic information for your experiment.
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
                Use the designer below to create your experiment flow.
              </CardDescription>
            </CardHeader>
            <CardContent className="p-0">
              <ExperimentDesigner
                onChange={setSteps}
              />
            </CardContent>
          </Card>

          <div className="flex justify-end gap-4">
            <Button
              variant="outline"
              onClick={() => router.push(`/dashboard/studies/${studyId}/experiments`)}
            >
              Cancel
            </Button>
            <Button
              onClick={() => {
                createExperiment({
                  studyId,
                  title,
                  description,
                  steps,
                });
              }}
              disabled={isCreating || !title}
            >
              {isCreating ? "Creating..." : "Create Experiment"}
            </Button>
          </div>
        </div>
      </PageContent>
    </>
  );
} 