"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Plus as PlusIcon, FlaskConical } from "lucide-react";
import { Badge } from "~/components/ui/badge";
import { use } from "react";

export default function ExperimentsPage({ params }: { params: Promise<{ id: string }> }) {
  const router = useRouter();
  const resolvedParams = use(params);
  const studyId = Number(resolvedParams.id);

  const { data: study } = api.study.getById.useQuery({ id: studyId });
  const { data: experiments, isLoading } = api.experiment.getByStudyId.useQuery({ studyId });

  const canCreateExperiments = study && ["OWNER", "ADMIN", "PRINCIPAL_INVESTIGATOR"]
    .map(r => r.toLowerCase())
    .includes(study.role.toLowerCase());

  return (
    <>
      <PageHeader
        title="Experiments"
        description={study ? `Manage experiments for ${study.title}` : "Loading..."}
      >
        {canCreateExperiments && (
          <Button
            onClick={() => router.push(`/dashboard/studies/${studyId}/experiments/new`)}
            size="sm"
          >
            <PlusIcon className="h-4 w-4 mr-2" />
            New Experiment
          </Button>
        )}
      </PageHeader>
      <PageContent>
        {isLoading ? (
          <div className="grid gap-6">
            {[...Array(3)].map((_, i) => (
              <Card key={i} className="animate-pulse">
                <CardHeader>
                  <div className="h-6 w-1/3 bg-muted rounded" />
                  <div className="h-4 w-1/2 bg-muted rounded mt-2" />
                </CardHeader>
                <CardContent>
                  <div className="h-4 w-1/4 bg-muted rounded" />
                </CardContent>
              </Card>
            ))}
          </div>
        ) : !experiments || experiments.length === 0 ? (
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <FlaskConical className="h-5 w-5" />
                No Experiments
              </CardTitle>
              <CardDescription>
                {canCreateExperiments
                  ? "Get started by creating your first experiment."
                  : "No experiments have been created for this study yet."}
              </CardDescription>
            </CardHeader>
          </Card>
        ) : (
          <div className="grid gap-6">
            {experiments.map((experiment) => (
              <Card
                key={experiment.id}
                className="hover:bg-muted/50 cursor-pointer transition-colors"
                onClick={() => router.push(`/dashboard/studies/${studyId}/experiments/${experiment.id}`)}
              >
                <CardHeader>
                  <div className="flex items-center justify-between">
                    <CardTitle>{experiment.title}</CardTitle>
                    <Badge variant={
                      experiment.status === "active" ? "default" :
                      experiment.status === "archived" ? "secondary" :
                      "outline"
                    }>
                      {experiment.status}
                    </Badge>
                  </div>
                  <CardDescription>
                    {experiment.description || "No description provided"}
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="text-sm text-muted-foreground">
                    Version {experiment.version} • {experiment.steps.length} steps
                  </div>
                </CardContent>
              </Card>
            ))}
          </div>
        )}
      </PageContent>
    </>
  );
} 