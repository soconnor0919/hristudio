"use client";

import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { PageHeader } from "~/components/layout/page-header";
import { PageContent } from "~/components/layout/page-content";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Plus as PlusIcon } from "lucide-react";

export default function StudiesPage() {
  const router = useRouter();
  const { data: studies, isLoading } = api.study.getMyStudies.useQuery();

  if (isLoading) {
    return <div>Loading...</div>;
  }

  return (
    <>
      <PageHeader
        title="Studies"
        description="Manage your research studies"
      >
        <Button
          onClick={() => router.push("/dashboard/studies/new")}
          size="sm"
        >
          <PlusIcon className="h-4 w-4 mr-2" />
          New Study
        </Button>
      </PageHeader>
      <PageContent>
        <div className="grid gap-6">
          {!studies || studies.length === 0 ? (
            <Card>
              <CardHeader>
                <CardTitle>No Studies</CardTitle>
                <CardDescription>
                  You haven't created any studies yet. Click the button above to create your first study.
                </CardDescription>
              </CardHeader>
            </Card>
          ) : (
            studies.map((study) => (
              <Card
                key={study.id}
                className="hover:bg-muted/50 cursor-pointer transition-colors"
                onClick={() => router.push(`/dashboard/studies/${study.id}`)}
              >
                <CardHeader>
                  <CardTitle>{study.title}</CardTitle>
                  <CardDescription>
                    {study.description || "No description provided"}
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="text-sm text-muted-foreground">
                    Your role: {study.role}
                  </div>
                </CardContent>
              </Card>
            ))
          )}
        </div>
      </PageContent>
    </>
  );
} 