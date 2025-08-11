"use client";

import { AlertTriangle, Building, Loader2 } from "lucide-react";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { useStudyContext } from "~/lib/study-context";

interface StudyGuardProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
}

export function StudyGuard({ children, fallback }: StudyGuardProps) {
  const { selectedStudyId, isLoading } = useStudyContext();

  if (isLoading) {
    return <LoadingMessage />;
  }

  if (!selectedStudyId) {
    return fallback ?? <DefaultStudyRequiredMessage />;
  }

  return <>{children}</>;
}

function LoadingMessage() {
  return (
    <div className="flex min-h-[60vh] items-center justify-center">
      <Card className="mx-auto w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mb-4 flex justify-center">
            <div className="rounded-full bg-blue-100 p-3">
              <Loader2 className="h-6 w-6 animate-spin text-blue-600" />
            </div>
          </div>
          <CardTitle>Loading...</CardTitle>
          <CardDescription>Checking your study selection</CardDescription>
        </CardHeader>
      </Card>
    </div>
  );
}

function DefaultStudyRequiredMessage() {
  return (
    <div className="flex min-h-[60vh] items-center justify-center">
      <Card className="mx-auto w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mb-4 flex justify-center">
            <div className="rounded-full bg-amber-100 p-3">
              <AlertTriangle className="h-6 w-6 text-amber-600" />
            </div>
          </div>
          <CardTitle>Study Required</CardTitle>
          <CardDescription>
            You need to select an active study to access this section
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <p className="text-muted-foreground text-center text-sm">
            Use the study selector in the sidebar to choose an active study, or
            create a new study to get started.
          </p>
          <div className="flex flex-col gap-2">
            <Button asChild>
              <Link href="/studies">
                <Building className="mr-2 h-4 w-4" />
                Browse Studies
              </Link>
            </Button>
            <Button variant="outline" asChild>
              <Link href="/studies/new">Create New Study</Link>
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
