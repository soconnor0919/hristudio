"use client";

import { useEffect } from "react";
import { useRouter } from "next/navigation";
import Link from "next/link";
import { TestTube, ArrowRight } from "lucide-react";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { useStudyContext } from "~/lib/study-context";

export default function TrialsRedirect() {
  const router = useRouter();
  const { selectedStudyId } = useStudyContext();

  useEffect(() => {
    // If user has a selected study, redirect to study trials
    if (selectedStudyId) {
      router.replace(`/studies/${selectedStudyId}/trials`);
    }
  }, [selectedStudyId, router]);

  return (
    <div className="flex min-h-[60vh] items-center justify-center p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-full bg-orange-50">
            <TestTube className="h-8 w-8 text-orange-500" />
          </div>
          <CardTitle className="text-2xl">Trials Moved</CardTitle>
          <CardDescription>
            Trial management is now organized by study for better workflow
            organization.
          </CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="text-muted-foreground space-y-2 text-center text-sm">
            <p>To manage trials:</p>
            <ul className="space-y-1 text-left">
              <li>• Select a study from your studies list</li>
              <li>• Navigate to that study&apos;s trials page</li>
              <li>• Schedule and monitor trials for that specific study</li>
            </ul>
          </div>
          <div className="flex flex-col gap-2 pt-4">
            <Button asChild className="w-full">
              <Link href="/studies">
                <ArrowRight className="mr-2 h-4 w-4" />
                Browse Studies
              </Link>
            </Button>
            <Button asChild variant="outline" className="w-full">
              <Link href="/dashboard">Go to Dashboard</Link>
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}
