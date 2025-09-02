import { formatDistanceToNow } from "date-fns";
import {
  AlertTriangle,
  ArrowLeft,
  CheckCircle2,
  Clock,
  FlaskConical,
  Play,
  TestTube,
  User,
} from "lucide-react";
import Link from "next/link";
import { notFound, redirect } from "next/navigation";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { auth } from "~/server/auth";
import { api } from "~/trpc/server";

interface StartPageProps {
  params: Promise<{
    trialId: string;
  }>;
}

export default async function StartTrialPage({ params }: StartPageProps) {
  const session = await auth();
  if (!session) {
    redirect("/auth/signin");
  }

  const role = session.user.roles?.[0]?.role ?? "observer";
  if (!["wizard", "researcher", "administrator"].includes(role)) {
    redirect("/trials?error=insufficient_permissions");
  }

  const { trialId } = await params;

  let trial: Awaited<ReturnType<typeof api.trials.get>>;
  try {
    trial = await api.trials.get({ id: trialId });
  } catch {
    notFound();
  }

  // Guard: Only allow start from scheduled; if in progress, go to wizard; if completed, go to analysis
  if (trial.status === "in_progress") {
    redirect(`/trials/${trialId}/wizard`);
  }
  if (trial.status === "completed") {
    redirect(`/trials/${trialId}/analysis`);
  }
  if (!["scheduled"].includes(trial.status)) {
    redirect(`/trials/${trialId}?error=trial_not_startable`);
  }

  // Server Action: start trial and redirect to wizard
  async function startTrial() {
    "use server";
    // Confirm auth on action too
    const s = await auth();
    if (!s) redirect("/auth/signin");
    const r = s.user.roles?.[0]?.role ?? "observer";
    if (!["wizard", "researcher", "administrator"].includes(r)) {
      redirect(`/trials/${trialId}?error=insufficient_permissions`);
    }
    await api.trials.start({ id: trialId });
    redirect(`/trials/${trialId}/wizard`);
  }

  const scheduled =
    trial.scheduledAt instanceof Date
      ? trial.scheduledAt
      : trial.scheduledAt
        ? new Date(trial.scheduledAt)
        : null;

  const hasWizardAssigned = Boolean(trial.wizardId);

  return (
    <div className="min-h-screen bg-slate-50">
      {/* Header */}
      <div className="border-b border-slate-200 bg-white px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Button asChild variant="ghost" size="sm">
              <Link href={`/trials/${trial.id}`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trial
              </Link>
            </Button>
            <Separator orientation="vertical" className="h-6" />
            <div>
              <h1 className="text-2xl font-bold text-slate-900">Start Trial</h1>
              <p className="mt-1 text-sm text-slate-600">
                {trial.experiment.name} • Participant:{" "}
                {trial.participant.participantCode}
              </p>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant="outline" className="bg-blue-50 text-blue-700">
              Scheduled
            </Badge>
          </div>
        </div>
      </div>

      {/* Content */}
      <div className="mx-auto max-w-5xl space-y-6 p-6">
        {/* Summary */}
        <div className="grid grid-cols-1 gap-4 md:grid-cols-3">
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-sm font-medium text-slate-700">
                Experiment
              </CardTitle>
            </CardHeader>
            <CardContent className="flex items-center gap-2">
              <FlaskConical className="h-4 w-4 text-slate-600" />
              <div className="text-sm font-semibold text-slate-900">
                {trial.experiment.name}
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-sm font-medium text-slate-700">
                Participant
              </CardTitle>
            </CardHeader>
            <CardContent className="flex items-center gap-2">
              <User className="h-4 w-4 text-slate-600" />
              <div className="text-sm font-semibold text-slate-900">
                {trial.participant.participantCode}
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-sm font-medium text-slate-700">
                Scheduled
              </CardTitle>
            </CardHeader>
            <CardContent className="flex items-center gap-2">
              <Clock className="h-4 w-4 text-slate-600" />
              <div className="text-sm font-semibold text-slate-900">
                {scheduled
                  ? `${formatDistanceToNow(scheduled, { addSuffix: true })}`
                  : "Not set"}
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Preflight Checks */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2 text-base">
              <TestTube className="h-4 w-4 text-slate-700" />
              Preflight Checklist
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-3">
            <div className="flex items-start gap-3 rounded-md border border-slate-200 bg-white p-3">
              <CheckCircle2 className="mt-0.5 h-4 w-4 text-green-600" />
              <div className="text-sm">
                <div className="font-medium text-slate-900">Permissions</div>
                <div className="text-slate-600">
                  You have sufficient permissions to start this trial.
                </div>
              </div>
            </div>

            <div className="flex items-start gap-3 rounded-md border border-slate-200 bg-white p-3">
              {hasWizardAssigned ? (
                <CheckCircle2 className="mt-0.5 h-4 w-4 text-green-600" />
              ) : (
                <AlertTriangle className="mt-0.5 h-4 w-4 text-amber-600" />
              )}
              <div className="text-sm">
                <div className="font-medium text-slate-900">Wizard</div>
                <div className="text-slate-600">
                  {hasWizardAssigned
                    ? "A wizard has been assigned to this trial."
                    : "No wizard assigned. You can still start, but consider assigning a wizard for clarity."}
                </div>
              </div>
            </div>

            <div className="flex items-start gap-3 rounded-md border border-slate-200 bg-white p-3">
              <CheckCircle2 className="mt-0.5 h-4 w-4 text-green-600" />
              <div className="text-sm">
                <div className="font-medium text-slate-900">Status</div>
                <div className="text-slate-600">
                  Trial is currently scheduled and ready to start.
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        {/* Actions */}
        <div className="flex items-center justify-between">
          <Button asChild variant="ghost">
            <Link href={`/trials/${trial.id}`}>
              <ArrowLeft className="mr-2 h-4 w-4" />
              Cancel
            </Link>
          </Button>

          <form action={startTrial}>
            <Button type="submit" className="shadow-sm">
              <Play className="mr-2 h-4 w-4" />
              Start Trial
            </Button>
          </form>
        </div>
      </div>
    </div>
  );
}

export async function generateMetadata({
  params,
}: StartPageProps): Promise<{ title: string; description: string }> {
  try {
    const { trialId } = await params;
    const trial = await api.trials.get({ id: trialId });
    return {
      title: `Start Trial - ${trial.experiment.name} | HRIStudio`,
      description: `Preflight and start trial for participant ${trial.participant.participantCode}`,
    };
  } catch {
    return {
      title: "Start Trial | HRIStudio",
      description: "Preflight checklist to start an HRI trial",
    };
  }
}
