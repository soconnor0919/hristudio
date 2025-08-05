import { format, formatDistanceToNow } from "date-fns";
import {
  Activity,
  AlertTriangle,
  ArrowLeft,
  BarChart3,
  Bot,
  CheckCircle,
  Clock,
  Download,
  Eye,
  Play,
  Settings,
  Share,
  Target,
  Timer,
  User,
  Users,
  XCircle,
} from "lucide-react";
import Link from "next/link";
import { notFound, redirect } from "next/navigation";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";
import { auth } from "~/server/auth";
import { api } from "~/trpc/server";

interface TrialDetailPageProps {
  params: Promise<{
    trialId: string;
  }>;
  searchParams: Promise<{
    error?: string;
  }>;
}

export default async function TrialDetailPage({
  params,
  searchParams,
}: TrialDetailPageProps) {
  const session = await auth();

  if (!session) {
    redirect("/auth/signin");
  }

  const { trialId } = await params;
  const { error } = await searchParams;
  let trial;
  try {
    trial = await api.trials.get({ id: trialId });
  } catch (_error) {
    notFound();
  }

  const userRole = session.user.roles?.[0]?.role;
  const canControl =
    userRole && ["wizard", "researcher", "administrator"].includes(userRole);

  const statusConfig = {
    scheduled: {
      label: "Scheduled",
      className: "bg-blue-100 text-blue-800",
      icon: Clock,
    },
    in_progress: {
      label: "In Progress",
      className: "bg-green-100 text-green-800",
      icon: Activity,
    },
    completed: {
      label: "Completed",
      className: "bg-gray-100 text-gray-800",
      icon: CheckCircle,
    },
    aborted: {
      label: "Aborted",
      className: "bg-red-100 text-red-800",
      icon: XCircle,
    },
    failed: {
      label: "Failed",
      className: "bg-red-100 text-red-800",
      icon: AlertTriangle,
    },
  };

  const currentStatus = statusConfig[trial.status];
  const StatusIcon = currentStatus.icon;

  // Calculate trial duration
  const duration =
    trial.startedAt && trial.completedAt
      ? Math.floor(
          (new Date(trial.completedAt).getTime() -
            new Date(trial.startedAt).getTime()) /
            1000 /
            60,
        )
      : trial.startedAt
        ? Math.floor(
            (Date.now() - new Date(trial.startedAt).getTime()) / 1000 / 60,
          )
        : null;

  // Mock experiment steps - in real implementation, fetch from experiment API
  const experimentSteps: any[] = [];
  const stepTypes = experimentSteps.reduce(
    (acc: Record<string, number>, step: any) => {
      acc[step.type] = (acc[step.type] || 0) + 1;
      return acc;
    },
    {},
  );

  return (
    <div className="min-h-screen bg-slate-50">
      {/* Header */}
      <div className="border-b border-slate-200 bg-white px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Button variant="ghost" size="sm" asChild>
              <Link href="/trials">
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </Link>
            </Button>
            <Separator orientation="vertical" className="h-6" />
            <div>
              <h1 className="text-2xl font-bold text-slate-900">
                Trial Details
              </h1>
              <p className="mt-1 text-sm text-slate-600">
                {trial.experiment.name} • Participant:{" "}
                {trial.participant.participantCode}
              </p>
            </div>
          </div>
          <div className="flex items-center space-x-3">
            <Badge className={currentStatus.className} variant="secondary">
              <StatusIcon className="mr-1 h-3 w-3" />
              {currentStatus.label}
            </Badge>
          </div>
        </div>
      </div>

      {/* Error Alert */}
      {error && (
        <div className="px-6 pt-4">
          <Alert variant="destructive">
            <AlertDescription>
              {error === "trial_not_active" &&
                "This trial is not currently active for wizard control."}
              {error === "insufficient_permissions" &&
                "You don't have permission to access the wizard interface."}
            </AlertDescription>
          </Alert>
        </div>
      )}

      <div className="space-y-6 p-6">
        {/* Quick Actions */}
        <div className="flex items-center space-x-3">
          {trial.status === "scheduled" && canControl && (
            <Button asChild>
              <Link href={`/trials/${trial.id}/wizard`}>
                <Play className="mr-2 h-4 w-4" />
                Start Trial
              </Link>
            </Button>
          )}
          {trial.status === "in_progress" && (
            <Button asChild>
              <Link href={`/trials/${trial.id}/wizard`}>
                <Eye className="mr-2 h-4 w-4" />
                Wizard Interface
              </Link>
            </Button>
          )}
          {trial.status === "completed" && (
            <Button asChild>
              <Link href={`/trials/${trial.id}/analysis`}>
                <BarChart3 className="mr-2 h-4 w-4" />
                View Analysis
              </Link>
            </Button>
          )}
          <Button variant="outline" asChild>
            <Link href={`/experiments/${trial.experiment.id}/designer`}>
              <Settings className="mr-2 h-4 w-4" />
              View Experiment
            </Link>
          </Button>
          <Button variant="outline">
            <Share className="mr-2 h-4 w-4" />
            Share
          </Button>
          <Button variant="outline">
            <Download className="mr-2 h-4 w-4" />
            Export Data
          </Button>
        </div>

        <div className="grid grid-cols-1 gap-6 lg:grid-cols-3">
          {/* Main Content */}
          <div className="space-y-6 lg:col-span-2">
            {/* Trial Overview */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Target className="h-5 w-5" />
                  <span>Trial Overview</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Trial ID
                    </label>
                    <p className="font-mono text-sm">{trial.id}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Status
                    </label>
                    <div className="mt-1 flex items-center space-x-2">
                      <Badge
                        className={currentStatus.className}
                        variant="secondary"
                      >
                        <StatusIcon className="mr-1 h-3 w-3" />
                        {currentStatus.label}
                      </Badge>
                    </div>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Scheduled
                    </label>
                    <p className="text-sm">
                      {trial.startedAt
                        ? format(trial.startedAt, "PPP 'at' p")
                        : "Not scheduled"}
                    </p>
                  </div>
                  {trial.startedAt && (
                    <div>
                      <label className="text-sm font-medium text-slate-600">
                        Started
                      </label>
                      <p className="text-sm">
                        {format(trial.startedAt, "PPP 'at' p")}
                      </p>
                      <p className="text-xs text-slate-500">
                        {formatDistanceToNow(trial.startedAt, {
                          addSuffix: true,
                        })}
                      </p>
                    </div>
                  )}
                  {trial.completedAt && (
                    <div>
                      <label className="text-sm font-medium text-slate-600">
                        Completed
                      </label>
                      <p className="text-sm">
                        {format(trial.completedAt, "PPP 'at' p")}
                      </p>
                      <p className="text-xs text-slate-500">
                        {formatDistanceToNow(trial.completedAt, {
                          addSuffix: true,
                        })}
                      </p>
                    </div>
                  )}
                  {duration !== null && (
                    <div>
                      <label className="text-sm font-medium text-slate-600">
                        Duration
                      </label>
                      <div className="flex items-center space-x-1">
                        <Timer className="h-3 w-3 text-slate-500" />
                        <span className="text-sm">{duration} minutes</span>
                      </div>
                    </div>
                  )}
                </div>

                {trial.notes && (
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Notes
                    </label>
                    <p className="mt-1 text-sm text-slate-700">{trial.notes}</p>
                  </div>
                )}
              </CardContent>
            </Card>

            {/* Experiment Details */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Bot className="h-5 w-5" />
                  <span>Experiment Protocol</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="flex items-start justify-between">
                  <div>
                    <h3 className="font-medium text-slate-900">
                      {trial.experiment.name}
                    </h3>
                    {trial.experiment.description && (
                      <p className="mt-1 text-sm text-slate-600">
                        {trial.experiment.description}
                      </p>
                    )}
                    <div className="mt-2 flex items-center space-x-4 text-sm text-slate-500">
                      <Link
                        href={`/studies/${trial.experiment.studyId}`}
                        className="text-blue-600 hover:text-blue-800"
                      >
                        Study Details
                      </Link>
                    </div>
                  </div>
                  <Button variant="outline" size="sm" asChild>
                    <Link href={`/experiments/${trial.experiment.id}/designer`}>
                      <Eye className="mr-1 h-3 w-3" />
                      View
                    </Link>
                  </Button>
                </div>

                <Separator />

                {/* Experiment Steps Summary */}
                <div>
                  <h4 className="mb-3 font-medium text-slate-900">
                    Protocol Summary
                  </h4>
                  <div className="grid grid-cols-2 gap-4">
                    <div>
                      <label className="text-sm font-medium text-slate-600">
                        Total Steps
                      </label>
                      <p className="text-lg font-semibold">
                        {experimentSteps.length}
                      </p>
                    </div>
                    <div>
                      <label className="text-sm font-medium text-slate-600">
                        Estimated Duration
                      </label>
                      <p className="text-lg font-semibold">
                        {Math.round(
                          experimentSteps.reduce(
                            (sum: number, step: any) =>
                              sum + (step.duration || 0),
                            0,
                          ) / 60,
                        )}{" "}
                        min
                      </p>
                    </div>
                  </div>

                  {Object.keys(stepTypes).length > 0 && (
                    <div className="mt-4">
                      <label className="mb-2 block text-sm font-medium text-slate-600">
                        Step Types
                      </label>
                      <div className="flex flex-wrap gap-2">
                        {Object.entries(stepTypes).map(([type, count]) => (
                          <Badge
                            key={type}
                            variant="outline"
                            className="text-xs"
                          >
                            {type.replace(/_/g, " ")}: {String(count)}
                          </Badge>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              </CardContent>
            </Card>

            {/* Trial Progress */}
            {trial.status === "in_progress" && (
              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <Activity className="h-5 w-5" />
                    <span>Current Progress</span>
                  </CardTitle>
                </CardHeader>
                <CardContent>
                  <div className="space-y-4">
                    <div className="flex items-center justify-between text-sm">
                      <span>Trial Progress</span>
                      <span>Step 1 of {experimentSteps.length}</span>
                    </div>
                    <Progress
                      value={(1 / experimentSteps.length) * 100}
                      className="h-2"
                    />
                    <div className="text-sm text-slate-600">
                      Currently executing the first step of the experiment
                      protocol.
                    </div>
                  </div>
                </CardContent>
              </Card>
            )}
          </div>

          {/* Sidebar */}
          <div className="space-y-6">
            {/* Participant Info */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <User className="h-5 w-5" />
                  <span>Participant</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div>
                  <label className="text-sm font-medium text-slate-600">
                    Participant Code
                  </label>
                  <p className="font-mono text-sm">
                    {trial.participant.participantCode}
                  </p>
                </div>

                <Separator />

                <div className="flex items-center space-x-2 text-sm text-green-600">
                  <CheckCircle className="h-4 w-4" />
                  <span>Consent verified</span>
                </div>

                <Button variant="outline" size="sm" className="w-full">
                  <Eye className="mr-1 h-3 w-3" />
                  View Details
                </Button>
              </CardContent>
            </Card>

            {/* Wizard Assignment */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Users className="h-5 w-5" />
                  <span>Team</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div className="text-sm text-slate-500">No wizard assigned</div>

                <Separator />

                <div>
                  <label className="text-sm font-medium text-slate-600">
                    Your Role
                  </label>
                  <Badge variant="outline" className="mt-1 text-xs">
                    {userRole || "Observer"}
                  </Badge>
                </div>
              </CardContent>
            </Card>

            {/* Quick Stats */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <BarChart3 className="h-5 w-5" />
                  <span>Statistics</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div className="grid grid-cols-2 gap-3 text-center">
                  <div>
                    <div className="text-lg font-semibold text-blue-600">0</div>
                    <div className="text-xs text-slate-600">Events</div>
                  </div>
                  <div>
                    <div className="text-lg font-semibold text-green-600">
                      0
                    </div>
                    <div className="text-xs text-slate-600">Media</div>
                  </div>
                  <div>
                    <div className="text-lg font-semibold text-purple-600">
                      0
                    </div>
                    <div className="text-xs text-slate-600">Annotations</div>
                  </div>
                  <div>
                    <div className="text-lg font-semibold text-orange-600">
                      0
                    </div>
                    <div className="text-xs text-slate-600">Interventions</div>
                  </div>
                </div>

                {trial.status === "completed" && (
                  <>
                    <Separator />
                    <Button
                      variant="outline"
                      size="sm"
                      className="w-full"
                      asChild
                    >
                      <Link href={`/trials/${trial.id}/analysis`}>
                        <BarChart3 className="mr-1 h-3 w-3" />
                        View Full Analysis
                      </Link>
                    </Button>
                  </>
                )}
              </CardContent>
            </Card>

            {/* Recent Activity */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Clock className="h-5 w-5" />
                  <span>Recent Activity</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="py-4 text-center text-sm text-slate-500">
                  No recent activity
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </div>
  );
}

// Generate metadata for the page
export async function generateMetadata({ params }: TrialDetailPageProps) {
  try {
    const { trialId } = await params;
    const trial = await api.trials.get({ id: trialId });
    return {
      title: `${trial.experiment.name} - Trial ${trial.participant.participantCode} | HRIStudio`,
      description: `Trial details for ${trial.experiment.name} with participant ${trial.participant.participantCode}`,
    };
  } catch {
    return {
      title: "Trial Details | HRIStudio",
      description: "View trial information and control wizard interface",
    };
  }
}
