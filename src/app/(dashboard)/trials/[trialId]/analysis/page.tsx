import { format } from "date-fns";
import {
  Activity,
  ArrowLeft,
  BarChart3,
  Bot,
  Camera,
  CheckCircle,
  Clock,
  Download,
  FileText,
  MessageSquare,
  Share,
  Target,
  Timer,
  TrendingUp,
  User,
} from "lucide-react";
import Link from "next/link";
import { notFound, redirect } from "next/navigation";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { Separator } from "~/components/ui/separator";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { auth } from "~/server/auth";
import { api } from "~/trpc/server";

interface AnalysisPageProps {
  params: Promise<{
    trialId: string;
  }>;
}

export default async function AnalysisPage({ params }: AnalysisPageProps) {
  const session = await auth();

  if (!session) {
    redirect("/auth/signin");
  }

  const { trialId } = await params;
  let trial;
  try {
    trial = await api.trials.get({ id: trialId });
  } catch (_error) {
    notFound();
  }

  // Only allow analysis view for completed trials
  if (trial.status !== "completed") {
    redirect(`/trials/${trialId}?error=trial_not_completed`);
  }

  // Calculate trial metrics
  const duration =
    trial.startedAt && trial.completedAt
      ? Math.floor(
          (new Date(trial.completedAt).getTime() -
            new Date(trial.startedAt).getTime()) /
            1000 /
            60,
        )
      : 0;

  // Mock experiment steps - in real implementation, fetch from experiment API
  const experimentSteps: any[] = [];

  // Mock analysis data - in real implementation, this would come from API
  const analysisData = {
    totalEvents: 45,
    wizardInterventions: 3,
    robotActions: 12,
    mediaCaptures: 8,
    annotations: 15,
    participantResponses: 22,
    averageResponseTime: 2.3,
    completionRate: 100,
    successRate: 95,
    errorCount: 2,
  };

  return (
    <div className="min-h-screen bg-slate-50">
      {/* Header */}
      <div className="border-b border-slate-200 bg-white px-6 py-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <Button variant="ghost" size="sm" asChild>
              <Link href={`/trials/${trial.id}`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trial
              </Link>
            </Button>
            <Separator orientation="vertical" className="h-6" />
            <div>
              <h1 className="text-2xl font-bold text-slate-900">
                Trial Analysis
              </h1>
              <p className="mt-1 text-sm text-slate-600">
                {trial.experiment.name} • Participant:{" "}
                {trial.participant.participantCode}
              </p>
            </div>
          </div>
          <div className="flex items-center space-x-3">
            <Badge className="bg-green-100 text-green-800" variant="secondary">
              <CheckCircle className="mr-1 h-3 w-3" />
              Completed
            </Badge>
            <Button variant="outline">
              <Download className="mr-2 h-4 w-4" />
              Export Data
            </Button>
            <Button variant="outline">
              <Share className="mr-2 h-4 w-4" />
              Share Results
            </Button>
          </div>
        </div>
      </div>

      <div className="space-y-6 p-6">
        {/* Trial Summary Cards */}
        <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-4">
          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <Timer className="h-4 w-4 text-blue-600" />
                <div>
                  <p className="text-sm font-medium text-slate-600">Duration</p>
                  <p className="text-lg font-semibold">{duration} min</p>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <Target className="h-4 w-4 text-green-600" />
                <div>
                  <p className="text-sm font-medium text-slate-600">
                    Completion Rate
                  </p>
                  <p className="text-lg font-semibold">
                    {analysisData.completionRate}%
                  </p>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <Activity className="h-4 w-4 text-purple-600" />
                <div>
                  <p className="text-sm font-medium text-slate-600">
                    Total Events
                  </p>
                  <p className="text-lg font-semibold">
                    {analysisData.totalEvents}
                  </p>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <TrendingUp className="h-4 w-4 text-orange-600" />
                <div>
                  <p className="text-sm font-medium text-slate-600">
                    Success Rate
                  </p>
                  <p className="text-lg font-semibold">
                    {analysisData.successRate}%
                  </p>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Main Analysis Content */}
        <Tabs defaultValue="overview" className="space-y-6">
          <TabsList className="grid w-full grid-cols-5">
            <TabsTrigger value="overview">Overview</TabsTrigger>
            <TabsTrigger value="timeline">Timeline</TabsTrigger>
            <TabsTrigger value="interactions">Interactions</TabsTrigger>
            <TabsTrigger value="media">Media</TabsTrigger>
            <TabsTrigger value="export">Export</TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6">
            <div className="grid grid-cols-1 gap-6 lg:grid-cols-2">
              {/* Performance Metrics */}
              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <BarChart3 className="h-5 w-5" />
                    <span>Performance Metrics</span>
                  </CardTitle>
                </CardHeader>
                <CardContent className="space-y-4">
                  <div className="space-y-3">
                    <div>
                      <div className="mb-1 flex justify-between text-sm">
                        <span>Task Completion</span>
                        <span>{analysisData.completionRate}%</span>
                      </div>
                      <Progress
                        value={analysisData.completionRate}
                        className="h-2"
                      />
                    </div>

                    <div>
                      <div className="mb-1 flex justify-between text-sm">
                        <span>Success Rate</span>
                        <span>{analysisData.successRate}%</span>
                      </div>
                      <Progress
                        value={analysisData.successRate}
                        className="h-2"
                      />
                    </div>

                    <div>
                      <div className="mb-1 flex justify-between text-sm">
                        <span>Response Time (avg)</span>
                        <span>{analysisData.averageResponseTime}s</span>
                      </div>
                      <Progress value={75} className="h-2" />
                    </div>
                  </div>

                  <Separator />

                  <div className="grid grid-cols-2 gap-4 text-center">
                    <div>
                      <div className="text-lg font-semibold text-green-600">
                        {experimentSteps.length}
                      </div>
                      <div className="text-xs text-slate-600">
                        Steps Completed
                      </div>
                    </div>
                    <div>
                      <div className="text-lg font-semibold text-red-600">
                        {analysisData.errorCount}
                      </div>
                      <div className="text-xs text-slate-600">Errors</div>
                    </div>
                  </div>
                </CardContent>
              </Card>

              {/* Event Breakdown */}
              <Card>
                <CardHeader>
                  <CardTitle className="flex items-center space-x-2">
                    <Activity className="h-5 w-5" />
                    <span>Event Breakdown</span>
                  </CardTitle>
                </CardHeader>
                <CardContent className="space-y-4">
                  <div className="space-y-3">
                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <Bot className="h-4 w-4 text-green-600" />
                        <span className="text-sm">Robot Actions</span>
                      </div>
                      <Badge variant="outline">
                        {analysisData.robotActions}
                      </Badge>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <User className="h-4 w-4 text-blue-600" />
                        <span className="text-sm">Wizard Interventions</span>
                      </div>
                      <Badge variant="outline">
                        {analysisData.wizardInterventions}
                      </Badge>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <MessageSquare className="h-4 w-4 text-purple-600" />
                        <span className="text-sm">Participant Responses</span>
                      </div>
                      <Badge variant="outline">
                        {analysisData.participantResponses}
                      </Badge>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <Camera className="h-4 w-4 text-indigo-600" />
                        <span className="text-sm">Media Captures</span>
                      </div>
                      <Badge variant="outline">
                        {analysisData.mediaCaptures}
                      </Badge>
                    </div>

                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-2">
                        <FileText className="h-4 w-4 text-orange-600" />
                        <span className="text-sm">Annotations</span>
                      </div>
                      <Badge variant="outline">
                        {analysisData.annotations}
                      </Badge>
                    </div>
                  </div>
                </CardContent>
              </Card>
            </div>

            {/* Trial Information */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <FileText className="h-5 w-5" />
                  <span>Trial Information</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="grid grid-cols-1 gap-4 md:grid-cols-2 lg:grid-cols-4">
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Started
                    </label>
                    <p className="text-sm">
                      {trial.startedAt
                        ? format(trial.startedAt, "PPP 'at' p")
                        : "N/A"}
                    </p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Completed
                    </label>
                    <p className="text-sm">
                      {trial.completedAt
                        ? format(trial.completedAt, "PPP 'at' p")
                        : "N/A"}
                    </p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Participant
                    </label>
                    <p className="text-sm">
                      {trial.participant.participantCode}
                    </p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-600">
                      Wizard
                    </label>
                    <p className="text-sm">N/A</p>
                  </div>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="timeline" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Clock className="h-5 w-5" />
                  <span>Event Timeline</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="py-12 text-center text-slate-500">
                  <Clock className="mx-auto mb-4 h-12 w-12 opacity-50" />
                  <h3 className="mb-2 text-lg font-medium">
                    Timeline Analysis
                  </h3>
                  <p className="text-sm">
                    Detailed timeline visualization and event analysis will be
                    available here. This would show the sequence of all trial
                    events with timestamps.
                  </p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="interactions" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <MessageSquare className="h-5 w-5" />
                  <span>Interaction Analysis</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="py-12 text-center text-slate-500">
                  <MessageSquare className="mx-auto mb-4 h-12 w-12 opacity-50" />
                  <h3 className="mb-2 text-lg font-medium">
                    Interaction Patterns
                  </h3>
                  <p className="text-sm">
                    Analysis of participant-robot interactions, communication
                    patterns, and behavioral observations will be displayed
                    here.
                  </p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="media" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Camera className="h-5 w-5" />
                  <span>Media Recordings</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="py-12 text-center text-slate-500">
                  <Camera className="mx-auto mb-4 h-12 w-12 opacity-50" />
                  <h3 className="mb-2 text-lg font-medium">Media Gallery</h3>
                  <p className="text-sm">
                    Video recordings, audio captures, and sensor data
                    visualizations from the trial will be available for review
                    here.
                  </p>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="export" className="space-y-6">
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Download className="h-5 w-5" />
                  <span>Export Data</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <p className="text-sm text-slate-600">
                  Export trial data in various formats for further analysis or
                  reporting.
                </p>

                <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
                  <Button
                    variant="outline"
                    className="h-auto justify-start p-4"
                  >
                    <div className="flex items-start space-x-3">
                      <FileText className="mt-0.5 h-5 w-5" />
                      <div className="text-left">
                        <div className="font-medium">Trial Report (PDF)</div>
                        <div className="mt-1 text-xs text-slate-500">
                          Complete analysis report with visualizations
                        </div>
                      </div>
                    </div>
                  </Button>

                  <Button
                    variant="outline"
                    className="h-auto justify-start p-4"
                  >
                    <div className="flex items-start space-x-3">
                      <BarChart3 className="mt-0.5 h-5 w-5" />
                      <div className="text-left">
                        <div className="font-medium">Raw Data (CSV)</div>
                        <div className="mt-1 text-xs text-slate-500">
                          Event data, timestamps, and measurements
                        </div>
                      </div>
                    </div>
                  </Button>

                  <Button
                    variant="outline"
                    className="h-auto justify-start p-4"
                  >
                    <div className="flex items-start space-x-3">
                      <Camera className="mt-0.5 h-5 w-5" />
                      <div className="text-left">
                        <div className="font-medium">Media Archive (ZIP)</div>
                        <div className="mt-1 text-xs text-slate-500">
                          All video, audio, and sensor recordings
                        </div>
                      </div>
                    </div>
                  </Button>

                  <Button
                    variant="outline"
                    className="h-auto justify-start p-4"
                  >
                    <div className="flex items-start space-x-3">
                      <MessageSquare className="mt-0.5 h-5 w-5" />
                      <div className="text-left">
                        <div className="font-medium">Annotations (JSON)</div>
                        <div className="mt-1 text-xs text-slate-500">
                          Researcher notes and coded observations
                        </div>
                      </div>
                    </div>
                  </Button>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
}

// Generate metadata for the page
export async function generateMetadata({ params }: AnalysisPageProps) {
  try {
    const { trialId } = await params;
    const trial = await api.trials.get({ id: trialId });
    return {
      title: `Analysis - ${trial.experiment.name} | HRIStudio`,
      description: `Analysis dashboard for trial with participant ${trial.participant.participantCode}`,
    };
  } catch {
    return {
      title: "Trial Analysis | HRIStudio",
      description: "Analyze trial data and participant interactions",
    };
  }
}
