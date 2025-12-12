"use client";

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { LineChart, BarChart, Clock, Database, FileText } from "lucide-react";
import { formatDistanceToNow } from "date-fns";

interface TrialAnalysisViewProps {
    trial: {
        id: string;
        status: string;
        startedAt: Date | null;
        completedAt: Date | null;
        duration: number | null;
        experiment: { name: string };
        participant: { participantCode: string };
        eventCount?: number;
        mediaCount?: number;
    };
}

export function TrialAnalysisView({ trial }: TrialAnalysisViewProps) {
    return (
        <div className="container mx-auto p-6 space-y-6">
            <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
                <Card>
                    <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                        <CardTitle className="text-sm font-medium">Status</CardTitle>
                        <Clock className="h-4 w-4 text-muted-foreground" />
                    </CardHeader>
                    <CardContent>
                        <div className="text-2xl font-bold capitalize">{trial.status.replace("_", " ")}</div>
                        <p className="text-xs text-muted-foreground">
                            {trial.completedAt
                                ? `Completed ${formatDistanceToNow(new Date(trial.completedAt), { addSuffix: true })}`
                                : "Not completed"}
                        </p>
                    </CardContent>
                </Card>

                <Card>
                    <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                        <CardTitle className="text-sm font-medium">Duration</CardTitle>
                        <BarChart className="h-4 w-4 text-muted-foreground" />
                    </CardHeader>
                    <CardContent>
                        <div className="text-2xl font-bold">
                            {trial.duration ? `${Math.floor(trial.duration / 60)}m ${trial.duration % 60}s` : "N/A"}
                        </div>
                        <p className="text-xs text-muted-foreground">
                            Total execution time
                        </p>
                    </CardContent>
                </Card>

                <Card>
                    <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                        <CardTitle className="text-sm font-medium">Events Logged</CardTitle>
                        <Database className="h-4 w-4 text-muted-foreground" />
                    </CardHeader>
                    <CardContent>
                        <div className="text-2xl font-bold">{trial.eventCount ?? 0}</div>
                        <p className="text-xs text-muted-foreground">
                            System & user events
                        </p>
                    </CardContent>
                </Card>

                <Card>
                    <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                        <CardTitle className="text-sm font-medium">Media Files</CardTitle>
                        <FileText className="h-4 w-4 text-muted-foreground" />
                    </CardHeader>
                    <CardContent>
                        <div className="text-2xl font-bold">{trial.mediaCount ?? 0}</div>
                        <p className="text-xs text-muted-foreground">
                            Recordings & snapshots
                        </p>
                    </CardContent>
                </Card>
            </div>

            <Tabs defaultValue="overview" className="space-y-4">
                <TabsList>
                    <TabsTrigger value="overview">Overview</TabsTrigger>
                    <TabsTrigger value="events">Event Log</TabsTrigger>
                    <TabsTrigger value="charts">Charts</TabsTrigger>
                </TabsList>
                <TabsContent value="overview" className="space-y-4">
                    <Card>
                        <CardHeader>
                            <CardTitle>Analysis Overview</CardTitle>
                            <CardDescription>
                                Summary of trial execution for {trial.participant.participantCode} in experiment {trial.experiment.name}.
                            </CardDescription>
                        </CardHeader>
                        <CardContent className="h-[400px] flex items-center justify-center border-2 border-dashed rounded-md m-4">
                            <div className="text-center text-muted-foreground">
                                <LineChart className="h-10 w-10 mx-auto mb-2 opacity-20" />
                                <p>Detailed analysis visualizations coming soon.</p>
                            </div>
                        </CardContent>
                    </Card>
                </TabsContent>
                <TabsContent value="events">
                    <Card>
                        <CardHeader>
                            <CardTitle>Event Log</CardTitle>
                            <CardDescription>
                                Chronological record of all trial events.
                            </CardDescription>
                        </CardHeader>
                        <CardContent className="h-[400px] flex items-center justify-center border-2 border-dashed rounded-md m-4">
                            <div className="text-center text-muted-foreground">
                                <p>Event log view placeholder.</p>
                            </div>
                        </CardContent>
                    </Card>
                </TabsContent>
            </Tabs>
        </div>
    );
}
