import { formatDistanceToNow } from "date-fns";
import {
    AlertCircle, ArrowLeft, Calendar, Edit, FileText, Mail, Play, Shield, Trash2, Users
} from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
    Card,
    CardContent,
    CardDescription,
    CardHeader,
    CardTitle
} from "~/components/ui/card";
import { auth } from "~/server/auth";
import { api } from "~/trpc/server";

interface ParticipantDetailPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function ParticipantDetailPage({
  params,
}: ParticipantDetailPageProps) {
  const resolvedParams = await params;
  const session = await auth();

  if (!session?.user) {
    return notFound();
  }

  try {
    const participant = await api.participants.get({ id: resolvedParams.id });

    if (!participant) {
      return notFound();
    }

    const userRole = session.user.roles?.[0]?.role ?? "observer";
    const canEdit = ["administrator", "researcher"].includes(userRole);
    const canDelete = ["administrator", "researcher"].includes(userRole);

    // Get participant's trials
    const trials = await api.trials.list({
      participantId: resolvedParams.id,
      limit: 10,
    });

    return (
      <div className="container mx-auto max-w-6xl px-4 py-8">
        {/* Header */}
        <div className="mb-8">
          <div className="mb-4 flex items-center gap-4">
            <Button variant="ghost" size="sm" asChild>
              <Link href="/participants">
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Participants
              </Link>
            </Button>
          </div>

          <div className="flex items-center justify-between">
            <div className="flex items-center gap-4">
              <div className="bg-primary text-primary-foreground flex h-16 w-16 items-center justify-center rounded-lg">
                <Users className="h-8 w-8" />
              </div>
              <div>
                <h1 className="text-foreground text-3xl font-bold">
                  {participant.name || participant.participantCode}
                </h1>
                <p className="text-muted-foreground text-lg">
                  {participant.name
                    ? `Code: ${participant.participantCode}`
                    : "Participant"}
                </p>
              </div>
            </div>

            {canEdit && (
              <div className="flex gap-2">
                <Button variant="outline" asChild>
                  <Link href={`/participants/${resolvedParams.id}/edit`}>
                    <Edit className="mr-2 h-4 w-4" />
                    Edit
                  </Link>
                </Button>
                <Button variant="destructive" size="sm">
                  <Trash2 className="mr-2 h-4 w-4" />
                  Delete
                </Button>
              </div>
            )}
          </div>
        </div>

        <div className="grid gap-6 lg:grid-cols-3">
          {/* Main Content */}
          <div className="space-y-6 lg:col-span-2">
            {/* Participant Information */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <FileText className="h-5 w-5" />
                  Participant Information
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid gap-4 md:grid-cols-2">
                  <div>
                    <h4 className="text-muted-foreground text-sm font-medium">
                      Participant Code
                    </h4>
                    <p className="bg-muted rounded px-2 py-1 font-mono text-sm">
                      {participant.participantCode}
                    </p>
                  </div>

                  {participant.name && (
                    <div>
                      <h4 className="text-muted-foreground text-sm font-medium">
                        Name
                      </h4>
                      <p className="text-sm">{participant.name}</p>
                    </div>
                  )}

                  {participant.email && (
                    <div>
                      <h4 className="text-muted-foreground text-sm font-medium">
                        Email
                      </h4>
                      <p className="flex items-center gap-2 text-sm">
                        <Mail className="h-4 w-4" />
                        <a
                          href={`mailto:${participant.email}`}
                          className="hover:underline"
                        >
                          {participant.email}
                        </a>
                      </p>
                    </div>
                  )}

                  <div>
                    <h4 className="text-muted-foreground text-sm font-medium">
                      Study
                    </h4>
                    <p className="text-sm">
                      <Link
                        href={`/studies/${(participant.study as any)?.id}`}
                        className="text-primary hover:underline"
                      >
                        {(participant.study as any)?.name}
                      </Link>
                    </p>
                  </div>
                </div>

                {participant.demographics &&
                  typeof participant.demographics === "object" &&
                  Object.keys(participant.demographics).length > 0 ? (
                    <div className="border-t pt-4">
                      <h4 className="text-muted-foreground mb-2 text-sm font-medium">
                        Demographics
                      </h4>
                      <div className="grid gap-4 md:grid-cols-2">
                        {(participant.demographics as Record<string, any>)
                          ?.age && (
                          <div>
                            <span className="text-sm font-medium">Age:</span>{" "}
                            <span className="text-sm">
                              {String(
                                (
                                  participant.demographics as Record<
                                    string,
                                    any
                                  >
                                ).age,
                              )}
                            </span>
                          </div>
                        )}
                        {(participant.demographics as Record<string, any>)
                          ?.gender && (
                          <div>
                            <span className="text-sm font-medium">Gender:</span>{" "}
                            <span className="text-sm">
                              {String(
                                (
                                  participant.demographics as Record<
                                    string,
                                    any
                                  >
                                ).gender,
                              )}
                            </span>
                          </div>
                        )}
                      </div>
                    </div>
                  ) : null}

                {/* Notes */}
                {participant.notes && (
                  <div className="border-t pt-4">
                    <h4 className="text-muted-foreground mb-2 text-sm font-medium">
                      Notes
                    </h4>
                    <p className="bg-muted rounded p-3 text-sm whitespace-pre-wrap">
                      {participant.notes}
                    </p>
                  </div>
                )}
              </CardContent>
            </Card>

            {/* Trial History */}
            <Card>
              <CardHeader>
                <div className="flex items-center justify-between">
                  <CardTitle className="flex items-center gap-2">
                    <Play className="h-5 w-5" />
                    Trial History
                  </CardTitle>
                  {canEdit && (
                    <Button size="sm" asChild>
                      <Link href={`/trials/new?participantId=${resolvedParams.id}`}>
                        Schedule Trial
                      </Link>
                    </Button>
                  )}
                </div>
                <CardDescription>
                  Experimental sessions for this participant
                </CardDescription>
              </CardHeader>
              <CardContent>
                {trials.length > 0 ? (
                  <div className="space-y-3">
                    {trials.map((trial) => (
                      <div
                        key={trial.id}
                        className="hover:bg-muted/50 rounded-lg border p-4 transition-colors"
                      >
                        <div className="mb-2 flex items-center justify-between">
                          <Link
                            href={`/trials/${trial.id}`}
                            className="font-medium hover:underline"
                          >
                            {trial.experiment?.name || "Trial"}
                          </Link>
                          <Badge
                            variant={
                              trial.status === "completed"
                                ? "default"
                                : trial.status === "in_progress"
                                  ? "secondary"
                                  : trial.status === "failed"
                                    ? "destructive"
                                    : "outline"
                            }
                          >
                            {trial.status.replace("_", " ")}
                          </Badge>
                        </div>
                        <div className="text-muted-foreground flex items-center gap-4 text-sm">
                          <span className="flex items-center gap-1">
                            <Calendar className="h-4 w-4" />
                            {(trial as any).scheduledAt
                              ? formatDistanceToNow(
                                  (trial as any).scheduledAt,
                                  {
                                    addSuffix: true,
                                  },
                                )
                              : "Not scheduled"}
                          </span>
                          {trial.duration && (
                            <span>
                              {Math.round(trial.duration / 60)} minutes
                            </span>
                          )}
                        </div>
                      </div>
                    ))}
                  </div>
                ) : (
                  <div className="py-8 text-center">
                    <Play className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
                    <h3 className="mb-2 font-medium">No Trials Yet</h3>
                    <p className="text-muted-foreground mb-4 text-sm">
                      This participant hasn't been assigned to any trials.
                    </p>
                    {canEdit && (
                      <Button asChild>
                        <Link href={`/trials/new?participantId=${resolvedParams.id}`}>
                          Schedule First Trial
                        </Link>
                      </Button>
                    )}
                  </div>
                )}
              </CardContent>
            </Card>
          </div>

          {/* Sidebar */}
          <div className="space-y-6">
            {/* Consent Status */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center gap-2 text-base">
                  <Shield className="h-4 w-4" />
                  Consent Status
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  <div className="flex items-center justify-between">
                    <span className="text-sm">Informed Consent</span>
                    <Badge
                      variant={
                        participant.consentGiven ? "default" : "destructive"
                      }
                    >
                      {participant.consentGiven ? "Given" : "Not Given"}
                    </Badge>
                  </div>

                  {participant.consentDate && (
                    <div className="text-muted-foreground text-sm">
                      Consented:{" "}
                      {formatDistanceToNow(participant.consentDate, {
                        addSuffix: true,
                      })}
                    </div>
                  )}

                  {!participant.consentGiven && (
                    <Alert>
                      <AlertCircle className="h-4 w-4" />
                      <AlertDescription className="text-sm">
                        Consent required before trials can be conducted.
                      </AlertDescription>
                    </Alert>
                  )}
                </div>
              </CardContent>
            </Card>

            {/* Registration Details */}
            <Card>
              <CardHeader>
                <CardTitle className="text-base">
                  Registration Details
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div>
                  <h4 className="text-muted-foreground text-sm font-medium">
                    Registered
                  </h4>
                  <p className="text-sm">
                    {formatDistanceToNow(participant.createdAt, {
                      addSuffix: true,
                    })}
                  </p>
                </div>

                {participant.updatedAt &&
                  participant.updatedAt !== participant.createdAt && (
                    <div>
                      <h4 className="text-muted-foreground text-sm font-medium">
                        Last Updated
                      </h4>
                      <p className="text-sm">
                        {formatDistanceToNow(participant.updatedAt, {
                          addSuffix: true,
                        })}
                      </p>
                    </div>
                  )}
              </CardContent>
            </Card>

            {/* Quick Actions */}
            {canEdit && (
              <Card>
                <CardHeader>
                  <CardTitle className="text-base">Quick Actions</CardTitle>
                </CardHeader>
                <CardContent className="space-y-2">
                  <Button
                    variant="outline"
                    className="w-full justify-start"
                    asChild
                  >
                    <Link href={`/trials/new?participantId=${resolvedParams.id}`}>
                      <Play className="mr-2 h-4 w-4" />
                      Schedule Trial
                    </Link>
                  </Button>

                  <Button
                    variant="outline"
                    className="w-full justify-start"
                    asChild
                  >
                    <Link href={`/participants/${resolvedParams.id}/edit`}>
                      <Edit className="mr-2 h-4 w-4" />
                      Edit Information
                    </Link>
                  </Button>

                  <Button variant="outline" className="w-full justify-start">
                    <FileText className="mr-2 h-4 w-4" />
                    Export Data
                  </Button>
                </CardContent>
              </Card>
            )}
          </div>
        </div>
      </div>
    );
  } catch (_error) {
    return notFound();
  }
}
