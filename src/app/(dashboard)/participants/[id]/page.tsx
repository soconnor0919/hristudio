"use client";

import { formatDistanceToNow } from "date-fns";
import {
  AlertCircle,
  Calendar,
  CheckCircle,
  Edit,
  Mail,
  Trash2,
  XCircle,
} from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { useEffect, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  EntityView,
  EntityViewHeader,
  EntityViewSection,
  EntityViewSidebar,
  EmptyState,
  InfoGrid,
  QuickActions,
} from "~/components/ui/entity-view";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useSession } from "next-auth/react";
import { api } from "~/trpc/react";

interface ParticipantDetailPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default function ParticipantDetailPage({
  params,
}: ParticipantDetailPageProps) {
  const { data: session } = useSession();
  const [participant, setParticipant] = useState<{
    id: string;
    name: string | null;
    email: string | null;
    participantCode: string;
    study: { id: string; name: string } | null;
    demographics: unknown;
    notes: string | null;
    consentGiven: boolean;
    consentDate: Date | null;
    createdAt: Date;
    updatedAt: Date;
    studyId: string;
    trials: unknown[];
    consents: unknown[];
  } | null>(null);
  const [trials, setTrials] = useState<
    {
      id: string;
      status: string;
      createdAt: Date;
      duration: number | null;
      experiment: { name: string } | null;
    }[]
  >([]);
  const [loading, setLoading] = useState(true);
  const [resolvedParams, setResolvedParams] = useState<{ id: string } | null>(
    null,
  );

  useEffect(() => {
    async function resolveParams() {
      const resolved = await params;
      setResolvedParams(resolved);
    }
    void resolveParams();
  }, [params]);

  const { data: participantData } = api.participants.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: trialsData } = api.trials.list.useQuery(
    { participantId: resolvedParams?.id ?? "", limit: 10 },
    { enabled: !!resolvedParams?.id },
  );

  useEffect(() => {
    if (participantData) {
      setParticipant(participantData);
    }
    if (trialsData) {
      setTrials(trialsData);
    }
    if (participantData !== undefined) {
      setLoading(false);
    }
  }, [participantData, trialsData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Participants", href: "/participants" },
    {
      label: participant?.name ?? participant?.participantCode ?? "Participant",
    },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (loading || !participant) {
    return <div>Loading...</div>;
  }

  const userRole = session.user.roles?.[0]?.role ?? "observer";
  const canEdit = ["administrator", "researcher"].includes(userRole);

  return (
    <EntityView>
      {/* Header */}
      <EntityViewHeader
        title={participant.name ?? participant.participantCode}
        subtitle={
          participant.name
            ? `Code: ${participant.participantCode}`
            : "Participant"
        }
        icon="Users"
        actions={
          canEdit && (
            <>
              <Button variant="outline" asChild>
                <Link href={`/participants/${resolvedParams?.id}/edit`}>
                  <Edit className="mr-2 h-4 w-4" />
                  Edit
                </Link>
              </Button>
              <Button variant="destructive" size="sm">
                <Trash2 className="mr-2 h-4 w-4" />
                Delete
              </Button>
            </>
          )
        }
      />

      <div className="grid gap-6 lg:grid-cols-3">
        {/* Main Content */}
        <div className="space-y-6 lg:col-span-2">
          {/* Participant Information */}
          <EntityViewSection title="Participant Information" icon="FileText">
            <InfoGrid
              items={[
                {
                  label: "Participant Code",
                  value: (
                    <code className="bg-muted rounded px-2 py-1 font-mono text-sm">
                      {participant.participantCode}
                    </code>
                  ),
                },
                {
                  label: "Name",
                  value: participant?.name ?? "Not provided",
                },
                {
                  label: "Email",
                  value: participant?.email ? (
                    <div className="flex items-center gap-2">
                      <Mail className="h-4 w-4" />
                      <a
                        href={`mailto:${participant.email}`}
                        className="text-primary hover:underline"
                      >
                        {participant.email}
                      </a>
                    </div>
                  ) : (
                    "Not provided"
                  ),
                },
                {
                  label: "Study",
                  value: participant?.study ? (
                    <Link
                      href={`/studies/${participant.study.id}`}
                      className="text-primary hover:underline"
                    >
                      {participant.study.name}
                    </Link>
                  ) : (
                    "No study assigned"
                  ),
                },
              ]}
            />

            {/* Demographics */}
            {participant?.demographics &&
            typeof participant.demographics === "object" &&
            participant.demographics !== null &&
            Object.keys(participant.demographics as Record<string, unknown>)
              .length > 0 ? (
              <div className="border-t pt-4">
                <h4 className="text-muted-foreground mb-3 text-sm font-medium">
                  Demographics
                </h4>
                <InfoGrid
                  items={(() => {
                    const demo = participant.demographics as Record<
                      string,
                      unknown
                    >;
                    const items: Array<{ label: string; value: string }> = [];

                    if (demo.age) {
                      items.push({
                        label: "Age",
                        value:
                          typeof demo.age === "number"
                            ? demo.age.toString()
                            : typeof demo.age === "string"
                              ? demo.age
                              : "Unknown",
                      });
                    }

                    if (demo.gender) {
                      items.push({
                        label: "Gender",
                        value:
                          typeof demo.gender === "string"
                            ? demo.gender
                            : "Unknown",
                      });
                    }

                    return items;
                  })()}
                />
              </div>
            ) : null}

            {/* Notes */}
            {participant?.notes && (
              <div className="border-t pt-4">
                <h4 className="text-muted-foreground mb-2 text-sm font-medium">
                  Notes
                </h4>
                <div className="bg-muted rounded p-3 text-sm whitespace-pre-wrap">
                  {participant.notes}
                </div>
              </div>
            )}
          </EntityViewSection>

          {/* Trial History */}
          <EntityViewSection
            title="Trial History"
            icon="Play"
            description="Experimental sessions for this participant"
            actions={
              canEdit && (
                <Button size="sm" asChild>
                  <Link
                    href={`/trials/new?participantId=${resolvedParams?.id}`}
                  >
                    Schedule Trial
                  </Link>
                </Button>
              )
            }
          >
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
                        {trial.experiment?.name ?? "Trial"}
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
                        {trial.createdAt
                          ? formatDistanceToNow(new Date(trial.createdAt), {
                              addSuffix: true,
                            })
                          : "Not scheduled"}
                      </span>
                      {trial.duration && (
                        <span>{Math.round(trial.duration / 60)} min</span>
                      )}
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <EmptyState
                icon="Play"
                title="No Trials Yet"
                description="This participant hasn't been assigned to any trials."
                action={
                  canEdit && (
                    <Button asChild>
                      <Link
                        href={`/trials/new?participantId=${resolvedParams?.id}`}
                      >
                        Schedule First Trial
                      </Link>
                    </Button>
                  )
                }
              />
            )}
          </EntityViewSection>
        </div>

        {/* Sidebar */}
        <EntityViewSidebar>
          {/* Consent Status */}
          <EntityViewSection title="Consent Status" icon="Shield">
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <span className="text-sm">Informed Consent</span>
                <Badge
                  variant={
                    participant?.consentGiven ? "default" : "destructive"
                  }
                >
                  {participant?.consentGiven ? (
                    <>
                      <CheckCircle className="mr-1 h-3 w-3" />
                      Given
                    </>
                  ) : (
                    <>
                      <XCircle className="mr-1 h-3 w-3" />
                      Not Given
                    </>
                  )}
                </Badge>
              </div>

              {participant?.consentDate && (
                <div className="text-muted-foreground text-sm">
                  Consented:{" "}
                  {formatDistanceToNow(new Date(participant.consentDate), {
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
          </EntityViewSection>

          {/* Registration Details */}
          <EntityViewSection title="Registration Details" icon="Calendar">
            <InfoGrid
              columns={1}
              items={[
                {
                  label: "Registered",
                  value: formatDistanceToNow(participant?.createdAt, {
                    addSuffix: true,
                  }),
                },
                ...(participant.updatedAt &&
                participant.updatedAt !== participant.createdAt
                  ? [
                      {
                        label: "Last Updated",
                        value: formatDistanceToNow(participant.updatedAt, {
                          addSuffix: true,
                        }),
                      },
                    ]
                  : []),
              ]}
            />
          </EntityViewSection>

          {/* Quick Actions */}
          {canEdit && (
            <EntityViewSection title="Quick Actions" icon="Edit">
              <QuickActions
                actions={[
                  {
                    label: "Schedule Trial",
                    icon: "Play",
                    href: `/trials/new?participantId=${resolvedParams?.id}`,
                  },
                  {
                    label: "Edit Information",
                    icon: "Edit",
                    href: `/participants/${resolvedParams?.id}/edit`,
                  },
                  {
                    label: "Export Data",
                    icon: "FileText",
                    href: `/participants/${resolvedParams?.id}/export`,
                  },
                ]}
              />
            </EntityViewSection>
          )}
        </EntityViewSidebar>
      </div>
    </EntityView>
  );
}
