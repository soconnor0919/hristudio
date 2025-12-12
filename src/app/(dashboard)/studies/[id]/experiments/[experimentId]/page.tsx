"use client";

import { formatDistanceToNow } from "date-fns";
import { Calendar, Clock, Edit, Play, Settings, Users } from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
import { useEffect, useState } from "react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
    EntityView,
    EntityViewHeader,
    EntityViewSection,
    EmptyState,
    InfoGrid,
    QuickActions,
    StatsGrid,
} from "~/components/ui/entity-view";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { api } from "~/trpc/react";
import { useSession } from "next-auth/react";
import { useStudyManagement } from "~/hooks/useStudyManagement";

interface ExperimentDetailPageProps {
    params: Promise<{ id: string; experimentId: string }>;
}

const statusConfig = {
    draft: {
        label: "Draft",
        variant: "secondary" as const,
        icon: "FileText" as const,
    },
    testing: {
        label: "Testing",
        variant: "outline" as const,
        icon: "TestTube" as const,
    },
    ready: {
        label: "Ready",
        variant: "default" as const,
        icon: "CheckCircle" as const,
    },
    deprecated: {
        label: "Deprecated",
        variant: "destructive" as const,
        icon: "AlertTriangle" as const,
    },
};

type Experiment = {
    id: string;
    name: string;
    description: string | null;
    status: string;
    createdAt: Date;
    updatedAt: Date;
    study: { id: string; name: string };
    robot: { id: string; name: string; description: string | null } | null;
    protocol?: { blocks: unknown[] } | null;
    visualDesign?: unknown;
    studyId: string;
    createdBy: string;
    robotId: string | null;
    version: number;
};

type Trial = {
    id: string;
    status: string;
    createdAt: Date;
    duration: number | null;
    participant: {
        id: string;
        participantCode: string;
        name?: string | null;
    } | null;
    experiment: { name: string } | null;
    participantId: string | null;
    experimentId: string;
    startedAt: Date | null;
    completedAt: Date | null;
    notes: string | null;
    updatedAt: Date;
    canAccess: boolean;
    userRole: string;
};

export default function ExperimentDetailPage({
    params,
}: ExperimentDetailPageProps) {
    const { data: session } = useSession();
    const [experiment, setExperiment] = useState<Experiment | null>(null);
    const [trials, setTrials] = useState<Trial[]>([]);
    const [loading, setLoading] = useState(true);
    const [resolvedParams, setResolvedParams] = useState<{ id: string; experimentId: string } | null>(
        null,
    );
    const { selectStudy } = useStudyManagement();

    useEffect(() => {
        const resolveParams = async () => {
            const resolved = await params;
            setResolvedParams(resolved);
            // Ensure study context is synced
            if (resolved.id) {
                void selectStudy(resolved.id);
            }
        };
        void resolveParams();
    }, [params, selectStudy]);

    const experimentQuery = api.experiments.get.useQuery(
        { id: resolvedParams?.experimentId ?? "" },
        { enabled: !!resolvedParams?.experimentId },
    );

    const trialsQuery = api.trials.list.useQuery(
        { experimentId: resolvedParams?.experimentId ?? "" },
        { enabled: !!resolvedParams?.experimentId },
    );

    useEffect(() => {
        if (experimentQuery.data) {
            setExperiment(experimentQuery.data);
        }
    }, [experimentQuery.data]);

    useEffect(() => {
        if (trialsQuery.data) {
            setTrials(trialsQuery.data);
        }
    }, [trialsQuery.data]);

    useEffect(() => {
        if (experimentQuery.isLoading || trialsQuery.isLoading) {
            setLoading(true);
        } else {
            setLoading(false);
        }
    }, [experimentQuery.isLoading, trialsQuery.isLoading]);

    // Set breadcrumbs
    useBreadcrumbsEffect([
        {
            label: "Dashboard",
            href: "/",
        },
        {
            label: "Studies",
            href: "/studies",
        },
        {
            label: experiment?.study?.name ?? "Study",
            href: `/studies/${experiment?.study?.id}`,
        },
        {
            label: "Experiments",
            href: `/studies/${experiment?.study?.id}/experiments`,
        },
        {
            label: experiment?.name ?? "Experiment",
        },
    ]);

    if (loading) return <div>Loading...</div>;
    if (experimentQuery.error) return notFound();
    if (!experiment) return notFound();

    const displayName = experiment.name ?? "Untitled Experiment";
    const description = experiment.description;

    // Check if user can edit this experiment
    const userRoles = session?.user?.roles?.map((r) => r.role) ?? [];
    const canEdit =
        userRoles.includes("administrator") || userRoles.includes("researcher");

    const statusInfo =
        statusConfig[experiment.status as keyof typeof statusConfig];

    const studyId = experiment.study.id;
    const experimentId = experiment.id;

    return (
        <EntityView>
            <EntityViewHeader
                title={displayName}
                subtitle={description ?? undefined}
                icon="TestTube"
                status={{
                    label: statusInfo?.label ?? "Unknown",
                    variant: statusInfo?.variant ?? "secondary",
                    icon: statusInfo?.icon ?? "TestTube",
                }}
                actions={
                    canEdit ? (
                        <>
                            <Button asChild variant="outline">
                                <Link href={`/studies/${studyId}/experiments/${experimentId}/edit`}>
                                    <Edit className="mr-2 h-4 w-4" />
                                    Edit
                                </Link>
                            </Button>
                            <Button asChild variant="outline">
                                <Link href={`/studies/${studyId}/experiments/${experimentId}/designer`}>
                                    <Settings className="mr-2 h-4 w-4" />
                                    Designer
                                </Link>
                            </Button>
                            <Button asChild>
                                <Link
                                    href={`/studies/${studyId}/trials/new?experimentId=${experimentId}`}
                                >
                                    <Play className="mr-2 h-4 w-4" />
                                    Start Trial
                                </Link>
                            </Button>
                        </>
                    ) : undefined
                }
            />

            <div className="grid gap-6 lg:grid-cols-3">
                <div className="space-y-6 lg:col-span-2">
                    {/* Basic Information */}
                    <EntityViewSection title="Information" icon="Info">
                        <InfoGrid
                            columns={2}
                            items={[
                                {
                                    label: "Study",
                                    value: experiment.study ? (
                                        <Link
                                            href={`/studies/${experiment.study.id}`}
                                            className="text-primary hover:underline"
                                        >
                                            {experiment.study.name}
                                        </Link>
                                    ) : (
                                        "No study assigned"
                                    ),
                                },
                                {
                                    label: "Status",
                                    value: statusInfo?.label ?? "Unknown",
                                },
                                {
                                    label: "Created",
                                    value: formatDistanceToNow(experiment.createdAt, {
                                        addSuffix: true,
                                    }),
                                },
                                {
                                    label: "Last Updated",
                                    value: formatDistanceToNow(experiment.updatedAt, {
                                        addSuffix: true,
                                    }),
                                },
                            ]}
                        />
                    </EntityViewSection>

                    {/* Protocol Section */}
                    <EntityViewSection
                        title="Experiment Protocol"
                        icon="FileText"
                        actions={
                            canEdit && (
                                <Button asChild variant="outline" size="sm">
                                    <Link href={`/studies/${studyId}/experiments/${experimentId}/designer`}>
                                        <Edit className="mr-2 h-4 w-4" />
                                        Edit Protocol
                                    </Link>
                                </Button>
                            )
                        }
                    >
                        {experiment.protocol &&
                            typeof experiment.protocol === "object" &&
                            experiment.protocol !== null ? (
                            <div className="space-y-3">
                                <div className="text-muted-foreground text-sm">
                                    Protocol contains{" "}
                                    {Array.isArray(
                                        (experiment.protocol as { blocks: unknown[] }).blocks,
                                    )
                                        ? (experiment.protocol as { blocks: unknown[] }).blocks
                                            .length
                                        : 0}{" "}
                                    blocks
                                </div>
                            </div>
                        ) : (
                            <EmptyState
                                icon="FileText"
                                title="No protocol defined"
                                description="Create an experiment protocol using the visual designer"
                                action={
                                    canEdit && (
                                        <Button asChild>
                                            <Link href={`/studies/${studyId}/experiments/${experimentId}/designer`}>
                                                Open Designer
                                            </Link>
                                        </Button>
                                    )
                                }
                            />
                        )}
                    </EntityViewSection>

                    {/* Recent Trials */}
                    <EntityViewSection
                        title="Recent Trials"
                        icon="Play"
                        actions={
                            <Button asChild variant="outline" size="sm">
                                <Link href={`/studies/${experiment.study?.id}/trials`}>
                                    View All
                                </Link>
                            </Button>
                        }
                    >
                        {trials.length > 0 ? (
                            <div className="space-y-3">
                                {trials.slice(0, 5).map((trial) => (
                                    <div
                                        key={trial.id}
                                        className="hover:bg-muted/50 rounded-lg border p-4 transition-colors"
                                    >
                                        <div className="mb-2 flex items-center justify-between">
                                            <Link
                                                href={`/studies/${experiment.study.id}/trials/${trial.id}`}
                                                className="font-medium hover:underline"
                                            >
                                                Trial #{trial.id.slice(-6)}
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
                                                {trial.status.charAt(0).toUpperCase() +
                                                    trial.status.slice(1).replace("_", " ")}
                                            </Badge>
                                        </div>
                                        <div className="text-muted-foreground flex items-center gap-4 text-sm">
                                            <span className="flex items-center gap-1">
                                                <Calendar className="h-4 w-4" />
                                                {formatDistanceToNow(trial.createdAt, {
                                                    addSuffix: true,
                                                })}
                                            </span>
                                            {trial.duration && (
                                                <span className="flex items-center gap-1">
                                                    <Clock className="h-4 w-4" />
                                                    {Math.round(trial.duration / 60)} min
                                                </span>
                                            )}
                                            {trial.participant && (
                                                <span className="flex items-center gap-1">
                                                    <Users className="h-4 w-4" />
                                                    {trial.participant.name ??
                                                        trial.participant.participantCode}
                                                </span>
                                            )}
                                        </div>
                                    </div>
                                ))}
                            </div>
                        ) : (
                            <EmptyState
                                icon="Play"
                                title="No trials yet"
                                description="Start your first trial to collect data"
                                action={
                                    canEdit && (
                                        <Button asChild>
                                            <Link
                                                href={`/studies/${studyId}/trials/new?experimentId=${experimentId}`}
                                            >
                                                Start Trial
                                            </Link>
                                        </Button>
                                    )
                                }
                            />
                        )}
                    </EntityViewSection>
                </div>

                <div className="space-y-6">
                    {/* Statistics */}
                    <EntityViewSection title="Statistics" icon="BarChart">
                        <StatsGrid
                            stats={[
                                {
                                    label: "Total Trials",
                                    value: trials.length,
                                },
                                {
                                    label: "Completed",
                                    value: trials.filter((t) => t.status === "completed").length,
                                },
                                {
                                    label: "In Progress",
                                    value: trials.filter((t) => t.status === "in_progress")
                                        .length,
                                },
                            ]}
                        />
                    </EntityViewSection>

                    {/* Robot Information */}
                    {experiment.robot && (
                        <EntityViewSection title="Robot Platform" icon="Bot">
                            <InfoGrid
                                columns={1}
                                items={[
                                    {
                                        label: "Platform",
                                        value: experiment.robot.name,
                                    },
                                    {
                                        label: "Type",
                                        value: experiment.robot.description ?? "Not specified",
                                    },
                                ]}
                            />
                        </EntityViewSection>
                    )}

                    {/* Quick Actions */}
                    <EntityViewSection title="Quick Actions" icon="Zap">
                        <QuickActions
                            actions={[
                                {
                                    label: "Export Data",
                                    icon: "Download" as const,
                                    href: `/studies/${studyId}/experiments/${experimentId}/export`,
                                },
                                ...(canEdit
                                    ? [
                                        {
                                            label: "Edit Experiment",
                                            icon: "Edit" as const,
                                            href: `/studies/${studyId}/experiments/${experimentId}/edit`,
                                        },
                                        {
                                            label: "Open Designer",
                                            icon: "Palette" as const,
                                            href: `/studies/${studyId}/experiments/${experimentId}/designer`,
                                        },
                                    ]
                                    : []),
                            ]}
                        />
                    </EntityViewSection>
                </div>
            </div>
        </EntityView>
    );
}
