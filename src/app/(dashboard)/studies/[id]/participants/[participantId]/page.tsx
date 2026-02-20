import { notFound } from "next/navigation";
import { api } from "~/trpc/server";
import {
    EntityView,
    EntityViewHeader,
    EntityViewSection,
} from "~/components/ui/entity-view";
import { ParticipantDocuments } from "./participant-documents";
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { Button } from "~/components/ui/button";
import { Edit, Users } from "lucide-react";
import Link from "next/link";
import { PageHeader } from "~/components/ui/page-header";

import { ParticipantConsentManager } from "~/components/participants/ParticipantConsentManager";

interface ParticipantDetailPageProps {
    params: Promise<{ id: string; participantId: string }>;
}

export default async function ParticipantDetailPage({
    params,
}: ParticipantDetailPageProps) {
    const { id: studyId, participantId } = await params;

    const participant = await api.participants.get({ id: participantId });

    if (!participant) {
        notFound();
    }

    // Ensure participant belongs to study
    if (participant.studyId !== studyId) {
        notFound();
    }

    return (
        <EntityView>
            <PageHeader
                title={participant.participantCode}
                description={participant.name ?? "Unnamed Participant"}
                icon={Users}
                badges={[
                    {
                        label: participant.consentGiven ? "Consent Given" : "No Consent",
                        variant: participant.consentGiven ? "default" : "secondary"
                    }
                ]}
                actions={
                    <Button asChild variant="outline" size="sm">
                        <Link href={`/studies/${studyId}/participants/${participantId}/edit`}>
                            <Edit className="mr-2 h-4 w-4" />
                            Edit Participant
                        </Link>
                    </Button>
                }
            />

            <Tabs defaultValue="overview" className="w-full">
                <TabsList className="mb-4">
                    <TabsTrigger value="overview">Overview</TabsTrigger>
                    <TabsTrigger value="files">Files & Documents</TabsTrigger>
                </TabsList>

                <TabsContent value="overview">
                    <div className="grid gap-6 grid-cols-1">
                        <ParticipantConsentManager
                            studyId={studyId}
                            participantId={participantId}
                            consentGiven={participant.consentGiven}
                            consentDate={participant.consentDate}
                            existingConsent={participant.consents[0] ?? null}
                        />
                        <EntityViewSection title="Participant Information" icon="Info">
                            <div className="grid grid-cols-2 gap-4 text-sm md:grid-cols-4">
                                <div>
                                    <span className="text-muted-foreground block mb-1">Code</span>
                                    <span className="font-medium text-base">{participant.participantCode}</span>
                                </div>

                                <div>
                                    <span className="text-muted-foreground block mb-1">Name</span>
                                    <span className="font-medium text-base">{participant.name || "-"}</span>
                                </div>

                                <div>
                                    <span className="text-muted-foreground block mb-1">Email</span>
                                    <span className="font-medium text-base">{participant.email || "-"}</span>
                                </div>

                                <div>
                                    <span className="text-muted-foreground block mb-1">Added</span>
                                    <span className="font-medium text-base">{new Date(participant.createdAt).toLocaleDateString()}</span>
                                </div>

                                <div>
                                    <span className="text-muted-foreground block mb-1">Age</span>
                                    <span className="font-medium text-base">{(participant.demographics as any)?.age || "-"}</span>
                                </div>

                                <div>
                                    <span className="text-muted-foreground block mb-1">Gender</span>
                                    <span className="font-medium capitalize text-base">{(participant.demographics as any)?.gender?.replace("_", " ") || "-"}</span>
                                </div>
                            </div>
                        </EntityViewSection>
                    </div>
                </TabsContent>

                <TabsContent value="files">
                    <EntityViewSection title="Documents" icon="FileText">
                        <ParticipantDocuments participantId={participantId} />
                    </EntityViewSection>
                </TabsContent>
            </Tabs>
        </EntityView>
    );
}
