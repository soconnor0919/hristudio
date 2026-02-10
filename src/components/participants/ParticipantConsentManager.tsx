"use client";

import { useState } from "react";
import { api } from "~/trpc/react";
import { Button } from "~/components/ui/button";
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogHeader,
    DialogTitle,
    DialogTrigger,
} from "~/components/ui/dialog";
import { ConsentUploadForm } from "./ConsentUploadForm";
import { FileText, Download, CheckCircle, AlertCircle, Upload } from "lucide-react";
import { toast } from "sonner";
import { Badge } from "~/components/ui/badge";
import { cn } from "~/lib/utils";

interface ParticipantConsentManagerProps {
    studyId: string;
    participantId: string;
    consentGiven: boolean;
    consentDate: Date | null;
    existingConsent: {
        id: string;
        storagePath: string | null;
        signedAt: Date;
        consentForm: {
            title: string;
            version: number;
        };
    } | null;
}

export function ParticipantConsentManager({
    studyId,
    participantId,
    consentGiven,
    consentDate,
    existingConsent,
}: ParticipantConsentManagerProps) {
    const [isOpen, setIsOpen] = useState(false);
    const utils = api.useUtils();

    // Fetch active consent forms to know which form to sign/upload against
    const { data: consentForms } = api.participants.getConsentForms.useQuery({ studyId });
    const activeForm = consentForms?.find((f) => f.active) ?? consentForms?.[0];

    // Helper to get download URL
    const { refetch: fetchDownloadUrl } = api.files.getDownloadUrl.useQuery(
        { storagePath: existingConsent?.storagePath ?? "" },
        { enabled: false }
    );

    const handleDownload = async () => {
        if (!existingConsent?.storagePath) return;
        try {
            const result = await fetchDownloadUrl();
            if (result.data?.url) {
                window.open(result.data.url, "_blank");
            } else {
                toast.error("Error", { description: "Could not retrieve document" });
            }
        } catch (error) {
            toast.error("Error", { description: "Failed to get download URL" });
        }
    };

    const handleSuccess = () => {
        setIsOpen(false);
        utils.participants.get.invalidate({ id: participantId });
        toast.success("Success", { description: "Consent recorded successfully" });
    };

    return (
        <div className="rounded-lg border bg-card text-card-foreground shadow-sm">
            <div className="p-6 flex flex-row items-center justify-between space-y-0 pb-2">
                <div className="flex flex-col space-y-1.5">
                    <h3 className="font-semibold leading-none tracking-tight flex items-center gap-2">
                        <FileText className="h-5 w-5" />
                        Consent Status
                    </h3>
                    <p className="text-sm text-muted-foreground">
                        Manage participant consent and forms.
                    </p>
                </div>
                <Badge variant={consentGiven ? "default" : "destructive"}>
                    {consentGiven ? "Consent Given" : "Not Recorded"}
                </Badge>
            </div>
            <div className="p-6 pt-4">
                <div className="flex items-center justify-between">
                    <div className="space-y-1">
                        {consentGiven ? (
                            <>
                                <div className="flex items-center gap-2 text-sm font-medium">
                                    <CheckCircle className="h-4 w-4 text-green-600" />
                                    Signed on {consentDate ? new Date(consentDate).toLocaleDateString() : "Unknown date"}
                                </div>
                                {existingConsent && (
                                    <p className="text-xs text-muted-foreground">
                                        Form: {existingConsent.consentForm.title} (v{existingConsent.consentForm.version})
                                    </p>
                                )}
                            </>
                        ) : (
                            <div className="flex items-center gap-2 text-sm text-muted-foreground">
                                <AlertCircle className="h-4 w-4" />
                                No consent recorded for this participant.
                            </div>
                        )}
                    </div>
                    <div className="flex gap-2">
                        {consentGiven && existingConsent?.storagePath && (
                            <Button variant="outline" size="sm" onClick={handleDownload}>
                                <Download className="mr-2 h-4 w-4" />
                                Download PDF
                            </Button>
                        )}

                        <Dialog open={isOpen} onOpenChange={setIsOpen}>
                            <DialogTrigger asChild>
                                <Button size="sm" variant={consentGiven ? "secondary" : "default"}>
                                    <Upload className="mr-2 h-4 w-4" />
                                    {consentGiven ? "Update Consent" : "Record Consent"}
                                </Button>
                            </DialogTrigger>
                            <DialogContent>
                                <DialogHeader>
                                    <DialogTitle>Upload Signed Consent Form</DialogTitle>
                                    <DialogDescription>
                                        Upload the signed PDF or image of the consent form for this participant.
                                        {activeForm && (
                                            <span className="block mt-1 font-medium text-foreground">
                                                Active Form: {activeForm.title} (v{activeForm.version})
                                            </span>
                                        )}
                                    </DialogDescription>
                                </DialogHeader>
                                {activeForm ? (
                                    <ConsentUploadForm
                                        studyId={studyId}
                                        participantId={participantId}
                                        consentFormId={activeForm.id}
                                        onSuccess={handleSuccess}
                                        onCancel={() => setIsOpen(false)}
                                    />
                                ) : (
                                    <div className="py-4 text-center text-muted-foreground">
                                        No active consent form found for this study. Please create one first.
                                    </div>
                                )}
                            </DialogContent>
                        </Dialog>
                    </div>
                </div>
            </div>
        </div>
    );
}
