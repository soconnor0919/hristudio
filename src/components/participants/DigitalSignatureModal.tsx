"use client";

import { useRef, useState } from "react";
import SignatureCanvas from "react-signature-canvas";
import { Button } from "~/components/ui/button";
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogHeader,
    DialogTitle,
    DialogTrigger,
} from "~/components/ui/dialog";
import { PenBox, Eraser, Loader2, CheckCircle } from "lucide-react";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { generatePdfBlobFromHtml } from "~/lib/pdf-generator";
import { Editor, EditorContent, useEditor } from "@tiptap/react";
import StarterKit from "@tiptap/starter-kit";
import { Markdown } from "tiptap-markdown";
import { Table } from "@tiptap/extension-table";
import TableRow from "@tiptap/extension-table-row";
import TableCell from "@tiptap/extension-table-cell";
import TableHeader from "@tiptap/extension-table-header";
import { ScrollArea } from "~/components/ui/scroll-area";

interface DigitalSignatureModalProps {
    studyId: string;
    participantId: string;
    participantName?: string | null;
    participantCode: string;
    activeForm: { id: string; content: string; version: number };
    onSuccess: () => void;
}

export function DigitalSignatureModal({
    studyId,
    participantId,
    participantName,
    participantCode,
    activeForm,
    onSuccess,
}: DigitalSignatureModalProps) {
    const [isOpen, setIsOpen] = useState(false);
    const [isSubmitting, setIsSubmitting] = useState(false);
    const sigCanvas = useRef<any>(null);

    // Mutations
    const getUploadUrlMutation = api.participants.getConsentUploadUrl.useMutation();
    const recordConsentMutation = api.participants.recordConsent.useMutation();

    // Create a preview version of the text
    let previewMd = activeForm.content;
    previewMd = previewMd.replace(/{{PARTICIPANT_NAME}}/g, participantName ?? "_________________");
    previewMd = previewMd.replace(/{{PARTICIPANT_CODE}}/g, participantCode);
    const today = new Date().toLocaleDateString();
    previewMd = previewMd.replace(/{{DATE}}/g, today);
    previewMd = previewMd.replace(/{{SIGNATURE_IMAGE}}/g, "_[Signature Here]_");

    const previewEditor = useEditor({
        extensions: [StarterKit, Table, TableRow, TableHeader, TableCell, Markdown],
        content: previewMd,
        editable: false,
        immediatelyRender: false,
    });

    const handleClear = () => {
        sigCanvas.current?.clear();
    };

    const handleSubmit = async () => {
        if (sigCanvas.current?.isEmpty()) {
            toast.error("Signature required", { description: "Please sign the document before submitting." });
            return;
        }

        try {
            setIsSubmitting(true);
            toast.loading("Generating Signed Document...", { id: "sig-upload" });

            // 1. Get Signature Image Data URL
            const signatureDataUrl = sigCanvas.current.getTrimmedCanvas().toDataURL("image/png");

            // 2. Prepare final Markdown and HTML
            let finalMd = activeForm.content;
            finalMd = finalMd.replace(/{{PARTICIPANT_NAME}}/g, participantName ?? "_________________");
            finalMd = finalMd.replace(/{{PARTICIPANT_CODE}}/g, participantCode);
            finalMd = finalMd.replace(/{{DATE}}/g, today);
            finalMd = finalMd.replace(/{{SIGNATURE_IMAGE}}/g, `<img src="${signatureDataUrl}" style="height: 60px; max-width: 250px;" />`);

            const headlessEditor = new Editor({
                extensions: [StarterKit, Table, TableRow, TableHeader, TableCell, Markdown],
                content: finalMd,
            });
            const htmlContent = headlessEditor.getHTML();
            headlessEditor.destroy();

            // 3. Generate PDF Blob
            const filename = `Signed_Consent_${participantCode}_v${activeForm.version}.pdf`;
            const pdfBlob = await generatePdfBlobFromHtml(htmlContent, { filename });
            const file = new File([pdfBlob], filename, { type: "application/pdf" });

            // 4. Get Presigned URL
            toast.loading("Uploading Document...", { id: "sig-upload" });
            const { url, key } = await getUploadUrlMutation.mutateAsync({
                studyId,
                participantId,
                filename: file.name,
                contentType: file.type,
                size: file.size,
            });

            // 5. Upload to MinIO
            await new Promise<void>((resolve, reject) => {
                const xhr = new XMLHttpRequest();
                xhr.open("PUT", url, true);
                xhr.setRequestHeader("Content-Type", file.type);
                xhr.onload = () => {
                    if (xhr.status >= 200 && xhr.status < 300) resolve();
                    else reject(new Error(`Upload failed with status ${xhr.status}`));
                };
                xhr.onerror = () => reject(new Error("Network error during upload"));
                xhr.send(file);
            });

            // 6. Record Consent in DB
            toast.loading("Finalizing Consent...", { id: "sig-upload" });
            await recordConsentMutation.mutateAsync({
                participantId,
                consentFormId: activeForm.id,
                storagePath: key,
            });

            toast.success("Consent Successfully Recorded!", { id: "sig-upload" });
            setIsOpen(false);
            onSuccess();
        } catch (error) {
            console.error(error);
            toast.error("Failed to submit digital signature", {
                id: "sig-upload",
                description: error instanceof Error ? error.message : "Unknown error",
            });
        } finally {
            setIsSubmitting(false);
        }
    };

    return (
        <Dialog open={isOpen} onOpenChange={setIsOpen}>
            <DialogTrigger asChild>
                <Button variant="default" size="sm" className="bg-primary/90 hover:bg-primary">
                    <PenBox className="mr-2 h-4 w-4" />
                    Sign Digitally
                </Button>
            </DialogTrigger>
            <DialogContent className="max-w-4xl h-[90vh] flex flex-col p-6">
                <DialogHeader>
                    <DialogTitle>Digital Consent Signature</DialogTitle>
                    <DialogDescription>
                        Please review the document below and provide your digital signature to consent to this study.
                    </DialogDescription>
                </DialogHeader>

                <div className="flex-1 min-h-0 grid grid-cols-1 md:grid-cols-2 gap-6 mt-4">
                    {/* Document Preview (Left) */}
                    <div className="flex flex-col border rounded-md overflow-hidden bg-muted/20">
                        <div className="bg-muted px-4 py-2 border-b text-xs font-semibold text-muted-foreground uppercase tracking-wider">
                            Document Preview
                        </div>
                        <ScrollArea className="flex-1 w-full bg-white p-6 shadow-inner">
                            <div className="prose prose-sm max-w-none text-black">
                                <EditorContent editor={previewEditor} />
                            </div>
                        </ScrollArea>
                    </div>

                    {/* Signature Panel (Right) */}
                    <div className="flex flex-col space-y-4">
                        <div className="border rounded-md overflow-hidden bg-white shadow-sm flex flex-col">
                            <div className="bg-muted px-4 py-2 border-b text-xs font-semibold text-muted-foreground uppercase tracking-wider">
                                Digital Signature Pad
                            </div>
                            <div className="p-4 bg-muted/10 relative">
                                <div className="absolute top-4 right-4">
                                    <Button variant="ghost" size="sm" onClick={handleClear} disabled={isSubmitting}>
                                        <Eraser className="h-4 w-4 mr-2" />
                                        Clear
                                    </Button>
                                </div>
                                <div className="border-2 border-dashed border-input rounded-md bg-white mt-10" style={{ height: "250px" }}>
                                    <SignatureCanvas
                                        ref={sigCanvas}
                                        penColor="black"
                                        canvasProps={{ className: "w-full h-full cursor-crosshair rounded-md" }}
                                    />
                                </div>
                                <p className="text-center text-xs text-muted-foreground mt-2">
                                    Draw your signature using your mouse or touch screen inside the box above.
                                </p>
                            </div>
                        </div>

                        <div className="flex-1" />

                        {/* Submission Actions */}
                        <div className="flex flex-col space-y-3 p-4 bg-primary/5 rounded-lg border border-primary/20">
                            <h4 className="flex items-center text-sm font-semibold text-primary">
                                <CheckCircle className="h-4 w-4 mr-2" />
                                Agreement
                            </h4>
                            <p className="text-xs text-muted-foreground leading-relaxed">
                                By clicking "Submit Signed Document", you confirm that you have read and understood the information provided in the document preview, and you voluntarily agree to participate in this study.
                            </p>
                            <Button
                                className="w-full mt-2"
                                size="lg"
                                onClick={handleSubmit}
                                disabled={isSubmitting}
                            >
                                {isSubmitting ? (
                                    <>
                                        <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                                        Processing...
                                    </>
                                ) : (
                                    "Submit Signed Document"
                                )}
                            </Button>
                        </div>
                    </div>
                </div>
            </DialogContent>
        </Dialog>
    );
}
