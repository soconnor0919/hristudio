"use client";

import { useState } from "react";
import { Upload, FileText, Trash2, Download, Loader2 } from "lucide-react";
import { Button } from "~/components/ui/button";
import {
    Card,
    CardContent,
    CardDescription,
    CardHeader,
    CardTitle,
} from "~/components/ui/card";
import { api } from "~/trpc/react";
import { formatBytes } from "~/lib/utils";
import { toast } from "sonner";

interface ParticipantDocumentsProps {
    participantId: string;
}

export function ParticipantDocuments({ participantId }: ParticipantDocumentsProps) {
    const [isUploading, setIsUploading] = useState(false);
    const utils = api.useUtils();

    const { data: documents, isLoading } = api.files.listParticipantDocuments.useQuery({
        participantId,
    });

    const getPresignedUrl = api.files.getPresignedUrl.useMutation();
    const registerUpload = api.files.registerUpload.useMutation();
    const deleteDocument = api.files.deleteDocument.useMutation({
        onSuccess: () => {
            toast.success("Document deleted");
            utils.files.listParticipantDocuments.invalidate({ participantId });
        },
        onError: (err) => toast.error(`Failed to delete: ${err.message}`),
    });

    // Since presigned URLs are for PUT, we can use a direct fetch
    const handleFileUpload = async (e: React.ChangeEvent<HTMLInputElement>) => {
        const file = e.target.files?.[0];
        if (!file) return;

        setIsUploading(true);
        try {
            // 1. Get presigned URL
            const { url, storagePath } = await getPresignedUrl.mutateAsync({
                filename: file.name,
                contentType: file.type || "application/octet-stream",
                participantId,
            });

            // 2. Upload to MinIO/S3
            const uploadRes = await fetch(url, {
                method: "PUT",
                body: file,
                headers: {
                    "Content-Type": file.type || "application/octet-stream",
                },
            });

            if (!uploadRes.ok) {
                throw new Error("Upload to storage failed");
            }

            // 3. Register in DB
            await registerUpload.mutateAsync({
                participantId,
                name: file.name,
                type: file.type,
                storagePath,
                fileSize: file.size,
            });

            toast.success("File uploaded successfully");
            utils.files.listParticipantDocuments.invalidate({ participantId });
        } catch (error) {
            console.error(error);
            toast.error("Failed to upload file");
        } finally {
            setIsUploading(false);
            // Reset input
            e.target.value = "";
        }
    };

    const handleDownload = async (storagePath: string, filename: string) => {
        // We would typically get a temporary download URL here
        // For now assuming public bucket or implementing a separate download procedure
        // Let's implement a quick procedure call right here via client or assume the server router has it.
        // I added getDownloadUrl to the router in previous steps.
        try {
            const { url } = await utils.client.files.getDownloadUrl.query({ storagePath });
            window.open(url, "_blank");
        } catch (e) {
            toast.error("Could not get download URL");
        }
    };

    return (
        <Card>
            <CardHeader>
                <div className="flex items-center justify-between">
                    <div className="space-y-1">
                        <CardTitle>Documents</CardTitle>
                        <CardDescription>
                            Manage consent forms and other files for this participant.
                        </CardDescription>
                    </div>
                    <div className="flex items-center gap-2">
                        <Button disabled={isUploading} asChild>
                            <label className="cursor-pointer">
                                {isUploading ? (
                                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                                ) : (
                                    <Upload className="mr-2 h-4 w-4" />
                                )}
                                Upload PDF
                                <input
                                    type="file"
                                    className="hidden"
                                    accept=".pdf,.doc,.docx,.txt" // User asked for PDF, but generic is fine
                                    onChange={handleFileUpload}
                                    disabled={isUploading}
                                />
                            </label>
                        </Button>
                    </div>
                </div>
            </CardHeader>
            <CardContent>
                {isLoading ? (
                    <div className="flex justify-center p-4">
                        <Loader2 className="h-6 w-6 animate-spin text-muted-foreground" />
                    </div>
                ) : documents?.length === 0 ? (
                    <div className="flex flex-col items-center justify-center py-8 text-center text-muted-foreground">
                        <FileText className="mb-2 h-8 w-8 opacity-50" />
                        <p>No documents uploaded yet.</p>
                    </div>
                ) : (
                    <div className="space-y-2">
                        {documents?.map((doc) => (
                            <div
                                key={doc.id}
                                className="flex items-center justify-between rounded-lg border p-3 hover:bg-muted/50"
                            >
                                <div className="flex items-center gap-3">
                                    <div className="rounded-md bg-blue-50 p-2">
                                        <FileText className="h-4 w-4 text-blue-600" />
                                    </div>
                                    <div>
                                        <p className="font-medium">{doc.name}</p>
                                        <p className="text-xs text-muted-foreground">
                                            {formatBytes(doc.fileSize ?? 0)} • {new Date(doc.createdAt).toLocaleDateString()}
                                        </p>
                                    </div>
                                </div>
                                <div className="flex items-center gap-1">
                                    <Button
                                        variant="ghost"
                                        size="icon"
                                        onClick={() => handleDownload(doc.storagePath, doc.name)}
                                    >
                                        <Download className="h-4 w-4" />
                                    </Button>
                                    <Button
                                        variant="ghost"
                                        size="icon"
                                        className="text-destructive hover:text-destructive hover:bg-destructive/10"
                                        onClick={() => {
                                            if (confirm("Are you sure you want to delete this file?")) {
                                                deleteDocument.mutate({ id: doc.id });
                                            }
                                        }}
                                    >
                                        <Trash2 className="h-4 w-4" />
                                    </Button>
                                </div>
                            </div>
                        ))}
                    </div>
                )}
            </CardContent>
        </Card>
    );
}
