"use client";

import { useState } from "react";
import { Upload, X, FileText, CheckCircle, AlertCircle, Loader2 } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Progress } from "~/components/ui/progress";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { cn } from "~/lib/utils";

interface ConsentUploadFormProps {
    studyId: string;
    participantId: string;
    consentFormId: string;
    onSuccess: () => void;
    onCancel: () => void;
}

export function ConsentUploadForm({
    studyId,
    participantId,
    consentFormId,
    onSuccess,
    onCancel,
}: ConsentUploadFormProps) {
    const [file, setFile] = useState<File | null>(null);
    const [isUploading, setIsUploading] = useState(false);
    const [uploadProgress, setUploadProgress] = useState(0);

    // Mutations
    const getUploadUrlMutation = api.participants.getConsentUploadUrl.useMutation();
    const recordConsentMutation = api.participants.recordConsent.useMutation();

    const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
        if (e.target.files && e.target.files[0]) {
            const selectedFile = e.target.files[0];
            // Validate size (10MB)
            if (selectedFile.size > 10 * 1024 * 1024) {
                toast.error("File too large", {
                    description: "Maximum file size is 10MB",
                });
                return;
            }
            // Validate type
            const allowedTypes = ["application/pdf", "image/png", "image/jpeg", "image/jpg"];
            if (!allowedTypes.includes(selectedFile.type)) {
                toast.error("Invalid file type", {
                    description: "Please upload a PDF, PNG, or JPG file",
                });
                return;
            }
            setFile(selectedFile);
        }
    };

    const handleUpload = async () => {
        if (!file) return;

        try {
            setIsUploading(true);
            setUploadProgress(0);

            // 1. Get Presigned URL
            const { url, key } = await getUploadUrlMutation.mutateAsync({
                studyId,
                participantId,
                filename: file.name,
                contentType: file.type,
                size: file.size,
            });

            // 2. Upload to MinIO using XMLHttpRequest for progress
            await new Promise<void>((resolve, reject) => {
                const xhr = new XMLHttpRequest();
                xhr.open("PUT", url, true);
                xhr.setRequestHeader("Content-Type", file.type);

                xhr.upload.onprogress = (event) => {
                    if (event.lengthComputable) {
                        const percentCompleted = Math.round(
                            (event.loaded * 100) / event.total
                        );
                        setUploadProgress(percentCompleted);
                    }
                };

                xhr.onload = () => {
                    if (xhr.status >= 200 && xhr.status < 300) {
                        resolve();
                    } else {
                        reject(new Error(`Upload failed with status ${xhr.status}`));
                    }
                };

                xhr.onerror = () => reject(new Error("Network error during upload"));
                xhr.send(file);
            });

            // 3. Record Consent in DB
            await recordConsentMutation.mutateAsync({
                participantId,
                consentFormId,
                storagePath: key,
            });

            toast.success("Consent Recorded", {
                description: "The consent form has been uploaded and recorded successfully.",
            });

            onSuccess();
        } catch (error) {
            console.error("Upload failed:", error);
            toast.error("Upload Failed", {
                description: error instanceof Error ? error.message : "An unexpected error occurred",
            });
            setIsUploading(false);
        }
    };

    return (
        <div className="space-y-4">
            {!file ? (
                <div className="flex flex-col items-center justify-center border-2 border-dashed rounded-lg p-6 bg-muted/5 hover:bg-muted/10 transition-colors">
                    <Upload className="h-8 w-8 text-muted-foreground mb-4" />
                    <h3 className="font-semibold text-sm mb-1">Upload Signed Consent</h3>
                    <p className="text-xs text-muted-foreground mb-4 text-center">
                        Drag and drop or click to select<br />
                        PDF, PNG, JPG up to 10MB
                    </p>
                    <input
                        type="file"
                        id="consent-file-upload"
                        className="hidden"
                        accept=".pdf,.png,.jpg,.jpeg"
                        onChange={handleFileChange}
                    />
                    <Button variant="secondary" size="sm" onClick={() => document.getElementById("consent-file-upload")?.click()}>
                        Select File
                    </Button>
                </div>
            ) : (
                <div className="border rounded-lg p-4 bg-muted/5">
                    <div className="flex items-start justify-between mb-4">
                        <div className="flex items-center gap-3">
                            <div className="h-10 w-10 bg-primary/10 rounded flex items-center justify-center">
                                <FileText className="h-5 w-5 text-primary" />
                            </div>
                            <div>
                                <p className="text-sm font-medium line-clamp-1 break-all">{file.name}</p>
                                <p className="text-xs text-muted-foreground">
                                    {(file.size / 1024 / 1024).toFixed(2)} MB
                                </p>
                            </div>
                        </div>
                        {!isUploading && (
                            <Button variant="ghost" size="icon" className="h-6 w-6" onClick={() => setFile(null)}>
                                <X className="h-4 w-4" />
                            </Button>
                        )}
                    </div>

                    {isUploading && (
                        <div className="space-y-2 mb-4">
                            <div className="flex justify-between text-xs text-muted-foreground">
                                <span>Uploading...</span>
                                <span>{uploadProgress}%</span>
                            </div>
                            <Progress value={uploadProgress} className="h-2" />
                        </div>
                    )}

                    <div className="flex gap-2 justify-end">
                        <Button variant="outline" size="sm" onClick={onCancel} disabled={isUploading}>
                            Cancel
                        </Button>
                        <Button size="sm" onClick={handleUpload} disabled={isUploading}>
                            {isUploading ? (
                                <>
                                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                                    Uploading
                                </>
                            ) : (
                                <>
                                    <Upload className="mr-2 h-4 w-4" />
                                    Upload & Record
                                </>
                            )}
                        </Button>
                    </div>
                </div>
            )}
        </div>
    );
}
