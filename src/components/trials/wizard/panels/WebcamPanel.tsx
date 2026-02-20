"use client";

import React, { useCallback, useRef, useState } from "react";
import Webcam from "react-webcam";
import { Camera, CameraOff, Video, StopCircle, Loader2 } from "lucide-react";
import { Button } from "~/components/ui/button";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
} from "~/components/ui/select";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { AspectRatio } from "~/components/ui/aspect-ratio";
import { toast } from "sonner";
import { api } from "~/trpc/react";

export function WebcamPanel({ readOnly = false, trialId, trialStatus }: { readOnly?: boolean; trialId?: string; trialStatus?: string }) {
    const [deviceId, setDeviceId] = useState<string | null>(null);
    const [devices, setDevices] = useState<MediaDeviceInfo[]>([]);
    const [isCameraEnabled, setIsCameraEnabled] = useState(false);
    const [isRecording, setIsRecording] = useState(false);
    const [uploading, setUploading] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const webcamRef = useRef<Webcam>(null);
    const mediaRecorderRef = useRef<MediaRecorder | null>(null);
    const chunksRef = useRef<Blob[]>([]);

    // TRPC mutation for presigned URL
    const getUploadUrlMutation = api.storage.getUploadPresignedUrl.useMutation();

    // Mutation to save recording metadata to DB
    const saveRecordingMutation = api.storage.saveRecording.useMutation();
    const logEventMutation = api.trials.logEvent.useMutation();

    const handleDevices = useCallback(
        (mediaDevices: MediaDeviceInfo[]) => {
            setDevices(mediaDevices.filter(({ kind, deviceId }) => kind === "videoinput" && deviceId !== ""));
        },
        [setDevices],
    );

    const [isMounted, setIsMounted] = useState(false);

    React.useEffect(() => {
        setIsMounted(true);
        navigator.mediaDevices.enumerateDevices().then(handleDevices);
    }, [handleDevices]);

    const handleEnableCamera = () => {
        setIsCameraEnabled(true);
        setError(null);
    };

    const handleDisableCamera = () => {
        if (isRecording) {
            handleStopRecording();
        }
        setIsCameraEnabled(false);
    };

    // Auto-record based on trial status
    React.useEffect(() => {
        if (!trialStatus || readOnly) return;

        if (trialStatus === "in_progress") {
            if (!isCameraEnabled) {
                console.log("Auto-enabling camera for trial start");
                setIsCameraEnabled(true);
            } else if (!isRecording && webcamRef.current?.stream) {
                handleStartRecording();
            }
        } else if (trialStatus === "completed" && isRecording) {
            handleStopRecording();
        }
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, [trialStatus, isCameraEnabled, isRecording, readOnly]);

    const handleUserMedia = () => {
        if (trialStatus === "in_progress" && !isRecording && !readOnly) {
            console.log("Stream ready, auto-starting camera recording");
            handleStartRecording();
        }
    };

    const handleStartRecording = () => {
        if (!webcamRef.current?.stream) return;

        setIsRecording(true);
        chunksRef.current = [];

        try {
            const recorder = new MediaRecorder(webcamRef.current.stream, {
                mimeType: "video/webm"
            });

            recorder.ondataavailable = (event) => {
                if (event.data.size > 0) {
                    chunksRef.current.push(event.data);
                }
            };

            recorder.onstop = async () => {
                const blob = new Blob(chunksRef.current, { type: "video/webm" });
                await handleUpload(blob);
            };

            recorder.start();
            mediaRecorderRef.current = recorder;
            if (trialId) {
                logEventMutation.mutate({
                    trialId,
                    type: "camera_started",
                    data: { action: "recording_started" }
                });
            }
            toast.success("Recording started");
        } catch (e) {
            console.error("Failed to start recorder:", e);
            toast.error("Failed to start recording");
            setIsRecording(false);
        }
    };

    const handleStopRecording = () => {
        if (mediaRecorderRef.current && isRecording) {
            mediaRecorderRef.current.stop();
            setIsRecording(false);
            if (trialId) {
                logEventMutation.mutate({
                    trialId,
                    type: "camera_stopped",
                    data: { action: "recording_stopped" }
                });
            }
        }
    };

    const handleUpload = async (blob: Blob) => {
        setUploading(true);
        const filename = `recording-${Date.now()}.webm`;

        try {
            // 1. Get Presigned URL
            const { url } = await getUploadUrlMutation.mutateAsync({
                filename,
                contentType: "video/webm",
            });

            // 2. Upload to S3
            const response = await fetch(url, {
                method: "PUT",
                body: blob,
                headers: {
                    "Content-Type": "video/webm",
                },
            });

            if (!response.ok) {
                const errorText = await response.text();
                throw new Error(`Upload failed: ${errorText} | Status: ${response.status}`);
            }

            // 3. Save metadata to DB
            if (trialId) {
                console.log("Attempting to link recording to trial:", trialId);
                try {
                    await saveRecordingMutation.mutateAsync({
                        trialId,
                        storagePath: filename,
                        mediaType: "video",
                        format: "webm",
                        fileSize: blob.size,
                    });
                    console.log("Recording successfully linked to trial:", trialId);
                    toast.success("Recording saved to trial log");
                } catch (mutationError) {
                    console.error("Failed to link recording to trial:", mutationError);
                    toast.error("Video uploaded but failed to link to trial");
                }
            } else {
                console.warn("No trialId provided, recording uploaded but not linked. Props:", { trialId });
                toast.warning("Trial ID missing - recording not linked");
            }

            toast.success("Recording uploaded successfully");
            console.log("Uploaded recording:", filename);
        } catch (e) {
            console.error("Upload error:", e);
            toast.error("Failed to upload recording");
        } finally {
            setUploading(false);
        }
    };

    return (
        <div className="flex h-full flex-col">
            <div className="flex items-center justify-between border-b p-3">
                <h2 className="text-sm font-semibold flex items-center gap-2">
                    <Camera className="h-4 w-4" />
                    Webcam Feed
                </h2>

                {!readOnly && (
                    <div className="flex items-center gap-2">
                        {devices.length > 0 && isMounted && (
                            <Select
                                value={deviceId ?? undefined}
                                onValueChange={setDeviceId}
                                disabled={!isCameraEnabled || isRecording}
                            >
                                <SelectTrigger className="h-7 w-[130px] text-xs">
                                    <SelectValue placeholder="Select Camera" />
                                </SelectTrigger>
                                <SelectContent>
                                    {devices.map((device, key) => (
                                        <SelectItem key={key} value={device.deviceId} className="text-xs">
                                            {device.label || `Camera ${key + 1}`}
                                        </SelectItem>
                                    ))}
                                </SelectContent>
                            </Select>
                        )}

                        {isCameraEnabled && (
                            !isRecording ? (
                                <Button
                                    variant="destructive"
                                    size="sm"
                                    className="h-7 px-2 text-xs animate-in fade-in"
                                    onClick={handleStartRecording}
                                    disabled={uploading}
                                >
                                    <Video className="mr-1 h-3 w-3" />
                                    Record
                                </Button>
                            ) : (
                                <Button
                                    variant="secondary"
                                    size="sm"
                                    className="h-7 px-2 text-xs border-red-500 border text-red-500 hover:bg-red-50"
                                    onClick={handleStopRecording}
                                >
                                    <StopCircle className="mr-1 h-3 w-3 animate-pulse" />
                                    Stop Rec
                                </Button>
                            )
                        )}

                        {isCameraEnabled ? (
                            <Button
                                variant="ghost"
                                size="sm"
                                className="h-7 px-2 text-xs text-muted-foreground hover:text-foreground"
                                onClick={handleDisableCamera}
                                disabled={isRecording}
                            >
                                <CameraOff className="mr-1 h-3 w-3" />
                                Off
                            </Button>
                        ) : (
                            <Button
                                variant="default"
                                size="sm"
                                className="h-7 px-2 text-xs"
                                onClick={handleEnableCamera}
                            >
                                <Camera className="mr-1 h-3 w-3" />
                                Start Camera
                            </Button>
                        )}
                    </div>
                )}
            </div>

            <div className="flex-1 overflow-hidden bg-muted/50 p-4 flex items-center justify-center relative">
                {isCameraEnabled ? (
                    <div className="w-full relative rounded-lg overflow-hidden border border-border shadow-sm bg-black">
                        <AspectRatio ratio={16 / 9}>
                            <Webcam
                                ref={webcamRef}
                                audio={false}
                                width="100%"
                                height="100%"
                                videoConstraints={{ deviceId: deviceId ?? undefined }}
                                onUserMedia={handleUserMedia}
                                onUserMediaError={(err) => setError(String(err))}
                                className="object-contain w-full h-full"
                            />
                        </AspectRatio>

                        {/* Recording Overlay */}
                        {isRecording && (
                            <div className="absolute top-2 right-2 flex items-center gap-2 bg-black/50 px-2 py-1 rounded-full backdrop-blur-sm">
                                <div className="w-2 h-2 rounded-full bg-red-500 animate-pulse" />
                                <span className="text-[10px] font-medium text-white">REC</span>
                            </div>
                        )}

                        {/* Uploading Overlay */}
                        {uploading && (
                            <div className="absolute inset-0 flex items-center justify-center bg-black/60 backdrop-blur-sm">
                                <div className="flex flex-col items-center gap-2 text-white">
                                    <Loader2 className="h-6 w-6 animate-spin" />
                                    <span className="text-xs font-medium">Uploading...</span>
                                </div>
                            </div>
                        )}

                        {error && (
                            <div className="absolute inset-0 flex items-center justify-center bg-black/80">
                                <Alert variant="destructive" className="max-w-xs">
                                    <AlertDescription>{error}</AlertDescription>
                                </Alert>
                            </div>
                        )}
                    </div>
                ) : (
                    <div className="text-center text-muted-foreground/50">
                        <div className="mx-auto mb-2 flex h-12 w-12 items-center justify-center rounded-full bg-muted">
                            <CameraOff className="h-6 w-6 opacity-50" />
                        </div>
                        <p className="text-sm font-medium">Camera is disabled</p>
                        <Button
                            variant="secondary"
                            size="sm"
                            className="mt-4"
                            onClick={handleEnableCamera}
                        >
                            Enable Camera
                        </Button>
                    </div>
                )}
            </div>
        </div>
    );
}
