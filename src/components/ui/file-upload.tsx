"use client";

import {
    AlertCircle, CheckCircle, File, FileAudio, FileImage,
    FileVideo, Loader2, Upload,
    X
} from "lucide-react";
import { useCallback, useRef, useState } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent } from "~/components/ui/card";
import { Progress } from "~/components/ui/progress";
import { cn } from "~/lib/utils";

interface FileUploadProps {
  accept?: string;
  multiple?: boolean;
  maxSize?: number; // in bytes
  maxFiles?: number;
  allowedTypes?: string[];
  category?: "video" | "audio" | "image" | "document" | "sensor_data";
  trialId?: string;
  onUploadComplete?: (files: UploadedFile[]) => void;
  onUploadError?: (error: string) => void;
  className?: string;
  disabled?: boolean;
}

interface UploadedFile {
  id: string;
  name: string;
  size: number;
  type: string;
  url: string;
  uploadedAt: string;
}

interface FileWithPreview extends File {
  preview?: string;
  progress?: number;
  error?: string;
  uploaded?: boolean;
  uploadedData?: UploadedFile;
}

export function FileUpload({
  accept,
  multiple = false,
  maxSize = 50 * 1024 * 1024, // 50MB default
  maxFiles = 5,
  allowedTypes = [],
  category = "document",
  trialId,
  onUploadComplete,
  onUploadError,
  className,
  disabled = false,
}: FileUploadProps) {
  const [files, setFiles] = useState<FileWithPreview[]>([]);
  const [isDragging, setIsDragging] = useState(false);
  const [isUploading, setIsUploading] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const validateFile = (file: File): string | null => {
    if (file.size > maxSize) {
      return `File size exceeds ${Math.round(maxSize / 1024 / 1024)}MB limit`;
    }

    if (allowedTypes.length > 0) {
      const extension = file.name.split('.').pop()?.toLowerCase() || '';
      if (!allowedTypes.includes(extension)) {
        return `File type .${extension} is not allowed`;
      }
    }

    return null;
  };

  const createFilePreview = (file: File): FileWithPreview => {
    const fileWithPreview = file as FileWithPreview;
    fileWithPreview.progress = 0;
    fileWithPreview.uploaded = false;

    // Create preview for images
    if (file.type.startsWith('image/')) {
      fileWithPreview.preview = URL.createObjectURL(file);
    }

    return fileWithPreview;
  };

  const handleFiles = useCallback((newFiles: FileList | File[]) => {
    const fileArray = Array.from(newFiles);

    // Check max files limit
    if (!multiple && fileArray.length > 1) {
      onUploadError?.("Only one file is allowed");
      return;
    }

    if (files.length + fileArray.length > maxFiles) {
      onUploadError?.(`Maximum ${maxFiles} files allowed`);
      return;
    }

    const validFiles: FileWithPreview[] = [];
    const errors: string[] = [];

    fileArray.forEach((file) => {
      const error = validateFile(file);
      if (error) {
        errors.push(`${file.name}: ${error}`);
      } else {
        validFiles.push(createFilePreview(file));
      }
    });

    if (errors.length > 0) {
      onUploadError?.(errors.join(', '));
      return;
    }

    setFiles((prev) => [...prev, ...validFiles]);
  }, [files.length, maxFiles, multiple, maxSize, allowedTypes, onUploadError]);

  const uploadFile = async (file: FileWithPreview): Promise<UploadedFile> => {
    const formData = new FormData();
    formData.append('file', file);
    formData.append('category', category);
    if (trialId) {
      formData.append('trialId', trialId);
    }

    const response = await fetch('/api/upload', {
      method: 'POST',
      body: formData,
    });

    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error || 'Upload failed');
    }

    const result = await response.json();
    return result.data;
  };

  const handleUpload = async () => {
    if (files.length === 0 || isUploading) return;

    setIsUploading(true);
    const uploadedFiles: UploadedFile[] = [];
    const errors: string[] = [];

    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      if (file?.uploaded) continue;

      try {
        // Update progress
        setFiles((prev) =>
          prev.map((f, index) =>
            index === i ? { ...f, progress: 0 } : f
          )
        );

        // Simulate progress (in real implementation, use XMLHttpRequest for progress)
        const progressInterval = setInterval(() => {
          setFiles((prev) =>
            prev.map((f, index) =>
              index === i ? { ...f, progress: Math.min((f.progress || 0) + 10, 90) } : f
            )
          );
        }, 100);

        const uploadedFile = await uploadFile(file!);

        clearInterval(progressInterval);

        // Mark as complete
        setFiles((prev) =>
          prev.map((f, index) =>
            index === i
              ? {
                  ...f,
                  progress: 100,
                  uploaded: true,
                  uploadedData: uploadedFile,
                }
              : f
          )
        );

        uploadedFiles.push(uploadedFile);
      } catch (_error) {
        const errorMessage = _error instanceof Error ? _error.message : 'Upload failed';
        errors.push(`${file?.name}: ${errorMessage}`);

        setFiles((prev) =>
          prev.map((f, index) =>
            index === i ? { ...f, error: errorMessage, progress: 0 } : f
          )
        );
      }
    }

    setIsUploading(false);

    if (errors.length > 0) {
      onUploadError?.(errors.join(', '));
    }

    if (uploadedFiles.length > 0) {
      onUploadComplete?.(uploadedFiles);
    }
  };

  const removeFile = (index: number) => {
    setFiles((prev) => {
      const newFiles = [...prev];
      const file = newFiles[index];
      if (file?.preview) {
        URL.revokeObjectURL(file.preview);
      }
      newFiles.splice(index, 1);
      return newFiles;
    });
  };

  const handleDrop = useCallback(
    (e: React.DragEvent<HTMLDivElement>) => {
      e.preventDefault();
      setIsDragging(false);

      if (disabled) return;

      const droppedFiles = e.dataTransfer.files;
      if (droppedFiles.length > 0) {
        handleFiles(droppedFiles);
      }
    },
    [handleFiles, disabled]
  );

  const handleDragOver = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    if (!disabled) {
      setIsDragging(true);
    }
  }, [disabled]);

  const handleDragLeave = useCallback((e: React.DragEvent<HTMLDivElement>) => {
    e.preventDefault();
    setIsDragging(false);
  }, []);

  const handleFileInputChange = useCallback(
    (e: React.ChangeEvent<HTMLInputElement>) => {
      const selectedFiles = e.target.files;
      if (selectedFiles && selectedFiles.length > 0) {
        handleFiles(selectedFiles);
      }
      // Reset input value to allow selecting the same file again
      e.target.value = '';
    },
    [handleFiles]
  );

  const getFileIcon = (file: File) => {
    if (file.type.startsWith('image/')) return FileImage;
    if (file.type.startsWith('video/')) return FileVideo;
    if (file.type.startsWith('audio/')) return FileAudio;
    return File;
  };

  const formatFileSize = (bytes: number) => {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
  };

  return (
    <div className={cn("space-y-4", className)}>
      {/* Upload Area */}
      <Card
        className={cn(
          "border-2 border-dashed transition-colors cursor-pointer",
          isDragging
            ? "border-blue-500 bg-blue-50"
            : "border-slate-300 hover:border-slate-400",
          disabled && "opacity-50 cursor-not-allowed"
        )}
        onDrop={handleDrop}
        onDragOver={handleDragOver}
        onDragLeave={handleDragLeave}
        onClick={() => !disabled && fileInputRef.current?.click()}
      >
        <CardContent className="flex flex-col items-center justify-center py-12 text-center">
          <Upload className={cn(
            "h-12 w-12 mb-4",
            isDragging ? "text-blue-500" : "text-slate-400"
          )} />
          <div className="space-y-2">
            <p className="text-lg font-medium">
              {isDragging ? "Drop files here" : "Upload files"}
            </p>
            <p className="text-sm text-slate-600">
              Drag and drop files here, or click to select
            </p>
            <div className="flex flex-wrap justify-center gap-2 text-xs text-slate-500">
              {allowedTypes.length > 0 && (
                <span>Allowed: {allowedTypes.join(', ')}</span>
              )}
              <span>Max size: {Math.round(maxSize / 1024 / 1024)}MB</span>
              {multiple && <span>Max files: {maxFiles}</span>}
            </div>
          </div>
        </CardContent>
      </Card>

      <input
        ref={fileInputRef}
        type="file"
        accept={accept}
        multiple={multiple}
        onChange={handleFileInputChange}
        className="hidden"
        disabled={disabled}
      />

      {/* File List */}
      {files.length > 0 && (
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <h4 className="font-medium">Selected Files ({files.length})</h4>
            <div className="flex space-x-2">
              <Button
                size="sm"
                onClick={handleUpload}
                disabled={isUploading || files.every(f => f.uploaded)}
              >
                {isUploading ? (
                  <>
                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                    Uploading...
                  </>
                ) : (
                  "Upload All"
                )}
              </Button>
              <Button
                variant="outline"
                size="sm"
                onClick={() => setFiles([])}
                disabled={isUploading}
              >
                Clear All
              </Button>
            </div>
          </div>

          <div className="space-y-2">
            {files.map((file, index) => {
              const FileIcon = getFileIcon(file);
              return (
                <Card key={index} className="p-3">
                  <div className="flex items-center space-x-3">
                    {file.preview ? (
                      <img
                        src={file.preview}
                        alt={file.name}
                        className="h-10 w-10 rounded object-cover"
                      />
                    ) : (
                      <div className="flex h-10 w-10 items-center justify-center rounded bg-slate-100">
                        <FileIcon className="h-5 w-5 text-slate-600" />
                      </div>
                    )}

                    <div className="flex-1 min-w-0">
                      <p className="font-medium truncate">{file.name}</p>
                      <p className="text-sm text-slate-600">
                        {formatFileSize(file.size)}
                      </p>

                      {file.progress !== undefined && file.progress > 0 && (
                        <Progress value={file.progress} className="mt-1 h-1" />
                      )}

                      {file.error && (
                        <Alert className="mt-2">
                          <AlertCircle className="h-4 w-4" />
                          <AlertDescription className="text-sm">
                            {file.error}
                          </AlertDescription>
                        </Alert>
                      )}
                    </div>

                    <div className="flex items-center space-x-2">
                      {file.uploaded ? (
                        <Badge className="bg-green-100 text-green-800">
                          <CheckCircle className="mr-1 h-3 w-3" />
                          Uploaded
                        </Badge>
                      ) : file.error ? (
                        <Badge variant="destructive">
                          <AlertCircle className="mr-1 h-3 w-3" />
                          Error
                        </Badge>
                      ) : file.progress !== undefined && file.progress > 0 ? (
                        <Badge className="bg-blue-100 text-blue-800">
                          <Loader2 className="mr-1 h-3 w-3 animate-spin" />
                          {file.progress}%
                        </Badge>
                      ) : (
                        <Badge variant="outline">Pending</Badge>
                      )}

                      <Button
                        variant="ghost"
                        size="sm"
                        onClick={() => removeFile(index)}
                        disabled={isUploading}
                      >
                        <X className="h-4 w-4" />
                      </Button>
                    </div>
                  </div>
                </Card>
              );
            })}
          </div>
        </div>
      )}
    </div>
  );
}
