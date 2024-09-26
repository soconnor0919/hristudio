import React, { useState, useEffect } from 'react';
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { useToast } from "~/hooks/use-toast";

interface UploadConsentFormProps {
  studyId: number;
  participantId: number;
}

export function UploadConsentForm({ studyId, participantId }: UploadConsentFormProps) {
  const [file, setFile] = useState<File | null>(null);
  const [isUploading, setIsUploading] = useState(false);
  const { toast } = useToast();

  useEffect(() => {
    toast({
      title: "Test Toast",
      description: "This is a test toast message",
    });
  }, []);

  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (e.target.files && e.target.files[0]) {
      setFile(e.target.files[0]);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!file) {
      toast({
        title: "Error",
        description: "Please select a file to upload",
        variant: "destructive",
      });
      return;
    }

    setIsUploading(true);

    const formData = new FormData();
    formData.append('file', file);
    formData.append('studyId', studyId.toString());
    formData.append('participantId', participantId.toString());

    try {
      const response = await fetch('/api/informed-consent', {
        method: 'POST',
        body: formData,
      });

      if (!response.ok) {
        throw new Error('Failed to upload form');
      }

      toast({
        title: "Success",
        description: "Informed consent form uploaded successfully",
      });
      setFile(null);
    } catch (error) {
      console.error('Error uploading form:', error);
      toast({
        title: "Error",
        description: "Failed to upload informed consent form",
        variant: "destructive",
      });
    } finally {
      setIsUploading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className="space-y-4">
      <div>
        <Input
          type="file"
          accept=".pdf"
          onChange={handleFileChange}
          disabled={isUploading}
        />
      </div>
      <Button type="submit" disabled={!file || isUploading}>
        {isUploading ? 'Uploading...' : 'Upload Consent Form'}
      </Button>
    </form>
  );
}