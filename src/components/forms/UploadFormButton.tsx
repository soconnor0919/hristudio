"use client";

import { useState } from "react";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { useToast } from "~/hooks/use-toast";
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { useStudyContext } from "~/context/StudyContext";

export function UploadFormButton() {
  const [isOpen, setIsOpen] = useState(false);
  const [file, setFile] = useState<File | null>(null);
  const [title, setTitle] = useState("");
  const [participantId, setParticipantId] = useState("");
  const { toast } = useToast();
  const { selectedStudy } = useStudyContext();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!file || !title || !participantId || !selectedStudy) {
      toast({
        title: "Error",
        description: "Please fill in all fields and select a file.",
        variant: "destructive",
      });
      return;
    }

    const formData = new FormData();
    formData.append("file", file);
    formData.append("title", title);
    formData.append("studyId", selectedStudy.id.toString());
    formData.append("participantId", participantId);

    try {
      const response = await fetch("/api/forms", {
        method: "POST",
        body: formData,
      });

      if (!response.ok) {
        throw new Error("Failed to upload form");
      }

      toast({
        title: "Success",
        description: "Form uploaded successfully",
      });
      setIsOpen(false);
      setFile(null);
      setTitle("");
      setParticipantId("");
    } catch (error) {
      console.error("Error uploading form:", error);
      toast({
        title: "Error",
        description: "Failed to upload form. Please try again.",
        variant: "destructive",
      });
    }
  };

  return (
    <Dialog open={isOpen} onOpenChange={setIsOpen}>
      <DialogTrigger asChild>
        <Button>Upload Form</Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Upload New Form</DialogTitle>
        </DialogHeader>
        <form onSubmit={handleSubmit} className="space-y-4">
          <div>
            <Label htmlFor="title">Title</Label>
            <Input
              id="title"
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              required
            />
          </div>
          <div>
            <Label htmlFor="participantId">Participant ID</Label>
            <Input
              id="participantId"
              value={participantId}
              onChange={(e) => setParticipantId(e.target.value)}
              required
            />
          </div>
          <div>
            <Label htmlFor="file">File</Label>
            <Input
              id="file"
              type="file"
              accept=".pdf"
              onChange={(e) => setFile(e.target.files?.[0] || null)}
              required
            />
          </div>
          <Button type="submit">Upload</Button>
        </form>
      </DialogContent>
    </Dialog>
  );
}