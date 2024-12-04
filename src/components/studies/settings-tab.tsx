'use client';

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { Button } from "~/components/ui/button";
import { useToast } from "~/hooks/use-toast";
import { useState } from "react";
import { PERMISSIONS } from "~/lib/permissions-client";

interface SettingsTabProps {
  study: {
    id: number;
    title: string;
    description: string | null;
    permissions: string[];
  };
}

export function SettingsTab({ study }: SettingsTabProps) {
  const [title, setTitle] = useState(study.title);
  const [description, setDescription] = useState(study.description || "");
  const { toast } = useToast();

  const hasPermission = (permission: string) => study.permissions.includes(permission);
  const canEditStudy = hasPermission(PERMISSIONS.EDIT_STUDY);

  const updateStudy = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const response = await fetch(`/api/studies/${study.id}`, {
        method: "PATCH",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ title, description }),
      });

      if (!response.ok) throw new Error("Failed to update study");

      toast({
        title: "Success",
        description: "Study updated successfully",
      });
    } catch (error) {
      console.error("Error updating study:", error);
      toast({
        title: "Error",
        description: "Failed to update study",
        variant: "destructive",
      });
    }
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Study Settings</CardTitle>
        <CardDescription>Update your study details and configuration</CardDescription>
      </CardHeader>
      <CardContent>
        <form onSubmit={updateStudy} className="space-y-4">
          <div className="space-y-2">
            <Label htmlFor="title">Study Title</Label>
            <Input
              id="title"
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              placeholder="Enter study title"
              required
              disabled={!canEditStudy}
            />
          </div>
          <div className="space-y-2">
            <Label htmlFor="description">Description</Label>
            <Textarea
              id="description"
              value={description}
              onChange={(e) => setDescription(e.target.value)}
              placeholder="Enter study description"
              disabled={!canEditStudy}
            />
          </div>
          {canEditStudy && (
            <Button type="submit">
              Save Changes
            </Button>
          )}
        </form>
      </CardContent>
    </Card>
  );
} 