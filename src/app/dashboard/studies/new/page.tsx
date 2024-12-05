'use client';

import { useState } from "react";
import { useRouter } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { useToast } from "~/hooks/use-toast";
import { ArrowLeft, Settings2Icon } from "lucide-react";
import Link from "next/link";
import { useActiveStudy } from "~/context/active-study";
import { getApiUrl } from "~/lib/fetch-utils";

export default function NewStudy() {
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const router = useRouter();
  const { toast } = useToast();
  const { refreshStudies } = useActiveStudy();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const response = await fetch(getApiUrl('/api/studies'), {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ title, description }),
      });

      if (!response.ok) {
        throw new Error("Failed to create study");
      }

      const data = await response.json();
      
      toast({
        title: "Success",
        description: "Study created successfully",
      });

      // Refresh studies list and redirect to the new study
      await refreshStudies();
      router.push(`/dashboard/studies/${data.data.id}`);
    } catch (error) {
      console.error("Error creating study:", error);
      toast({
        title: "Error",
        description: "Failed to create study",
        variant: "destructive",
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold tracking-tight">Create New Study</h2>
          <p className="text-muted-foreground">
            Set up a new research study and configure its settings
          </p>
        </div>
        <Button
          variant="outline"
          className="gap-2"
          asChild
        >
          <Link href="/dashboard">
            <ArrowLeft className="h-4 w-4" />
            Back to Dashboard
          </Link>
        </Button>
      </div>

      <div className="flex gap-6">
        <div className="w-48 flex flex-col gap-2">
          <Button
            variant="secondary"
            className="justify-start"
          >
            <Settings2Icon className="mr-2 h-4 w-4" />
            Basic Settings
          </Button>
        </div>

        <div className="flex-1">
          <Card>
            <CardHeader>
              <CardTitle>Study Details</CardTitle>
              <CardDescription>
                Configure the basic settings for your new study
              </CardDescription>
            </CardHeader>
            <CardContent>
              <form onSubmit={handleSubmit} className="space-y-4">
                <div className="space-y-2">
                  <Label htmlFor="title">Study Title</Label>
                  <Input
                    id="title"
                    value={title}
                    onChange={(e) => setTitle(e.target.value)}
                    placeholder="Enter study title"
                    required
                  />
                </div>
                <div className="space-y-2">
                  <Label htmlFor="description">Description</Label>
                  <Textarea
                    id="description"
                    value={description}
                    onChange={(e) => setDescription(e.target.value)}
                    placeholder="Enter study description"
                    rows={4}
                  />
                </div>
                <div className="flex justify-end">
                  <Button type="submit" disabled={isSubmitting}>
                    {isSubmitting ? "Creating..." : "Create Study"}
                  </Button>
                </div>
              </form>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
} 