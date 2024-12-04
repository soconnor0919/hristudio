'use client';

import { useEffect, useState } from "react";
import { useParams, useRouter } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { useToast } from "~/hooks/use-toast";
import { ArrowLeft } from "lucide-react";
import Link from "next/link";
import { useActiveStudy } from "~/context/active-study";
import { hasPermission } from "~/lib/permissions-client";
import { PERMISSIONS } from "~/lib/permissions";

export default function NewParticipant() {
  const [name, setName] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);
  const { id } = useParams();
  const router = useRouter();
  const { toast } = useToast();
  const { activeStudy } = useActiveStudy();

  useEffect(() => {
    if (!activeStudy || !hasPermission(activeStudy.permissions, PERMISSIONS.CREATE_PARTICIPANT)) {
      router.push(`/dashboard/studies/${id}`);
    }
  }, [activeStudy, id, router]);

  if (!activeStudy || !hasPermission(activeStudy.permissions, PERMISSIONS.CREATE_PARTICIPANT)) {
    return null;
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);

    try {
      const response = await fetch(`/api/studies/${id}/participants`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ name }),
      });

      if (!response.ok) {
        throw new Error("Failed to create participant");
      }

      toast({
        title: "Success",
        description: "Participant created successfully",
      });

      router.push(`/dashboard/studies/${id}/participants`);
    } catch (error) {
      console.error("Error creating participant:", error);
      toast({
        title: "Error",
        description: "Failed to create participant",
        variant: "destructive",
      });
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <Button
          variant="ghost"
          className="gap-2"
          asChild
        >
          <Link href={`/dashboard/studies/${id}/participants`}>
            <ArrowLeft className="h-4 w-4" />
            Back to Participants
          </Link>
        </Button>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Add New Participant</CardTitle>
          <CardDescription>
            Create a new participant for {activeStudy?.title}
          </CardDescription>
        </CardHeader>
        <CardContent>
          <form onSubmit={handleSubmit} className="space-y-4">
            <div className="space-y-2">
              <Label htmlFor="name">Participant Name</Label>
              <Input
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="Enter participant name"
                required
              />
            </div>
            <Button type="submit" disabled={isSubmitting}>
              {isSubmitting ? "Creating..." : "Create Participant"}
            </Button>
          </form>
        </CardContent>
      </Card>
    </div>
  );
} 