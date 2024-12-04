'use client';

import { useState, useEffect } from "react";
import { PlusIcon, Trash2Icon } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { useToast } from "~/hooks/use-toast";
import { PERMISSIONS } from "~/lib/permissions-client";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
  AlertDialogFooter
} from "~/components/ui/alert-dialog";

interface Participant {
  id: number;
  name: string;
  studyId: number;
  createdAt: string;
}

interface ParticipantsTabProps {
  studyId: number;
  permissions: string[];
}

export function ParticipantsTab({ studyId, permissions }: ParticipantsTabProps) {
  const [participants, setParticipants] = useState<Participant[]>([]);
  const [name, setName] = useState("");
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { toast } = useToast();

  useEffect(() => {
    fetchParticipants();
  }, [studyId]);

  const fetchParticipants = async () => {
    try {
      const response = await fetch(`/api/studies/${studyId}/participants`);
      if (!response.ok) throw new Error("Failed to fetch participants");
      const data = await response.json();
      setParticipants(data.data || []);
    } catch (error) {
      console.error("Error fetching participants:", error);
      toast({
        title: "Error",
        description: "Failed to load participants",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
    }
  };

  const createParticipant = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const response = await fetch(`/api/studies/${studyId}/participants`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ name }),
      });

      if (!response.ok) {
        throw new Error("Failed to create participant");
      }

      const data = await response.json();
      setParticipants([...participants, data.data]);
      setName("");
      toast({
        title: "Success",
        description: "Participant created successfully",
      });
    } catch (error) {
      console.error("Error creating participant:", error);
      toast({
        title: "Error",
        description: "Failed to create participant",
        variant: "destructive",
      });
    }
  };

  const deleteParticipant = async (participantId: number) => {
    try {
      const response = await fetch(`/api/studies/${studyId}/participants`, {
        method: "DELETE",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ participantId }),
      });

      if (!response.ok) {
        throw new Error("Failed to delete participant");
      }

      setParticipants(participants.filter(p => p.id !== participantId));
      toast({
        title: "Success",
        description: "Participant deleted successfully",
      });
    } catch (error) {
      console.error("Error deleting participant:", error);
      toast({
        title: "Error",
        description: "Failed to delete participant",
        variant: "destructive",
      });
    }
  };

  const hasPermission = (permission: string) => permissions.includes(permission);

  if (isLoading) {
    return (
      <Card>
        <CardContent className="py-8">
          <p className="text-center text-muted-foreground">Loading participants...</p>
        </CardContent>
      </Card>
    );
  }

  if (error) {
    return (
      <Card>
        <CardContent className="py-8">
          <p className="text-center text-destructive">{error}</p>
        </CardContent>
      </Card>
    );
  }

  return (
    <div className="space-y-6">
      {hasPermission(PERMISSIONS.CREATE_PARTICIPANT) && (
        <Card>
          <CardHeader>
            <CardTitle>Add New Participant</CardTitle>
            <CardDescription>Add a new participant to this study</CardDescription>
          </CardHeader>
          <CardContent>
            <form onSubmit={createParticipant} className="space-y-4">
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
              <Button type="submit">
                <PlusIcon className="w-4 h-4 mr-2" />
                Add Participant
              </Button>
            </form>
          </CardContent>
        </Card>
      )}

      <div className="grid gap-4">
        {participants.length > 0 ? (
          participants.map((participant) => (
            <Card key={participant.id}>
              <div className="p-6">
                <div className="flex items-center justify-between">
                  <div>
                    <h3 className="font-semibold">
                      {participant.name}
                    </h3>
                    <p className="text-sm text-muted-foreground">
                      Added {new Date(participant.createdAt).toLocaleDateString()}
                    </p>
                  </div>
                  {hasPermission(PERMISSIONS.DELETE_PARTICIPANT) && (
                    <AlertDialog>
                      <AlertDialogTrigger asChild>
                        <Button variant="outline" size="sm">
                          <Trash2Icon className="w-4 h-4 mr-2" />
                          Delete
                        </Button>
                      </AlertDialogTrigger>
                      <AlertDialogContent>
                        <AlertDialogHeader>
                          <AlertDialogTitle>Are you absolutely sure?</AlertDialogTitle>
                          <div className="text-sm text-muted-foreground">
                            This action cannot be undone. This will permanently delete the participant
                            &quot;{participant.name}&quot; and all associated data.
                          </div>
                        </AlertDialogHeader>
                        <AlertDialogFooter>
                          <AlertDialogCancel>Cancel</AlertDialogCancel>
                          <AlertDialogAction
                            onClick={() => deleteParticipant(participant.id)}
                            className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                          >
                            Delete Participant
                          </AlertDialogAction>
                        </AlertDialogFooter>
                      </AlertDialogContent>
                    </AlertDialog>
                  )}
                </div>
              </div>
            </Card>
          ))
        ) : (
          <Card>
            <CardContent className="py-8">
              <p className="text-center text-muted-foreground">
                No participants added yet. Add your first participant above.
              </p>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
} 