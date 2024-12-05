'use client';

import { useCallback, useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useToast } from "~/hooks/use-toast";
import { Plus, Trash2 } from "lucide-react";
import Link from "next/link";
import { useActiveStudy } from "~/context/active-study";
import { hasPermission } from "~/lib/permissions-client";
import { PERMISSIONS } from "~/lib/permissions";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "~/components/ui/alert-dialog";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
import { getApiUrl } from "~/lib/fetch-utils";
interface Participant {
  id: number;
  name: string;
  studyId: number;
  createdAt: string;
}

export default function ParticipantsList() {
  const [participants, setParticipants] = useState<Participant[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const { id } = useParams();
  const { toast } = useToast();
  const { activeStudy } = useActiveStudy();

  const canCreateParticipant = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.CREATE_PARTICIPANT);
  const canDeleteParticipant = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.DELETE_PARTICIPANT);
  const canViewNames = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.VIEW_PARTICIPANT_NAMES);

  const fetchParticipants = useCallback(async () => {
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}/participants`), {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
      });

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
  }, [toast, id]);

  useEffect(() => {
    fetchParticipants();
  }, [fetchParticipants]);

  const handleDelete = async (participantId: number) => {
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}/participants`), {
        method: "DELETE",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ participantId }),
      });

      if (!response.ok) throw new Error("Failed to delete participant");

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

  if (isLoading) {
    return (
      <Card>
        <CardContent className="py-8">
          <p className="text-center text-muted-foreground">Loading participants...</p>
        </CardContent>
      </Card>
    );
  }

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold tracking-tight">Participants</h2>
          <p className="text-muted-foreground">
            Manage study participants and their data
          </p>
        </div>
        {canCreateParticipant && (
          <Button asChild>
            <Link href={`/dashboard/studies/${id}/participants/new`}>
              <Plus className="mr-2 h-4 w-4" />
              Add Participant
            </Link>
          </Button>
        )}
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Study Participants</CardTitle>
          <CardDescription>
            All participants enrolled in {activeStudy?.title}
          </CardDescription>
        </CardHeader>
        <CardContent>
          {participants.length > 0 ? (
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>ID</TableHead>
                  <TableHead>Name</TableHead>
                  <TableHead>Added</TableHead>
                  {canDeleteParticipant && <TableHead className="w-[100px]">Actions</TableHead>}
                </TableRow>
              </TableHeader>
              <TableBody>
                {participants.map((participant) => (
                  <TableRow key={participant.id}>
                    <TableCell>{participant.id}</TableCell>
                    <TableCell>
                      {canViewNames ? participant.name : `Participant ${participant.id}`}
                    </TableCell>
                    <TableCell>
                      {new Date(participant.createdAt).toLocaleDateString()}
                    </TableCell>
                    {canDeleteParticipant && (
                      <TableCell>
                        <AlertDialog>
                          <AlertDialogTrigger asChild>
                            <Button variant="ghost" size="icon">
                              <Trash2 className="h-4 w-4" />
                            </Button>
                          </AlertDialogTrigger>
                          <AlertDialogContent>
                            <AlertDialogHeader>
                              <AlertDialogTitle>Delete Participant</AlertDialogTitle>
                              <AlertDialogDescription>
                                Are you sure you want to delete this participant? This action cannot be undone.
                              </AlertDialogDescription>
                            </AlertDialogHeader>
                            <AlertDialogFooter>
                              <AlertDialogCancel>Cancel</AlertDialogCancel>
                              <AlertDialogAction
                                onClick={() => handleDelete(participant.id)}
                                className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                              >
                                Delete
                              </AlertDialogAction>
                            </AlertDialogFooter>
                          </AlertDialogContent>
                        </AlertDialog>
                      </TableCell>
                    )}
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          ) : (
            <div className="py-8 text-center text-muted-foreground">
              No participants added yet
              {canCreateParticipant && (
                <>
                  .{" "}
                  <Link
                    href={`/dashboard/studies/${id}/participants/new`}
                    className="font-medium text-primary hover:underline"
                  >
                    Add your first participant
                  </Link>
                </>
              )}
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
} 