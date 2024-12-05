'use client';

import { useCallback, useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useToast } from "~/hooks/use-toast";
import { Plus, Trash2 } from "lucide-react";
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
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
import { getApiUrl } from "~/lib/fetch-utils";
import { Skeleton } from "~/components/ui/skeleton";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";

interface Participant {
  id: number;
  name: string;
  studyId: number;
  createdAt: string;
}

export default function ParticipantsList() {
  const [participants, setParticipants] = useState<Participant[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isAddingParticipant, setIsAddingParticipant] = useState(false);
  const [newParticipantName, setNewParticipantName] = useState("");
  const { id } = useParams();
  const { toast } = useToast();
  const { activeStudy } = useActiveStudy();

  const canCreateParticipant = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.CREATE_PARTICIPANT);
  const canDeleteParticipant = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.DELETE_PARTICIPANT);
  const canViewNames = activeStudy && hasPermission(activeStudy.permissions, PERMISSIONS.VIEW_PARTICIPANT_NAMES);

  const fetchParticipants = useCallback(async () => {
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}/participants`));
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
  }, [id, toast]);

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

  const handleAddParticipant = async () => {
    if (!newParticipantName.trim()) return;

    setIsAddingParticipant(true);
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}/participants`), {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ name: newParticipantName }),
      });

      if (!response.ok) throw new Error("Failed to add participant");

      const data = await response.json();
      setParticipants([...participants, data.data]);
      setNewParticipantName("");
      toast({
        title: "Success",
        description: "Participant added successfully",
      });
    } catch (error) {
      console.error("Error adding participant:", error);
      toast({
        title: "Error",
        description: "Failed to add participant",
        variant: "destructive",
      });
    } finally {
      setIsAddingParticipant(false);
    }
  };

  if (isLoading) {
    return (
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <div>
            <Skeleton className="h-8 w-[200px] mb-2" />
            <Skeleton className="h-4 w-[300px]" />
          </div>
          <Skeleton className="h-10 w-[140px]" />
        </div>

        <Card>
          <CardHeader>
            <Skeleton className="h-5 w-[150px] mb-2" />
            <Skeleton className="h-4 w-[250px]" />
          </CardHeader>
          <CardContent>
            <div className="rounded-md border">
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead><Skeleton className="h-4 w-[40px]" /></TableHead>
                    <TableHead><Skeleton className="h-4 w-[120px]" /></TableHead>
                    <TableHead><Skeleton className="h-4 w-[100px]" /></TableHead>
                    <TableHead className="w-[100px]"><Skeleton className="h-4 w-[60px]" /></TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {[1, 2, 3].map((i) => (
                    <TableRow key={i}>
                      <TableCell><Skeleton className="h-4 w-[30px]" /></TableCell>
                      <TableCell><Skeleton className="h-4 w-[150px]" /></TableCell>
                      <TableCell><Skeleton className="h-4 w-[100px]" /></TableCell>
                      <TableCell><Skeleton className="h-8 w-8" /></TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            </div>
          </CardContent>
        </Card>
      </div>
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
          <Dialog>
            <DialogTrigger asChild>
              <Button>
                <Plus className="mr-2 h-4 w-4" />
                Add Participant
              </Button>
            </DialogTrigger>
            <DialogContent>
              <DialogHeader>
                <DialogTitle>Add Participant</DialogTitle>
                <DialogDescription>
                  Add a new participant to {activeStudy?.title}
                </DialogDescription>
              </DialogHeader>
              <div className="space-y-4 py-4">
                <div className="space-y-2">
                  <Label htmlFor="name">Participant Name</Label>
                  <Input
                    id="name"
                    placeholder="Enter participant name"
                    value={newParticipantName}
                    onChange={(e) => setNewParticipantName(e.target.value)}
                  />
                </div>
              </div>
              <DialogFooter>
                <Button
                  onClick={handleAddParticipant}
                  disabled={isAddingParticipant || !newParticipantName.trim()}
                >
                  {isAddingParticipant ? "Adding..." : "Add Participant"}
                </Button>
              </DialogFooter>
            </DialogContent>
          </Dialog>
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
                  <Dialog>
                    <DialogTrigger asChild>
                      <Button variant="link" className="px-2 py-0">
                        Add your first participant
                      </Button>
                    </DialogTrigger>
                    <DialogContent>
                      <DialogHeader>
                        <DialogTitle>Add Participant</DialogTitle>
                        <DialogDescription>
                          Add a new participant to {activeStudy?.title}
                        </DialogDescription>
                      </DialogHeader>
                      <div className="space-y-4 py-4">
                        <div className="space-y-2">
                          <Label htmlFor="name-empty">Participant Name</Label>
                          <Input
                            id="name-empty"
                            placeholder="Enter participant name"
                            value={newParticipantName}
                            onChange={(e) => setNewParticipantName(e.target.value)}
                          />
                        </div>
                      </div>
                      <DialogFooter>
                        <Button
                          onClick={handleAddParticipant}
                          disabled={isAddingParticipant || !newParticipantName.trim()}
                        >
                          {isAddingParticipant ? "Adding..." : "Add Participant"}
                        </Button>
                      </DialogFooter>
                    </DialogContent>
                  </Dialog>
                </>
              )}
            </div>
          )}
        </CardContent>
      </Card>
    </div>
  );
} 