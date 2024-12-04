'use client';

import { PlusIcon, Trash2Icon } from "lucide-react";
import { useEffect, useState } from "react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "~/components/ui/select";
import { useToast } from "~/hooks/use-toast";

interface Study {
  id: number;
  title: string;
}

interface Participant {
  id: number;
  name: string;
  studyId: number;
}

export default function Participants() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [participants, setParticipants] = useState<Participant[]>([]);
  const [selectedStudyId, setSelectedStudyId] = useState<number | null>(null);
  const [participantName, setParticipantName] = useState("");
  const [loading, setLoading] = useState(true);
  const { toast } = useToast();

  useEffect(() => {
    fetchStudies();
  }, []);

  const fetchStudies = async () => {
    try {
      const response = await fetch('/api/studies');
      const data = await response.json();
      setStudies(data);
    } catch (error) {
      console.error('Error fetching studies:', error);
      toast({
        title: "Error",
        description: "Failed to load studies",
        variant: "destructive",
      });
    } finally {
      setLoading(false);
    }
  };

  const fetchParticipants = async (studyId: number) => {
    try {
      const response = await fetch(`/api/participants?studyId=${studyId}`);
      
      if (!response.ok) {
        throw new Error(`Failed to fetch participants`);
      }

      const data = await response.json();
      setParticipants(data);
    } catch (error) {
      console.error('Error fetching participants:', error);
      toast({
        title: "Error",
        description: "Failed to load participants",
        variant: "destructive",
      });
    }
  };

  const handleStudyChange = (studyId: string) => {
    const id = parseInt(studyId);
    setSelectedStudyId(id);
    fetchParticipants(id);
  };

  const addParticipant = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!selectedStudyId) return;

    try {
      const response = await fetch(`/api/participants`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          name: participantName,
          studyId: selectedStudyId,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to add participant');
      }

      const newParticipant = await response.json();
      setParticipants([...participants, newParticipant]);
      setParticipantName("");
      
      toast({
        title: "Success",
        description: "Participant added successfully",
      });
    } catch (error) {
      console.error('Error adding participant:', error);
      toast({
        title: "Error",
        description: "Failed to add participant",
        variant: "destructive",
      });
    }
  };

  const deleteParticipant = async (id: number) => {
    try {
      const response = await fetch(`/api/participants/${id}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete participant');
      }

      setParticipants(participants.filter(participant => participant.id !== id));
      toast({
        title: "Success",
        description: "Participant deleted successfully",
      });
    } catch (error) {
      console.error('Error deleting participant:', error);
      toast({
        title: "Error",
        description: "Failed to delete participant",
        variant: "destructive",
      });
    }
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="animate-spin h-8 w-8 border-4 border-primary border-t-transparent rounded-full" />
      </div>
    );
  }

  return (
    <div className="container py-6 space-y-6">
      <div>
        <h1 className="text-2xl font-bold">Participants</h1>
        <p className="text-muted-foreground">Manage study participants</p>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Study Selection</CardTitle>
          <CardDescription>
            Select a study to manage its participants
          </CardDescription>
        </CardHeader>
        <CardContent>
          <div className="space-y-2">
            <Label htmlFor="study">Select Study</Label>
            <Select onValueChange={handleStudyChange}>
              <SelectTrigger>
                <SelectValue placeholder="Select a study" />
              </SelectTrigger>
              <SelectContent>
                {studies.map((study) => (
                  <SelectItem key={study.id} value={study.id.toString()}>
                    {study.title}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>
        </CardContent>
      </Card>

      {selectedStudyId && (
        <Card>
          <CardHeader>
            <CardTitle>Add New Participant</CardTitle>
            <CardDescription>
              Add a new participant to the selected study
            </CardDescription>
          </CardHeader>
          <CardContent>
            <form onSubmit={addParticipant} className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="name">Participant Name</Label>
                <Input
                  type="text"
                  id="name"
                  value={participantName}
                  onChange={(e) => setParticipantName(e.target.value)}
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

      {selectedStudyId && participants.length > 0 && (
        <Card>
          <CardHeader>
            <CardTitle>Participants List</CardTitle>
            <CardDescription>
              Manage existing participants
            </CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              {participants.map((participant) => (
                <div
                  key={participant.id}
                  className="flex items-center justify-between p-4 border rounded-lg bg-card"
                >
                  <span className="font-medium">{participant.name}</span>
                  <Button
                    variant="outline"
                    size="sm"
                    onClick={() => deleteParticipant(participant.id)}
                  >
                    <Trash2Icon className="w-4 h-4 mr-2" />
                    Delete
                  </Button>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>
      )}

      {selectedStudyId && participants.length === 0 && (
        <Card>
          <CardContent className="py-8">
            <p className="text-center text-muted-foreground">
              No participants added yet. Add your first participant above.
            </p>
          </CardContent>
        </Card>
      )}

      {!selectedStudyId && (
        <Card>
          <CardContent className="py-8">
            <p className="text-center text-muted-foreground">
              Please select a study to view its participants.
            </p>
          </CardContent>
        </Card>
      )}
    </div>
  );
}