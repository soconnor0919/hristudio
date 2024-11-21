'use client';

import { PlusIcon, Trash2Icon } from "lucide-react";
import { useEffect, useState } from "react";
import { Button } from "~/components/ui/button";
import { 
  Card, 
  CardContent, 
  CardHeader, 
  CardTitle, 
  CardDescription,
  CardFooter 
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue
} from "~/components/ui/select";

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
    } finally {
      setLoading(false);
    }
  };

  const fetchParticipants = async (studyId: number) => {
    try {
      console.log(`Fetching participants for studyId: ${studyId}`);
      const response = await fetch(`/api/participants?studyId=${studyId}`);
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      setParticipants(data);
    } catch (error) {
      console.error('Error fetching participants:', error);
    }
  };

  const handleStudyChange = (studyId: string) => {
    const id = parseInt(studyId); // Convert the string to a number
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

      if (response.ok) {
        const newParticipant = await response.json();
        setParticipants([...participants, newParticipant]);
        setParticipantName("");
      } else {
        console.error('Error adding participant:', response.statusText);
      }
    } catch (error) {
      console.error('Error adding participant:', error);
    }
  };

  const deleteParticipant = async (id: number) => {
    try {
      const response = await fetch(`/api/participants/${id}`, {
        method: 'DELETE',
      });

      if (response.ok) {
        setParticipants(participants.filter(participant => participant.id !== id));
      } else {
        console.error('Error deleting participant:', response.statusText);
      }
    } catch (error) {
      console.error('Error deleting participant:', error);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="max-w-4xl mx-auto">
      <div className="flex justify-between items-center mb-8">
        <h1 className="text-3xl font-bold">Participants</h1>
      </div>

      <Card className="mb-8">
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

      <Card className="mb-8">
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
                required
              />
            </div>
            <Button type="submit" disabled={!selectedStudyId}>
              <PlusIcon className="w-4 h-4 mr-2" />
              Add Participant
            </Button>
          </form>
        </CardContent>
      </Card>

      <div className="grid gap-4">
        {participants.map((participant) => (
          <Card key={participant.id}>
            <CardHeader>
              <div className="flex justify-between items-start">
                <div>
                  <CardTitle>{participant.name}</CardTitle>
                  <CardDescription className="mt-1.5">
                    Participant ID: {participant.id}
                  </CardDescription>
                </div>
                <Button 
                  variant="ghost" 
                  size="icon" 
                  className="text-destructive"
                  onClick={() => deleteParticipant(participant.id)}
                >
                  <Trash2Icon className="w-4 h-4" />
                </Button>
              </div>
            </CardHeader>
            <CardFooter className="text-sm text-muted-foreground">
              Study ID: {participant.studyId}
            </CardFooter>
          </Card>
        ))}
        {participants.length === 0 && selectedStudyId && (
          <Card>
            <CardContent className="py-8">
              <p className="text-center text-muted-foreground">
                No participants found for this study. Add one above to get started.
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
    </div>
  );
}