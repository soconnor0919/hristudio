'use client';

import { useEffect, useState } from "react";
import { Button } from "~/components/ui/button";
import { Card, CardHeader, CardTitle, CardContent } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { PlusIcon, Trash2Icon } from "lucide-react";
import {
  Select,
  SelectTrigger,
  SelectValue,
  SelectContent,
  SelectItem,
  SelectLabel,
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
      const response = await fetch(`/api/participants?studyId=${studyId}`);
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
      <h1 className="text-3xl font-bold mb-4">Manage Participants</h1>
      <div className="mb-4">
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

      <Card>
        <CardHeader>
          <CardTitle>Add New Participant</CardTitle>
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

      <div className="mt-4">
        <h2 className="text-xl font-semibold">Participant List</h2>
        <ul>
          {participants.map((participant) => (
            <li key={participant.id} className="flex justify-between items-center">
              <span>{participant.name}</span>
              <Button onClick={() => deleteParticipant(participant.id)} variant="destructive">
                <Trash2Icon className="w-4 h-4" />
              </Button>
            </li>
          ))}
        </ul>
      </div>
    </div>
  );
}