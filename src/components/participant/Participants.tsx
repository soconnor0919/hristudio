"use client";

import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useStudyContext } from '../../context/StudyContext';
import { Participant } from '../../types/Participant';
import { CreateParticipantDialog } from './CreateParticipantDialog';
import { useToast } from '~/hooks/use-toast';
import { ParticipantCard } from './ParticipantCard';
import { Avatar, AvatarFallback } from "~/components/ui/avatar";

interface ParticipantWithTrial {
  id: number;
  name: string;
  latestTrialTimestamp: string | null;
  createdAt: string;
}

export function Participants() {
  const [participants, setParticipants] = useState<ParticipantWithTrial[]>([]);
  const { selectedStudy } = useStudyContext();
  const { toast } = useToast();

  useEffect(() => {
    if (selectedStudy) {
      fetchParticipants();
    }
  }, [selectedStudy]);

  const fetchParticipants = async () => {
    if (!selectedStudy) return;
    try {
      const response = await fetch(`/api/participants?studyId=${selectedStudy.id}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const text = await response.text();
      try {
        const data = JSON.parse(text);
        setParticipants(data);
      } catch (e) {
        console.error('Failed to parse JSON:', text);
        throw new Error('Invalid JSON in response');
      }
    } catch (error) {
      console.error('Error fetching participants:', error);
      // Handle the error appropriately, e.g., show a toast notification
    }
  };

  const createParticipant = async (name: string) => {
    if (!selectedStudy) return;
    const response = await fetch('/api/participants', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, studyId: selectedStudy.id }),
    });
    const createdParticipant = await response.json();
    setParticipants([...participants, createdParticipant]);
  };

  const deleteParticipant = async (id: number) => {
    if (!selectedStudy) return;
    try {
      const response = await fetch(`/api/participants/${id}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete participant');
      }

      setParticipants(participants.filter(p => p.id !== id));
      toast({
        title: "Success",
        description: "Participant deleted successfully",
      });
    } catch (error) {
      toast({
        title: "Error",
        description: error instanceof Error ? error.message : 'Failed to delete participant',
        variant: "destructive",
      });
    }
  };

  if (!selectedStudy) {
    return <div>Please select a study to manage participants.</div>;
  }

  return (
    <Card className="card-level-1">
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-2xl font-bold">Participants for {selectedStudy.title}</CardTitle>
        <CreateParticipantDialog onCreateParticipant={createParticipant} />
      </CardHeader>
      <CardContent>
        {participants.length > 0 ? (
          <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
            {participants.map(participant => (
              <Card key={participant.id} className="card-level-2 p-3 flex items-center w-full">
                <Avatar className="mr-4">
                  <AvatarFallback>{participant.name.split(' ').map(n => n[0]).join('')}</AvatarFallback>
                </Avatar>
                <div className="flex-1">
                  <h3 className="font-semibold">{participant.name}</h3>
                  <p className="text-sm text-muted-foreground">
                    {participant.latestTrialTimestamp 
                      ? `Last trial: ${new Date(participant.latestTrialTimestamp).toLocaleString()}` 
                      : 'No trials yet'}
                  </p>
                </div>
                <Button
                  variant="ghost"
                  size="sm"
                  className="text-destructive"
                  onClick={() => deleteParticipant(participant.id)}
                >
                  Delete
                </Button>
              </Card>
            ))}
          </div>
        ) : (
          <p>No participants added yet.</p>
        )}
      </CardContent>
    </Card>
  );
}