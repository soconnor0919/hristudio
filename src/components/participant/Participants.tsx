"use client";

import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useStudyContext } from '../../context/StudyContext';
import { Participant } from '../../types/Participant';
import { CreateParticipantDialog } from './CreateParticipantDialog';
import { Trash2 } from 'lucide-react';
import { useToast } from '~/hooks/use-toast';

export function Participants() {
  const [participants, setParticipants] = useState<Participant[]>([]);
  const { selectedStudy } = useStudyContext();
  const { toast } = useToast();

  useEffect(() => {
    if (selectedStudy) {
      fetchParticipants();
    }
  }, [selectedStudy]);

  const fetchParticipants = async () => {
    if (!selectedStudy) return;
    const response = await fetch(`/api/participants?studyId=${selectedStudy.id}`);
    const data = await response.json();
    setParticipants(data);
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
      console.log(`Attempting to delete participant with ID: ${id}`);
      const response = await fetch(`/api/participants/${id}`, {
        method: 'DELETE',
      });
      console.log('Delete response:', response);

      const contentType = response.headers.get("content-type");
      if (contentType && contentType.indexOf("application/json") !== -1) {
        const result = await response.json();
        console.log('Delete result:', result);

        if (!response.ok) {
          throw new Error(result.error || `Failed to delete participant. Status: ${response.status}`);
        }

        setParticipants(participants.filter(p => p.id !== id));
        toast({
          title: "Success",
          description: "Participant deleted successfully",
        });
      } else {
        const text = await response.text();
        console.error('Unexpected response:', text);
        throw new Error(`Unexpected response from server. Status: ${response.status}`);
      }
    } catch (error) {
      console.error('Error deleting participant:', error);
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
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-2xl font-bold">Participants for {selectedStudy.title}</CardTitle>
        <CreateParticipantDialog onCreateParticipant={createParticipant} />
      </CardHeader>
      <CardContent>
        {participants.length > 0 ? (
          <ul className="space-y-2">
            {participants.map(participant => (
              <li key={participant.id} className="bg-gray-100 p-2 rounded flex justify-between items-center">
                <span>{participant.name}</span>
                <Button 
                  variant="ghost" 
                  size="sm" 
                  onClick={() => deleteParticipant(participant.id)}
                  className="text-red-500 hover:text-red-700"
                >
                  <Trash2 className="h-4 w-4" />
                </Button>
              </li>
            ))}
          </ul>
        ) : (
          <p>No participants added yet.</p>
        )}
      </CardContent>
    </Card>
  );
}