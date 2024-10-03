"use client";

import React, { useState, useEffect } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useToast } from '~/hooks/use-toast';
import { CreateTrialDialog } from '~/components/trial/CreateTrialDialog';

interface Trial {
  id: number;
  title: string;
  participantIds: number[];
  createdAt: string;
}

export function Trials() {
  const [trials, setTrials] = useState<Trial[]>([]);
  const { toast } = useToast();

  useEffect(() => {
    fetchTrials();
  }, []);

  const fetchTrials = async () => {
    const response = await fetch('/api/trials');

    if (!response.ok) {
      const errorText = await response.text();
      console.error('Error fetching trials:', response.status, errorText);
      return;
    }

    const data = await response.json();
    if (!data || data.length === 0) {
      console.warn('No trials found');
      setTrials([]); // Set to an empty array if no trials are found
      return;
    }

    setTrials(data);
  };

  const createTrial = async (title: string, participantIds: number[]) => {
    const response = await fetch('/api/trials', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ title, participantIds }),
    });
    const newTrial = await response.json();
    setTrials([...trials, newTrial]);
    toast({
      title: "Success",
      description: "Trial created successfully",
    });
  };

  const deleteTrial = async (id: number) => {
    const response = await fetch(`/api/trials/${id}`, {
      method: 'DELETE',
    });

    if (response.ok) {
      setTrials(trials.filter(trial => trial.id !== id));
      toast({
        title: "Success",
        description: "Trial deleted successfully",
      });
    } else {
      toast({
        title: "Error",
        description: "Failed to delete trial",
        variant: "destructive",
      });
    }
  };

  return (
    <Card>
      <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
        <CardTitle className="text-2xl font-bold">Trials</CardTitle>
        <CreateTrialDialog onCreateTrial={createTrial} />
      </CardHeader>
      <CardContent>
        {trials.length > 0 ? (
          <div className="grid grid-cols-1 gap-4">
            {trials.map(trial => (
              <Card key={trial.id} className="bg-gray-100 p-3 flex items-center justify-between">
                <div>
                  <h3 className="font-semibold">{trial.title}</h3>
                  <p className="text-sm text-gray-500">Participants: {trial.participantIds.join(', ')}</p>
                </div>
                <Button
                  variant="ghost"
                  size="sm"
                  className="text-destructive"
                  onClick={() => deleteTrial(trial.id)}
                >
                  Delete
                </Button>
              </Card>
            ))}
          </div>
        ) : (
          <p>No trials added yet.</p>
        )}
      </CardContent>
    </Card>
  );
}
