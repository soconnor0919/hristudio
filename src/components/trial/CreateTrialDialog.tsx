import React, { useState } from 'react';
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from "~/components/ui/dialog";
import { Label } from "~/components/ui/label";

interface CreateTrialDialogProps {
  onCreateTrial: (title: string, participantIds: number[]) => void;
}

export function CreateTrialDialog({ onCreateTrial }: CreateTrialDialogProps) {
  const [title, setTitle] = useState('');
  const [participantIds, setParticipantIds] = useState<string>('');

  const handleCreate = () => {
    const ids = participantIds.split(',').map(id => parseInt(id.trim())).filter(id => !isNaN(id));
    if (title && ids.length > 0) {
      onCreateTrial(title, ids);
      setTitle('');
      setParticipantIds('');
    }
  };

  return (
    <Dialog>
      <DialogTrigger asChild>
        <Button variant="outline">Add Trial</Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Add New Trial</DialogTitle>
        </DialogHeader>
        <div className="grid gap-4 py-4">
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="title" className="text-right">Title</Label>
            <Input
              id="title"
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              className="col-span-3"
            />
          </div>
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="participants" className="text-right">Participant IDs (comma-separated)</Label>
            <Input
              id="participants"
              value={participantIds}
              onChange={(e) => setParticipantIds(e.target.value)}
              className="col-span-3"
              placeholder="e.g. 1, 2, 3"
            />
          </div>
        </div>
        <Button onClick={handleCreate}>Add Trial</Button>
      </DialogContent>
    </Dialog>
  );
}
