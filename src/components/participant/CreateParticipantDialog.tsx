import React, { useState } from 'react';
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from "~/components/ui/dialog";
import { Label } from "~/components/ui/label";
import { PlusCircle } from 'lucide-react';

interface CreateParticipantDialogProps {
  onCreateParticipant: (name: string) => void;
}

export function CreateParticipantDialog({ onCreateParticipant }: CreateParticipantDialogProps) {
  const [newParticipant, setNewParticipant] = useState({ name: '' });
  const [isOpen, setIsOpen] = useState(false);

  const handleCreate = () => {
    if (newParticipant.name) {
      onCreateParticipant(newParticipant.name);
      setNewParticipant({ name: '' });
      setIsOpen(false);
    }
  };

  return (
    <Dialog open={isOpen} onOpenChange={setIsOpen}>
      <DialogTrigger asChild>
        <Button variant="outline" size="icon">
          <PlusCircle className="h-4 w-4" />
        </Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Add New Participant</DialogTitle>
        </DialogHeader>
        <div className="grid gap-4 py-4">
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="name" className="text-right">
              Name
            </Label>
            <Input
              id="name"
              value={newParticipant.name}
              onChange={(e) => setNewParticipant({ name: e.target.value })}
              className="col-span-3"
            />
          </div>
        </div>
        <Button onClick={handleCreate}>Add Participant</Button>
      </DialogContent>
    </Dialog>
  );
}