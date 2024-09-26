import { Card, CardContent, CardFooter } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { Trash2 } from "lucide-react";
import { Participant } from "../../types/Participant";

interface ParticipantCardProps {
  participant: Participant;
  onDelete: (participantId: number) => void;
}

export function ParticipantCard({ participant, onDelete }: ParticipantCardProps) {
  return (
    <Card className="overflow-hidden">
      <CardContent className="p-4">
        <h3 className="font-semibold mb-2">{participant.name}</h3>
      </CardContent>
      <CardFooter className="flex justify-end p-4">
        <Button
          variant="ghost"
          size="sm"
          className="text-destructive"
          onClick={() => onDelete(participant.id)}
        >
          <Trash2 className="h-4 w-4 mr-2" />
          Delete
        </Button>
      </CardFooter>
    </Card>
  );
}