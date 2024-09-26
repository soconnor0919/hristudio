import { Card, CardContent, CardFooter } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Trash2 } from "lucide-react";

interface FormCardProps {
  form: {
    id: number;
    title: string;
    location: string;
    studyId: number;
    studyTitle: string;
    participantId: number;
    participantName: string;
    previewLocation: string; // Added this property
  };
  onDelete: (formId: number) => void;
}

export function FormCard({ form, onDelete }: FormCardProps) {
  return (
    <Card className="overflow-hidden">
      <CardContent className="p-0">
        <img
          src={form.previewLocation}
          alt={form.title}
          className="w-full h-40 object-cover"
        />
      </CardContent>
      <CardFooter className="flex flex-col items-start p-4">
        <h3 className="font-semibold mb-2">{form.title}</h3>
        <div className="flex flex-wrap gap-2 mb-2">
          <Badge variant="secondary">{form.studyTitle}</Badge>
          <Badge variant="outline">{form.participantName}</Badge>
        </div>
        <Button
          variant="ghost"
          size="sm"
          className="text-destructive"
          onClick={() => onDelete(form.id)}
        >
          <Trash2 className="h-4 w-4 mr-2" />
          Delete
        </Button>
      </CardFooter>
    </Card>
  );
}