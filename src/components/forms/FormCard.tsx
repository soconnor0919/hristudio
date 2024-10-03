import Image from 'next/image';
import { Card, CardContent, CardFooter } from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Trash2 } from "lucide-react";
import { Button } from "~/components/ui/button";

interface FormCardProps {
  form: {
    id: number;
    title: string;
    location: string;
    studyId: number;
    studyTitle: string;
    participantId: number;
    participantName: string;
    previewLocation: string;
  };
  onDelete: (formId: number) => void;
}

export function FormCard({ form, onDelete }: FormCardProps) {
  const handleCardClick = () => {
    window.open(form.location, '_blank');
  };

  return (
    <Card className="overflow-hidden cursor-pointer" onClick={handleCardClick} style={{ backgroundColor: 'var(--primary-card-background)' }}>
      <CardContent className="p-0 h-40 relative">
        <Image
          src={form.previewLocation}
          alt={form.title}
          fill
          className="object-cover object-top"
          onError={(e) => {
            e.currentTarget.src = '/placeholder-image.png';
            console.error('Error loading image:', form.previewLocation);
          }}
        />
      </CardContent>
      <CardFooter className="flex flex-col items-start p-4" style={{ backgroundColor: 'var(--secondary-card-background)' }}>
        <div className="flex items-center justify-between w-full">
          <h3 className="font-semibold mb-2">{form.title}</h3>
          <Button
            variant="ghost"
            size="sm"
            className="text-destructive"
            onClick={(e) => {
              e.stopPropagation();
              onDelete(form.id);
            }}
          >
            <Trash2 className="h-4 w-4" />
            Delete
          </Button>
        </div>
        <div className="flex flex-wrap gap-2 mb-2">
          <Badge variant="secondary">{form.studyTitle}</Badge>
          <Badge variant="outline">{form.participantName}</Badge>
        </div>
      </CardFooter>
    </Card>
  );
}