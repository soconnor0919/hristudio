import { useState } from 'react';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from "~/components/ui/dialog";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { Label } from "~/components/ui/label";
import { PlusCircle } from 'lucide-react';
import { Study } from '../../types/Study';

interface CreateStudyDialogProps {
  onCreateStudy: (study: Omit<Study, 'id'>) => void;
}

export function CreateStudyDialog({ onCreateStudy }: CreateStudyDialogProps) {
  const [isOpen, setIsOpen] = useState(false);
  const [newStudy, setNewStudy] = useState({ title: '', description: '' });
  const [touched, setTouched] = useState({ title: false, description: false });

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setNewStudy({ ...newStudy, [name]: value });
    setTouched({ ...touched, [name]: true });
  };

  const isFieldInvalid = (field: 'title' | 'description') => {
    return field === 'title' ? (touched.title && !newStudy.title) : false;
  };

  const handleCreateStudy = () => {
    setTouched({ title: true, description: true });

    if (!newStudy.title) {
      return;
    }

    onCreateStudy({
      title: newStudy.title,
      description: newStudy.description || undefined
    });

    setNewStudy({ title: '', description: '' });
    setTouched({ title: false, description: false });
    setIsOpen(false);
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
          <DialogTitle>Create New Study</DialogTitle>
        </DialogHeader>
        <div className="grid gap-4 py-4">
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="title" className="text-right">
              Title
            </Label>
            <Input
              id="title"
              name="title"
              className={`col-span-3 ${isFieldInvalid('title') ? 'border-red-500' : ''}`}
              value={newStudy.title}
              onChange={handleInputChange}
            />
          </div>
          {isFieldInvalid('title') && (
            <p className="text-red-500 text-sm col-span-4">Title is required</p>
          )}
          <div className="grid grid-cols-4 items-center gap-4">
            <Label htmlFor="description" className="text-right">
              Description
            </Label>
            <Textarea
              id="description"
              name="description"
              className="col-span-3"
              value={newStudy.description}
              onChange={handleInputChange}
            />
          </div>
        </div>
        <Button onClick={handleCreateStudy}>Create Study</Button>
      </DialogContent>
    </Dialog>
  );
}