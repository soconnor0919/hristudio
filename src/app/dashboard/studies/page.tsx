'use client';

import { useEffect, useState } from "react";
import { Button } from "~/components/ui/button";
import { PlusIcon, Trash2Icon } from "lucide-react";
import { 
  Card, 
  CardHeader, 
  CardTitle, 
  CardDescription, 
  CardContent, 
  CardFooter 
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { Label } from "~/components/ui/label";

interface Study {
  id: number;
  title: string;
  description: string | null;
  createdAt: string;
}

export default function Studies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [loading, setLoading] = useState(true);
  const [newStudyTitle, setNewStudyTitle] = useState("");
  const [newStudyDescription, setNewStudyDescription] = useState("");

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

  const createStudy = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const response = await fetch('/api/studies', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          title: newStudyTitle,
          description: newStudyDescription,
        }),
      });
      const newStudy = await response.json();
      setStudies([...studies, newStudy]);
      setNewStudyTitle("");
      setNewStudyDescription("");
    } catch (error) {
      console.error('Error creating study:', error);
    }
  };

  const deleteStudy = async (id: number) => {
    try {
      await fetch(`/api/studies/${id}`, {
        method: 'DELETE',
      });
      setStudies(studies.filter(study => study.id !== id));
    } catch (error) {
      console.error('Error deleting study:', error);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="max-w-4xl mx-auto">
      <div className="flex justify-between items-center mb-8">
        <h1 className="text-3xl font-bold">Studies</h1>
      </div>

      <Card className="mb-8">
        <CardHeader>
          <CardTitle>Create New Study</CardTitle>
          <CardDescription>
            Add a new research study to your collection
          </CardDescription>
        </CardHeader>
        <CardContent>
          <form onSubmit={createStudy} className="space-y-4">
            <div className="space-y-2">
              <Label htmlFor="title">Study Title</Label>
              <Input
                type="text"
                id="title"
                value={newStudyTitle}
                onChange={(e) => setNewStudyTitle(e.target.value)}
                required
              />
            </div>
            <div className="space-y-2">
              <Label htmlFor="description">Description</Label>
              <Textarea
                id="description"
                value={newStudyDescription}
                onChange={(e) => setNewStudyDescription(e.target.value)}
                rows={3}
              />
            </div>
            <Button type="submit">
              <PlusIcon className="w-4 h-4 mr-2" />
              Create Study
            </Button>
          </form>
        </CardContent>
      </Card>

      <div className="grid gap-4">
        {studies.map((study) => (
          <Card key={study.id}>
            <CardHeader>
              <div className="flex justify-between items-start">
                <div>
                  <CardTitle>{study.title}</CardTitle>
                  {study.description && (
                    <CardDescription className="mt-1.5">
                      {study.description}
                    </CardDescription>
                  )}
                </div>
                <Button variant="ghost" size="icon" className="text-destructive" onClick={() => deleteStudy(study.id)}>
                  <Trash2Icon className="w-4 h-4" />
                </Button>
              </div>
            </CardHeader>
            <CardFooter className="text-sm text-muted-foreground">
              Created: {new Date(study.createdAt).toLocaleDateString()}
            </CardFooter>
          </Card>
        ))}
      </div>
    </div>
  );
}
