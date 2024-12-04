'use client';

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { PlusIcon, Trash2Icon, Settings2 } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { useToast } from "~/hooks/use-toast";

interface Study {
  id: number;
  title: string;
  description: string;
  createdAt: string;
}

export default function Studies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [newStudyTitle, setNewStudyTitle] = useState("");
  const [newStudyDescription, setNewStudyDescription] = useState("");
  const [loading, setLoading] = useState(true);
  const router = useRouter();
  const { toast } = useToast();

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
      toast({
        title: "Error",
        description: "Failed to load studies",
        variant: "destructive",
      });
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

      if (!response.ok) {
        throw new Error('Failed to create study');
      }

      const newStudy = await response.json();
      setStudies([...studies, newStudy]);
      setNewStudyTitle("");
      setNewStudyDescription("");
      
      toast({
        title: "Success",
        description: "Study created successfully",
      });
    } catch (error) {
      console.error('Error creating study:', error);
      toast({
        title: "Error",
        description: "Failed to create study",
        variant: "destructive",
      });
    }
  };

  const deleteStudy = async (id: number) => {
    try {
      const response = await fetch(`/api/studies/${id}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete study');
      }

      setStudies(studies.filter(study => study.id !== id));
      toast({
        title: "Success",
        description: "Study deleted successfully",
      });
    } catch (error) {
      console.error('Error deleting study:', error);
      toast({
        title: "Error",
        description: "Failed to delete study",
        variant: "destructive",
      });
    }
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <div className="animate-spin h-8 w-8 border-4 border-primary border-t-transparent rounded-full" />
      </div>
    );
  }

  return (
    <div className="container py-6 space-y-6">
      <div>
        <h1 className="text-2xl font-bold">Studies</h1>
        <p className="text-muted-foreground">Manage your research studies</p>
      </div>

      <Card>
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
                placeholder="Enter study title"
                required
              />
            </div>
            <div className="space-y-2">
              <Label htmlFor="description">Description</Label>
              <Textarea
                id="description"
                value={newStudyDescription}
                onChange={(e) => setNewStudyDescription(e.target.value)}
                placeholder="Enter study description"
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
        {studies.length > 0 ? (
          studies.map((study) => (
            <Card key={study.id}>
              <CardHeader>
                <CardTitle>{study.title}</CardTitle>
                <CardDescription>{study.description}</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex items-center gap-2">
                  <Button
                    variant="outline"
                    onClick={() => router.push(`/dashboard/studies/${study.id}/settings`)}
                  >
                    <Settings2 className="w-4 h-4 mr-2" />
                    Settings
                  </Button>
                  <Button
                    variant="outline"
                    onClick={() => deleteStudy(study.id)}
                  >
                    <Trash2Icon className="w-4 h-4 mr-2" />
                    Delete
                  </Button>
                </div>
              </CardContent>
            </Card>
          ))
        ) : (
          <Card>
            <CardContent className="py-8">
              <p className="text-center text-muted-foreground">
                No studies created yet. Create your first study above.
              </p>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
