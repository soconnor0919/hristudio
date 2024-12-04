'use client';

import { useState } from "react";
import { useRouter } from "next/navigation";
import { PlusIcon, Trash2Icon, Settings2 } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import { useToast } from "~/hooks/use-toast";
import { PERMISSIONS, hasPermission } from "~/lib/permissions-client";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
  AlertDialogFooter
} from "~/components/ui/alert-dialog";
import { ROLES } from "~/lib/roles";

interface Study {
  id: number;
  title: string;
  description: string | null;
  createdAt: string;
  updatedAt: string | null;
  userId: string;
  permissions: string[];
  roles: string[];
}

// Helper function to format role name
function formatRoleName(role: string): string {
  return role
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
    .join(' ');
}

export default function Studies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [title, setTitle] = useState("");
  const [description, setDescription] = useState("");
  const router = useRouter();
  const { toast } = useToast();

  const fetchStudies = async () => {
    try {
      const response = await fetch("/api/studies");
      if (!response.ok) throw new Error("Failed to fetch studies");
      const data = await response.json();
      setStudies(data.data || []);
    } catch (error) {
      console.error("Error fetching studies:", error);
      toast({
        title: "Error",
        description: "Failed to load studies",
        variant: "destructive",
      });
    }
  };

  const createStudy = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const response = await fetch("/api/studies", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ title, description }),
      });

      if (!response.ok) {
        throw new Error("Failed to create study");
      }

      const data = await response.json();
      setStudies([...studies, data.data]);
      setTitle("");
      setDescription("");
      toast({
        title: "Success",
        description: "Study created successfully",
      });
    } catch (error) {
      console.error("Error creating study:", error);
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
        method: "DELETE",
      });

      if (!response.ok) {
        throw new Error("Failed to delete study");
      }

      setStudies(studies.filter(study => study.id !== id));
      toast({
        title: "Success",
        description: "Study deleted successfully",
      });
    } catch (error) {
      console.error("Error deleting study:", error);
      toast({
        title: "Error",
        description: "Failed to delete study",
        variant: "destructive",
      });
    }
  };

  // Fetch studies on mount
  useState(() => {
    fetchStudies();
  });

  return (
    <div className="container py-6 space-y-6">
      <div className="flex flex-col gap-2">
        <h1 className="text-3xl font-bold tracking-tight">Studies</h1>
        <p className="text-muted-foreground">
          Manage your research studies and experiments
        </p>
      </div>

      {hasPermission(studies[0]?.permissions || [], PERMISSIONS.CREATE_STUDY) && (
        <Card>
          <CardHeader>
            <CardTitle>Create New Study</CardTitle>
            <CardDescription>Add a new research study to your collection</CardDescription>
          </CardHeader>
          <CardContent>
            <form onSubmit={createStudy} className="space-y-4">
              <div className="space-y-2">
                <Label htmlFor="title">Study Title</Label>
                <Input
                  id="title"
                  value={title}
                  onChange={(e) => setTitle(e.target.value)}
                  placeholder="Enter study title"
                  required
                />
              </div>
              <div className="space-y-2">
                <Label htmlFor="description">Description</Label>
                <Textarea
                  id="description"
                  value={description}
                  onChange={(e) => setDescription(e.target.value)}
                  placeholder="Enter study description"
                />
              </div>
              <Button type="submit">
                <PlusIcon className="w-4 h-4 mr-2" />
                Create Study
              </Button>
            </form>
          </CardContent>
        </Card>
      )}

      <div className="grid gap-4">
        {studies.length > 0 ? (
          studies.map((study) => (
            <Card key={study.id} className="overflow-hidden">
              <div className="p-6">
                <div className="flex flex-col gap-1.5">
                  <div className="flex items-start justify-between">
                    <div className="space-y-1">
                      <h3 className="font-semibold leading-none tracking-tight">
                        {study.title}
                      </h3>
                      <p className="text-sm text-muted-foreground">
                        {study.description || "No description provided."}
                      </p>
                      <p className="text-sm">
                        <span className="text-muted-foreground">Your Roles: </span>
                        <span className="text-foreground">
                          {study.roles?.map(formatRoleName).join(", ")}
                        </span>
                      </p>
                    </div>
                    <div className="flex items-center gap-2">
                      {(hasPermission(study.permissions, PERMISSIONS.EDIT_STUDY) || 
                        hasPermission(study.permissions, PERMISSIONS.MANAGE_ROLES)) && (
                        <Button
                          variant="outline"
                          size="sm"
                          onClick={() => router.push(`/dashboard/studies/${study.id}/settings`)}
                        >
                          <Settings2 className="w-4 h-4 mr-2" />
                          Settings
                        </Button>
                      )}
                      {hasPermission(study.permissions, PERMISSIONS.MANAGE_SYSTEM_SETTINGS) && (
                        <AlertDialog>
                          <AlertDialogTrigger asChild>
                            <Button variant="outline" size="sm">
                              <Trash2Icon className="w-4 h-4 mr-2" />
                              Delete
                            </Button>
                          </AlertDialogTrigger>
                          <AlertDialogContent>
                            <AlertDialogHeader>
                              <AlertDialogTitle>Are you absolutely sure?</AlertDialogTitle>
                              <div className="text-sm text-muted-foreground">
                                <p className="mb-2">
                                  This action cannot be undone. This will permanently delete the study
                                  &quot;{study.title}&quot; and all associated data including:
                                </p>
                                <ul className="list-disc list-inside">
                                  <li>All participant data</li>
                                  <li>All user roles and permissions</li>
                                  <li>All pending invitations</li>
                                </ul>
                              </div>
                            </AlertDialogHeader>
                            <AlertDialogFooter>
                              <AlertDialogCancel>Cancel</AlertDialogCancel>
                              <AlertDialogAction
                                onClick={() => deleteStudy(study.id)}
                                className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                              >
                                Delete Study
                              </AlertDialogAction>
                            </AlertDialogFooter>
                          </AlertDialogContent>
                        </AlertDialog>
                      )}
                    </div>
                  </div>
                </div>
              </div>
            </Card>
          ))
        ) : (
          <Card>
            <CardContent className="py-8">
              <p className="text-center text-muted-foreground">
                No studies created yet.{' '}
                {hasPermission(studies[0]?.permissions || [], PERMISSIONS.CREATE_STUDY) 
                  ? "Create your first study above."
                  : "Ask your administrator to create your first study."
                }
              </p>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
