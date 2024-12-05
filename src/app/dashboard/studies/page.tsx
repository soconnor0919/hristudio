'use client';

import { useCallback, useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { PlusIcon, Trash2Icon, Settings2, ArrowRight } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
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
import { getApiUrl } from "~/lib/fetch-utils";
import { Skeleton } from "~/components/ui/skeleton";
import { useActiveStudy } from "~/context/active-study";

interface Study {
  id: number;
  title: string;
  description: string | null;
  userId: string;
  environment: string;
  createdAt: Date;
  updatedAt: Date | null;
  permissions: string[];
  roles: string[];
}

function formatRoleName(role: string): string {
  return role
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
    .join(' ');
}

export default function Studies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const router = useRouter();
  const { toast } = useToast();
  const { setActiveStudy } = useActiveStudy();

  const fetchStudies = useCallback(async () => {
    try {
      const response = await fetch(getApiUrl("/api/studies"));
      if (!response.ok) throw new Error("Failed to fetch studies");
      const { data } = await response.json();
      setStudies(data.map((study: any) => ({
        ...study,
        createdAt: new Date(study.createdAt),
        updatedAt: study.updatedAt ? new Date(study.updatedAt) : null
      })));
    } catch (error) {
      console.error("Error fetching studies:", error);
      toast({
        title: "Error",
        description: "Failed to load studies",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
    }
  }, [toast]);

  useEffect(() => {
    fetchStudies();
  }, [fetchStudies]);

  const handleDelete = async (id: number) => {
    try {
      const response = await fetch(getApiUrl(`/api/studies/${id}`), {
        method: "DELETE",
      });

      if (!response.ok) throw new Error("Failed to delete study");

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

  const handleEnterStudy = (study: Study) => {
    setActiveStudy(study);
    router.push(`/dashboard/studies/${study.id}`);
  };

  if (isLoading) {
    return (
      <div className="space-y-6">
        <div className="flex items-center justify-between">
          <div>
            <Skeleton className="h-8 w-[150px] mb-2" />
            <Skeleton className="h-4 w-[300px]" />
          </div>
          <Skeleton className="h-10 w-[140px]" />
        </div>

        <div className="grid gap-4">
          {[1, 2, 3].map((i) => (
            <Card key={i}>
              <CardContent className="p-6">
                <div className="flex items-start justify-between">
                  <div className="space-y-2">
                    <Skeleton className="h-5 w-[200px] mb-1" />
                    <Skeleton className="h-4 w-[300px] mb-1" />
                    <Skeleton className="h-4 w-[150px]" />
                  </div>
                  <div className="flex items-center gap-2">
                    <Skeleton className="h-9 w-[100px]" />
                    <Skeleton className="h-9 w-9" />
                  </div>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold tracking-tight">Studies</h2>
          <p className="text-muted-foreground">
            Manage your research studies and experiments
          </p>
        </div>
        {hasPermission(studies[0]?.permissions || [], PERMISSIONS.CREATE_STUDY) && (
          <Button onClick={() => router.push('/dashboard/studies/new')}>
            <PlusIcon className="w-4 h-4 mr-2" />
            Create New Study
          </Button>
        )}
      </div>

      <div className="grid gap-4">
        {studies.length > 0 ? (
          studies.map((study) => (
            <Card key={study.id}>
              <CardContent className="p-6">
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
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleEnterStudy(study)}
                    >
                      Enter Study
                      <ArrowRight className="ml-2 h-4 w-4" />
                    </Button>
                    {(hasPermission(study.permissions, PERMISSIONS.EDIT_STUDY) || 
                      hasPermission(study.permissions, PERMISSIONS.MANAGE_ROLES)) && (
                      <Button
                        variant="ghost"
                        size="sm"
                        onClick={() => router.push(`/dashboard/studies/${study.id}/settings`)}
                      >
                        <Settings2 className="h-4 w-4" />
                      </Button>
                    )}
                    {hasPermission(study.permissions, PERMISSIONS.DELETE_STUDY) && (
                      <AlertDialog>
                        <AlertDialogTrigger asChild>
                          <Button variant="ghost" size="sm">
                            <Trash2Icon className="h-4 w-4" />
                          </Button>
                        </AlertDialogTrigger>
                        <AlertDialogContent>
                          <AlertDialogHeader>
                            <AlertDialogTitle>Delete Study</AlertDialogTitle>
                          </AlertDialogHeader>
                          <p>
                            Are you sure you want to delete this study? This action cannot be undone.
                          </p>
                          <AlertDialogFooter>
                            <AlertDialogCancel>Cancel</AlertDialogCancel>
                            <AlertDialogAction
                              onClick={() => handleDelete(study.id)}
                              className="bg-destructive text-destructive-foreground hover:bg-destructive/90"
                            >
                              Delete
                            </AlertDialogAction>
                          </AlertDialogFooter>
                        </AlertDialogContent>
                      </AlertDialog>
                    )}
                  </div>
                </div>
              </CardContent>
            </Card>
          ))
        ) : (
          <Card>
            <CardContent className="py-8">
              <p className="text-center text-muted-foreground">
                No studies found{hasPermission(studies[0]?.permissions || [], PERMISSIONS.CREATE_STUDY) ? ". Create your first study above" : ""}.
              </p>
            </CardContent>
          </Card>
        )}
      </div>
    </div>
  );
}
