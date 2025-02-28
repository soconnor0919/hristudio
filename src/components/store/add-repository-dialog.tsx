"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useToast } from "~/hooks/use-toast";
import { Button } from "~/components/ui/button";
import { Plus } from "lucide-react";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { Input } from "~/components/ui/input";
import { api } from "~/trpc/react";
import { Alert, AlertDescription } from "~/components/ui/alert";

export function AddRepositoryDialog() {
  const router = useRouter();
  const [isLoading, setIsLoading] = useState(false);
  const [isOpen, setIsOpen] = useState(false);
  const [url, setUrl] = useState("");
  const { toast } = useToast();
  const utils = api.useUtils();

  const addRepository = api.pluginStore.addRepository.useMutation({
    onSuccess: async () => {
      toast({
        title: "Success",
        description: "Repository added successfully",
      });
      setIsOpen(false);
      setUrl("");

      // Invalidate and refetch all plugin store queries
      await Promise.all([
        utils.pluginStore.getRepositories.invalidate(),
        utils.pluginStore.getPlugins.invalidate(),
        utils.pluginStore.getInstalledPlugins.invalidate(),
      ]);

      // Force refetch
      await Promise.all([
        utils.pluginStore.getRepositories.refetch(),
        utils.pluginStore.getPlugins.refetch(),
        utils.pluginStore.getInstalledPlugins.refetch(),
      ]);
    },
    onError: (error) => {
      console.error("Failed to add repository:", error);
      toast({
        title: "Error",
        description: error.message || "Failed to add repository",
        variant: "destructive",
      });
    },
  });

  const handleAddRepository = async () => {
    if (!url) {
      toast({
        title: "Error",
        description: "Please enter a repository URL",
        variant: "destructive",
      });
      return;
    }

    try {
      setIsLoading(true);
      await addRepository.mutateAsync({ url });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Dialog open={isOpen} onOpenChange={setIsOpen}>
      <DialogTrigger asChild>
        <Button size="sm">
          <Plus className="h-4 w-4 mr-2" />
          Add Repository
        </Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Add Plugin Repository</DialogTitle>
          <DialogDescription>
            Enter the URL of a plugin repository. The repository must contain a repository.json file and follow the HRIStudio plugin repository structure.
          </DialogDescription>
        </DialogHeader>
        <div className="grid gap-4 py-4">
          <Alert>
            <AlertDescription>
              Example repository URL:
              <code className="ml-2 rounded bg-muted px-1.5 py-0.5">
                https://soconnor0919.github.io/robot-plugins
              </code>
            </AlertDescription>
          </Alert>
          <div className="grid gap-2">
            <Input
              placeholder="Enter repository URL"
              value={url}
              onChange={(e) => setUrl(e.target.value)}
            />
          </div>
        </div>
        <DialogFooter>
          <Button
            onClick={handleAddRepository}
            disabled={isLoading || addRepository.isLoading}
          >
            {isLoading || addRepository.isLoading ? "Adding..." : "Add Repository"}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
} 