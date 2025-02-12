"use client";

import { useState } from "react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
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
import { Label } from "~/components/ui/label";
import { api } from "~/trpc/react";
import { useToast } from "~/hooks/use-toast";
import { Plus, Trash2 } from "lucide-react";
import { Badge } from "~/components/ui/badge";

interface StudyMetadataProps {
  studyId: number;
  role: string;
}

export function StudyMetadata({ studyId, role }: StudyMetadataProps) {
  const [isAddOpen, setIsAddOpen] = useState(false);
  const [key, setKey] = useState("");
  const [value, setValue] = useState("");
  const { toast } = useToast();

  const { data: metadata, refetch } = api.study.getMetadata.useQuery({ studyId });
  const { mutate: addMetadata, isPending: isAdding } = api.study.addMetadata.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Metadata added successfully",
      });
      setIsAddOpen(false);
      setKey("");
      setValue("");
      refetch();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const { mutate: deleteMetadata } = api.study.deleteMetadata.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Metadata deleted successfully",
      });
      refetch();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const canManageMetadata = role === "ADMIN";

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <div>
            <CardTitle>Study Metadata</CardTitle>
            <CardDescription>Custom fields and tags for this study</CardDescription>
          </div>
          {canManageMetadata && (
            <Dialog open={isAddOpen} onOpenChange={setIsAddOpen}>
              <DialogTrigger asChild>
                <Button size="sm">
                  <Plus className="h-4 w-4 mr-2" />
                  Add Field
                </Button>
              </DialogTrigger>
              <DialogContent>
                <DialogHeader>
                  <DialogTitle>Add Metadata Field</DialogTitle>
                  <DialogDescription>
                    Add a new custom field or tag to this study.
                  </DialogDescription>
                </DialogHeader>
                <div className="grid gap-4 py-4">
                  <div className="grid gap-2">
                    <Label htmlFor="key">Field Name</Label>
                    <Input
                      id="key"
                      placeholder="Enter field name"
                      value={key}
                      onChange={(e) => setKey(e.target.value)}
                    />
                  </div>
                  <div className="grid gap-2">
                    <Label htmlFor="value">Value</Label>
                    <Input
                      id="value"
                      placeholder="Enter value"
                      value={value}
                      onChange={(e) => setValue(e.target.value)}
                    />
                  </div>
                </div>
                <DialogFooter>
                  <Button
                    onClick={() => addMetadata({ studyId, key, value })}
                    disabled={isAdding}
                  >
                    {isAdding ? "Adding..." : "Add Field"}
                  </Button>
                </DialogFooter>
              </DialogContent>
            </Dialog>
          )}
        </div>
      </CardHeader>
      <CardContent>
        {!metadata || metadata.length === 0 ? (
          <div className="text-center py-6 text-muted-foreground">
            No metadata fields found
          </div>
        ) : (
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>Field</TableHead>
                <TableHead>Value</TableHead>
                {canManageMetadata && <TableHead className="w-[100px]">Actions</TableHead>}
              </TableRow>
            </TableHeader>
            <TableBody>
              {metadata.map((item) => (
                <TableRow key={item.key}>
                  <TableCell className="font-medium">{item.key}</TableCell>
                  <TableCell>
                    <Badge variant="secondary">{item.value}</Badge>
                  </TableCell>
                  {canManageMetadata && (
                    <TableCell>
                      <Button
                        variant="ghost"
                        size="sm"
                        onClick={() => deleteMetadata({ studyId, key: item.key })}
                      >
                        <Trash2 className="h-4 w-4" />
                        <span className="sr-only">Delete</span>
                      </Button>
                    </TableCell>
                  )}
                </TableRow>
              ))}
            </TableBody>
          </Table>
        )}
      </CardContent>
    </Card>
  );
} 