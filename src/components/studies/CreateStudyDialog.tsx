"use client";

import { useState } from "react";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Card, CardContent } from "~/components/ui/card";
import { Textarea } from "~/components/ui/textarea";
import { api } from "~/trpc/react";

const createStudySchema = z.object({
  name: z.string().min(1, "Study name is required").max(100, "Name too long"),
  description: z
    .string()
    .min(10, "Description must be at least 10 characters")
    .max(1000, "Description too long"),
  irbProtocolNumber: z.string().optional(),
  institution: z
    .string()
    .min(1, "Institution is required")
    .max(100, "Institution name too long"),
  status: z.enum(["draft", "active", "completed", "archived"]),
});

type CreateStudyFormData = z.infer<typeof createStudySchema>;

interface CreateStudyDialogProps {
  children: React.ReactNode;
  onSuccess?: () => void;
}

export function CreateStudyDialog({
  children,
  onSuccess,
}: CreateStudyDialogProps) {
  const [open, setOpen] = useState(false);

  const {
    register,
    handleSubmit,
    reset,
    setValue,
    watch,
    formState: { errors, isSubmitting },
  } = useForm<CreateStudyFormData>({
    resolver: zodResolver(createStudySchema),
    defaultValues: {
      status: "draft" as const,
    },
  });

  const createStudyMutation = api.studies.create.useMutation({
    onSuccess: () => {
      setOpen(false);
      reset();
      onSuccess?.();
    },
    onError: (err) => {
      console.error("Failed to create study:", err);
    },
  });

  const onSubmit = async (data: CreateStudyFormData) => {
    try {
      await createStudyMutation.mutateAsync(data);
    } catch (error) {
      // Error handling is done in the mutation's onError callback
    }
  };

  const watchedStatus = watch("status");

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>{children}</DialogTrigger>
      <DialogContent className="max-w-md">
        <DialogHeader>
          <DialogTitle>Create New Study</DialogTitle>
          <DialogDescription>
            Start a new Human-Robot Interaction research study. You&apos;ll be
            assigned as the study owner.
          </DialogDescription>
        </DialogHeader>

        <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
          {/* Study Name */}
          <div className="space-y-2">
            <Label htmlFor="name">Study Name *</Label>
            <Input
              id="name"
              {...register("name")}
              placeholder="Enter study name..."
              className={errors.name ? "border-red-500" : ""}
            />
            {errors.name && (
              <p className="text-sm text-red-600">{errors.name.message}</p>
            )}
          </div>

          {/* Description */}
          <div className="space-y-2">
            <Label htmlFor="description">Description *</Label>
            <Textarea
              id="description"
              {...register("description")}
              placeholder="Describe your research study, objectives, and methodology..."
              rows={4}
              className={errors.description ? "border-red-500" : ""}
            />
            {errors.description && (
              <p className="text-sm text-red-600">
                {errors.description.message}
              </p>
            )}
          </div>

          {/* Institution */}
          <div className="space-y-2">
            <Label htmlFor="institution">Institution *</Label>
            <Input
              id="institution"
              {...register("institution")}
              placeholder="University or research institution..."
              className={errors.institution ? "border-red-500" : ""}
            />
            {errors.institution && (
              <p className="text-sm text-red-600">
                {errors.institution.message}
              </p>
            )}
          </div>

          {/* IRB Protocol Number */}
          <div className="space-y-2">
            <Label htmlFor="irbProtocolNumber">IRB Protocol Number</Label>
            <Input
              id="irbProtocolNumber"
              {...register("irbProtocolNumber")}
              placeholder="Optional IRB protocol number..."
            />
            <p className="text-muted-foreground text-xs">
              If your study has been approved by an Institutional Review Board
            </p>
          </div>

          {/* Status */}
          <div className="space-y-2">
            <Label htmlFor="status">Initial Status</Label>
            <Select
              value={watchedStatus}
              onValueChange={(value) =>
                setValue(
                  "status",
                  value as "draft" | "active" | "completed" | "archived",
                )
              }
            >
              <SelectTrigger>
                <SelectValue placeholder="Select status" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="draft">Draft - Planning stage</SelectItem>
                <SelectItem value="active">
                  Active - Recruiting participants
                </SelectItem>
                <SelectItem value="completed">
                  Completed - Data collection finished
                </SelectItem>
                <SelectItem value="archived">
                  Archived - Study concluded
                </SelectItem>
              </SelectContent>
            </Select>
          </div>

          {/* Info Card */}
          <Card>
            <CardContent className="pt-4">
              <div className="flex items-start space-x-3">
                <div className="mt-0.5 flex h-5 w-5 items-center justify-center rounded-full bg-blue-100">
                  <svg
                    className="h-3 w-3 text-blue-600"
                    fill="currentColor"
                    viewBox="0 0 20 20"
                  >
                    <path
                      fillRule="evenodd"
                      d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z"
                      clipRule="evenodd"
                    />
                  </svg>
                </div>
                <div className="text-muted-foreground text-sm">
                  <p className="text-foreground font-medium">
                    What happens next?
                  </p>
                  <ul className="mt-1 space-y-1 text-xs">
                    <li>• You&apos;ll be assigned as the study owner</li>
                    <li>• You can invite team members and assign roles</li>
                    <li>• Start designing experiments and protocols</li>
                    <li>• Schedule trials and manage participants</li>
                  </ul>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Error Message */}
          {createStudyMutation.error && (
            <div className="rounded-md bg-red-50 p-3">
              <p className="text-sm text-red-800">
                Failed to create study: {createStudyMutation.error.message}
              </p>
            </div>
          )}

          {/* Form Actions */}
          <div className="flex justify-end space-x-3">
            <Button
              type="button"
              variant="outline"
              onClick={() => setOpen(false)}
              disabled={isSubmitting}
            >
              Cancel
            </Button>
            <Button
              type="submit"
              disabled={isSubmitting}
              className="min-w-[100px]"
            >
              {isSubmitting ? (
                <div className="flex items-center space-x-2">
                  <svg
                    className="h-4 w-4 animate-spin"
                    fill="none"
                    viewBox="0 0 24 24"
                  >
                    <circle
                      className="opacity-25"
                      cx="12"
                      cy="12"
                      r="10"
                      stroke="currentColor"
                      strokeWidth="4"
                    />
                    <path
                      className="opacity-75"
                      fill="currentColor"
                      d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                    />
                  </svg>
                  <span>Creating...</span>
                </div>
              ) : (
                "Create Study"
              )}
            </Button>
          </div>
        </form>
      </DialogContent>
    </Dialog>
  );
}
