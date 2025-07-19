"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import Link from "next/link";
import { ArrowLeft, FlaskConical } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Textarea } from "~/components/ui/textarea";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { api } from "~/trpc/react";

const createExperimentSchema = z.object({
  name: z.string().min(1, "Experiment name is required").max(100, "Name too long"),
  description: z
    .string()
    .min(10, "Description must be at least 10 characters")
    .max(1000, "Description too long"),
  studyId: z.string().uuid("Please select a study"),
  estimatedDuration: z
    .number()
    .min(1, "Duration must be at least 1 minute")
    .max(480, "Duration cannot exceed 8 hours")
    .optional(),
  status: z.enum(["draft", "active", "completed", "archived"]),
});

type CreateExperimentFormData = z.infer<typeof createExperimentSchema>;

export default function NewExperimentPage() {
  const router = useRouter();
  const [isSubmitting, setIsSubmitting] = useState(false);

  const {
    register,
    handleSubmit,
    setValue,
    watch,
    formState: { errors },
  } = useForm<CreateExperimentFormData>({
    resolver: zodResolver(createExperimentSchema),
    defaultValues: {
      status: "draft" as const,
    },
  });

  // Fetch user's studies for the dropdown
  const { data: studiesData, isLoading: studiesLoading } = api.studies.list.useQuery(
    { memberOnly: true },
  );

  const createExperimentMutation = api.experiments.create.useMutation({
    onSuccess: (experiment) => {
      router.push(`/experiments/${experiment.id}/designer`);
    },
    onError: (error) => {
      console.error("Failed to create experiment:", error);
      setIsSubmitting(false);
    },
  });

  const onSubmit = async (data: CreateExperimentFormData) => {
    setIsSubmitting(true);
    try {
      await createExperimentMutation.mutateAsync({
        ...data,
        estimatedDuration: data.estimatedDuration || null,
      });
    } catch (error) {
      // Error handling is done in the mutation's onError callback
    }
  };

  const watchedStatus = watch("status");
  const watchedStudyId = watch("studyId");

  return (
    <div className="p-8">
      {/* Header */}
      <div className="mb-8">
        <div className="flex items-center space-x-2 text-sm text-slate-600 mb-4">
          <Link href="/experiments" className="hover:text-slate-900 flex items-center">
            <ArrowLeft className="h-4 w-4 mr-1" />
            Experiments
          </Link>
          <span>/</span>
          <span className="text-slate-900">New Experiment</span>
        </div>

        <div className="flex items-center space-x-3">
          <div className="flex h-12 w-12 items-center justify-center rounded-lg bg-blue-100">
            <FlaskConical className="h-6 w-6 text-blue-600" />
          </div>
          <div>
            <h1 className="text-3xl font-bold text-slate-900">Create New Experiment</h1>
            <p className="text-slate-600">Design a new experimental protocol for your HRI study</p>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Main Form */}
        <div className="lg:col-span-2">
          <Card>
            <CardHeader>
              <CardTitle>Experiment Details</CardTitle>
              <CardDescription>
                Define the basic information for your experiment. You&apos;ll design the protocol steps next.
              </CardDescription>
            </CardHeader>
            <CardContent>
              <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
                {/* Experiment Name */}
                <div className="space-y-2">
                  <Label htmlFor="name">Experiment Name *</Label>
                  <Input
                    id="name"
                    {...register("name")}
                    placeholder="Enter experiment name..."
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
                    placeholder="Describe the experiment objectives, methodology, and expected outcomes..."
                    rows={4}
                    className={errors.description ? "border-red-500" : ""}
                  />
                  {errors.description && (
                    <p className="text-sm text-red-600">{errors.description.message}</p>
                  )}
                </div>

                {/* Study Selection */}
                <div className="space-y-2">
                  <Label htmlFor="studyId">Study *</Label>
                  <Select
                    value={watchedStudyId}
                    onValueChange={(value) => setValue("studyId", value)}
                    disabled={studiesLoading}
                  >
                    <SelectTrigger className={errors.studyId ? "border-red-500" : ""}>
                      <SelectValue placeholder={studiesLoading ? "Loading studies..." : "Select a study"} />
                    </SelectTrigger>
                    <SelectContent>
                      {studiesData?.studies?.map((study) => (
                        <SelectItem key={study.id} value={study.id}>
                          {study.name}
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  {errors.studyId && (
                    <p className="text-sm text-red-600">{errors.studyId.message}</p>
                  )}
                </div>

                {/* Estimated Duration */}
                <div className="space-y-2">
                  <Label htmlFor="estimatedDuration">Estimated Duration (minutes)</Label>
                  <Input
                    id="estimatedDuration"
                    type="number"
                    min="1"
                    max="480"
                    {...register("estimatedDuration", { valueAsNumber: true })}
                    placeholder="e.g., 30"
                    className={errors.estimatedDuration ? "border-red-500" : ""}
                  />
                  {errors.estimatedDuration && (
                    <p className="text-sm text-red-600">{errors.estimatedDuration.message}</p>
                  )}
                  <p className="text-xs text-muted-foreground">
                    Optional: How long do you expect this experiment to take per participant?
                  </p>
                </div>

                {/* Status */}
                <div className="space-y-2">
                  <Label htmlFor="status">Initial Status</Label>
                  <Select
                    value={watchedStatus}
                    onValueChange={(value) =>
                      setValue("status", value as "draft" | "active" | "completed" | "archived")
                    }
                  >
                    <SelectTrigger>
                      <SelectValue placeholder="Select status" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="draft">Draft - Design in progress</SelectItem>
                      <SelectItem value="active">Active - Ready for trials</SelectItem>
                      <SelectItem value="completed">Completed - Data collection finished</SelectItem>
                      <SelectItem value="archived">Archived - Experiment concluded</SelectItem>
                    </SelectContent>
                  </Select>
                </div>

                {/* Error Message */}
                {createExperimentMutation.error && (
                  <div className="rounded-md bg-red-50 p-3">
                    <p className="text-sm text-red-800">
                      Failed to create experiment: {createExperimentMutation.error.message}
                    </p>
                  </div>
                )}

                {/* Form Actions */}
                <Separator />
                <div className="flex justify-end space-x-3">
                  <Button
                    type="button"
                    variant="outline"
                    onClick={() => router.back()}
                    disabled={isSubmitting}
                  >
                    Cancel
                  </Button>
                  <Button
                    type="submit"
                    disabled={isSubmitting || studiesLoading}
                    className="min-w-[140px]"
                  >
                    {isSubmitting ? (
                      <div className="flex items-center space-x-2">
                        <svg className="h-4 w-4 animate-spin" fill="none" viewBox="0 0 24 24">
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
                      "Create & Design"
                    )}
                  </Button>
                </div>
              </form>
            </CardContent>
          </Card>
        </div>

        {/* Sidebar */}
        <div className="space-y-6">
          {/* Next Steps */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center space-x-2">
                <FlaskConical className="h-5 w-5" />
                <span>What&apos;s Next?</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3 text-sm">
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-blue-600"></div>
                  <div>
                    <p className="font-medium">Design Protocol</p>
                    <p className="text-slate-600">Use the visual designer to create experiment steps</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Configure Actions</p>
                    <p className="text-slate-600">Set up robot actions and wizard controls</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Test & Validate</p>
                    <p className="text-slate-600">Run test trials to verify the protocol</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Schedule Trials</p>
                    <p className="text-slate-600">Begin data collection with participants</p>
                  </div>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Tips */}
          <Card>
            <CardHeader>
              <CardTitle>💡 Tips</CardTitle>
            </CardHeader>
            <CardContent className="space-y-3 text-sm text-slate-600">
              <p>
                <strong>Start simple:</strong> Begin with a basic protocol and add complexity later.
              </p>
              <p>
                <strong>Plan interactions:</strong> Consider both robot behaviors and participant responses.
              </p>
              <p>
                <strong>Test early:</strong> Validate your protocol with team members before recruiting participants.
              </p>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
