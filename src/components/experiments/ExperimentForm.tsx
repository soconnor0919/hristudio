"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { FlaskConical } from "lucide-react";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { useForm } from "react-hook-form";
import { z } from "zod";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import {
  EntityForm,
  FormField,
  FormSection,
  NextSteps,
  Tips,
} from "~/components/ui/entity-form";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Textarea } from "~/components/ui/textarea";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";

const experimentSchema = z.object({
  name: z
    .string()
    .min(1, "Experiment name is required")
    .max(100, "Name too long"),
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
  status: z.enum(["draft", "testing", "ready", "deprecated"]),
});

type ExperimentFormData = z.infer<typeof experimentSchema>;

interface ExperimentFormProps {
  mode: "create" | "edit";
  experimentId?: string;
}

export function ExperimentForm({ mode, experimentId }: ExperimentFormProps) {
  const router = useRouter();
  const { selectedStudyId } = useStudyContext();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isDeleting, setIsDeleting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<ExperimentFormData>({
    resolver: zodResolver(experimentSchema),
    defaultValues: {
      status: "draft" as const,
      studyId: selectedStudyId ?? "",
    },
  });

  // Fetch experiment data for edit mode
  const {
    data: experiment,
    isLoading,
    error: fetchError,
  } = api.experiments.get.useQuery(
    { id: experimentId! },
    { enabled: mode === "edit" && !!experimentId },
  );

  // Fetch user's studies for the dropdown
  const { data: studiesData, isLoading: studiesLoading } =
    api.studies.list.useQuery({ memberOnly: true });

  // Set breadcrumbs
  const breadcrumbs = [
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(selectedStudyId
      ? [
        {
          label: experiment?.study?.name ?? "Study",
          href: `/studies/${selectedStudyId}`,
        },
        { label: "Experiments", href: "/experiments" },
        ...(mode === "edit" && experiment
          ? [
            {
              label: experiment.name,
              href: `/studies/${selectedStudyId}/experiments/${experiment.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Experiment" }]),
      ]
      : [
        { label: "Experiments", href: "/experiments" },
        ...(mode === "edit" && experiment
          ? [
            {
              label: experiment.name,
              href: `/studies/${experiment.studyId}/experiments/${experiment.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Experiment" }]),
      ]),
  ];

  useBreadcrumbsEffect(breadcrumbs);

  // Populate form with existing data in edit mode
  useEffect(() => {
    if (mode === "edit" && experiment) {
      form.reset({
        name: experiment.name,
        description: experiment.description ?? "",
        studyId: experiment.studyId,
        estimatedDuration: experiment.estimatedDuration ?? undefined,
        status: experiment.status,
      });
    }
  }, [experiment, mode, form]);

  // Update studyId when selectedStudyId changes (for create mode)
  useEffect(() => {
    if (mode === "create" && selectedStudyId) {
      form.setValue("studyId", selectedStudyId);
    }
  }, [selectedStudyId, mode, form]);

  const createExperimentMutation = api.experiments.create.useMutation();
  const updateExperimentMutation = api.experiments.update.useMutation();
  const deleteExperimentMutation = api.experiments.delete.useMutation();

  // Form submission
  const onSubmit = async (data: ExperimentFormData) => {
    setIsSubmitting(true);
    setError(null);

    try {
      if (mode === "create") {
        const newExperiment = await createExperimentMutation.mutateAsync({
          ...data,
          estimatedDuration: data.estimatedDuration ?? undefined,
        });
        router.push(`/studies/${data.studyId}/experiments/${newExperiment.id}/designer`);
      } else {
        const updatedExperiment = await updateExperimentMutation.mutateAsync({
          id: experimentId!,
          ...data,
          estimatedDuration: data.estimatedDuration ?? undefined,
        });
        router.push(`/studies/${experiment?.studyId ?? data.studyId}/experiments/${updatedExperiment.id}`);
      }
    } catch (error) {
      setError(
        `Failed to ${mode} experiment: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  // Delete handler
  const onDelete = async () => {
    if (!experimentId) return;
    setIsDeleting(true);
    setError(null);

    try {
      await deleteExperimentMutation.mutateAsync({ id: experimentId });
      router.push("/experiments");
    } catch (error) {
      setError(
        `Failed to delete experiment: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsDeleting(false);
    }
  };

  // Loading state for edit mode
  if (mode === "edit" && isLoading) {
    return <div>Loading experiment...</div>;
  }

  // Error state for edit mode
  if (mode === "edit" && fetchError) {
    return <div>Error loading experiment: {fetchError.message}</div>;
  }

  // Form fields
  const formFields = (
    <FormSection
      title="Experiment Details"
      description="Define the basic information for your experiment protocol."
    >
      <FormField>
        <Label htmlFor="name">Experiment Name *</Label>
        <Input
          id="name"
          {...form.register("name")}
          placeholder="Enter experiment name..."
          className={form.formState.errors.name ? "border-red-500" : ""}
        />
        {form.formState.errors.name && (
          <p className="text-sm text-red-600">
            {form.formState.errors.name.message}
          </p>
        )}
      </FormField>

      <FormField>
        <Label htmlFor="description">Description *</Label>
        <Textarea
          id="description"
          {...form.register("description")}
          placeholder="Describe the experiment objectives, methodology, and expected outcomes..."
          rows={4}
          className={form.formState.errors.description ? "border-red-500" : ""}
        />
        {form.formState.errors.description && (
          <p className="text-sm text-red-600">
            {form.formState.errors.description.message}
          </p>
        )}
      </FormField>

      <FormField>
        <Label htmlFor="studyId">Study *</Label>
        <Select
          value={form.watch("studyId")}
          onValueChange={(value) => form.setValue("studyId", value)}
          disabled={studiesLoading || mode === "edit"}
        >
          <SelectTrigger
            className={form.formState.errors.studyId ? "border-red-500" : ""}
          >
            <SelectValue
              placeholder={
                studiesLoading ? "Loading studies..." : "Select a study"
              }
            />
          </SelectTrigger>
          <SelectContent>
            {studiesData?.studies?.map((study) => (
              <SelectItem key={study.id} value={study.id}>
                {study.name}
              </SelectItem>
            ))}
          </SelectContent>
        </Select>
        {form.formState.errors.studyId && (
          <p className="text-sm text-red-600">
            {form.formState.errors.studyId.message}
          </p>
        )}
        {mode === "edit" && (
          <p className="text-muted-foreground text-xs">
            Study cannot be changed after creation
          </p>
        )}
      </FormField>

      <FormField>
        <Label htmlFor="estimatedDuration">Estimated Duration (minutes)</Label>
        <Input
          id="estimatedDuration"
          type="number"
          min="1"
          max="480"
          {...form.register("estimatedDuration", { valueAsNumber: true })}
          placeholder="e.g., 30"
          className={
            form.formState.errors.estimatedDuration ? "border-red-500" : ""
          }
        />
        {form.formState.errors.estimatedDuration && (
          <p className="text-sm text-red-600">
            {form.formState.errors.estimatedDuration.message}
          </p>
        )}
        <p className="text-muted-foreground text-xs">
          Optional: How long do you expect this experiment to take per
          participant?
        </p>
      </FormField>

      <FormField>
        <Label htmlFor="status">Status</Label>
        <Select
          value={form.watch("status")}
          onValueChange={(value) =>
            form.setValue(
              "status",
              value as "draft" | "testing" | "ready" | "deprecated",
            )
          }
        >
          <SelectTrigger>
            <SelectValue placeholder="Select status" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="draft">Draft - Design in progress</SelectItem>
            <SelectItem value="testing">
              Testing - Protocol validation
            </SelectItem>
            <SelectItem value="ready">Ready - Available for trials</SelectItem>
            <SelectItem value="deprecated">
              Deprecated - No longer used
            </SelectItem>
          </SelectContent>
        </Select>
      </FormField>
    </FormSection>
  );

  // Sidebar content
  const sidebar = (
    <>
      <NextSteps
        steps={[
          {
            title: "Design Protocol",
            description: "Use the visual designer to create experiment steps",
            completed: mode === "edit",
          },
          {
            title: "Configure Actions",
            description: "Set up robot actions and wizard controls",
          },
          {
            title: "Test & Validate",
            description: "Run test trials to verify the protocol",
          },
          {
            title: "Schedule Trials",
            description: "Begin data collection with participants",
          },
        ]}
      />
      <Tips
        tips={[
          "Start simple: Begin with a basic protocol and add complexity later.",
          "Plan interactions: Consider both robot behaviors and participant responses.",
          "Test early: Validate your protocol with team members before recruiting participants.",
          "Document thoroughly: Clear descriptions help team members understand the protocol.",
        ]}
      />
    </>
  );

  return (
    <EntityForm
      mode={mode}
      entityName="Experiment"
      entityNamePlural="Experiments"
      backUrl="/experiments"
      listUrl="/experiments"
      title={
        mode === "create"
          ? "Create New Experiment"
          : `Edit ${experiment?.name ?? "Experiment"}`
      }
      description={
        mode === "create"
          ? "Design a new experimental protocol for your HRI study"
          : "Update the details for this experiment"
      }
      icon={FlaskConical}
      form={form}
      onSubmit={onSubmit}
      isSubmitting={isSubmitting}
      error={error}
      onDelete={mode === "edit" ? onDelete : undefined}
      isDeleting={isDeleting}
      sidebar={sidebar}
      submitText={mode === "create" ? "Create & Design" : "Save Changes"}
    >
      {formFields}
    </EntityForm>
  );
}
