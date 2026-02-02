"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { FlaskConical } from "lucide-react";
import { useState, useEffect } from "react";
import { useForm } from "react-hook-form";
import { z } from "zod";
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
import {
  EntityForm,
  FormField,
  FormSection,
  NextSteps,
  Tips,
} from "~/components/ui/entity-form";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useRouter } from "next/navigation";
import { api } from "~/trpc/react";
import { useTour } from "~/components/onboarding/TourProvider";
import { Button } from "../ui/button";

const studySchema = z.object({
  name: z.string().min(1, "Study name is required").max(255, "Name too long"),
  description: z
    .string()
    .min(10, "Description must be at least 10 characters")
    .max(1000, "Description too long"),
  institution: z
    .string()
    .min(1, "Institution is required")
    .max(255, "Institution name too long"),
  irbProtocolNumber: z.string().max(100, "Protocol number too long").optional(),
  status: z.enum(["draft", "active", "completed", "archived"]),
});

type StudyFormData = z.infer<typeof studySchema>;

interface StudyFormProps {
  mode: "create" | "edit";
  studyId?: string;
}

export function StudyForm({ mode, studyId }: StudyFormProps) {
  const router = useRouter();
  const { startTour } = useTour();
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isDeleting, setIsDeleting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<StudyFormData>({
    resolver: zodResolver(studySchema),
    defaultValues: {
      status: "draft" as const,
    },
  });

  // Fetch study data for edit mode
  const {
    data: study,
    isLoading,
    error: fetchError,
  } = api.studies.get.useQuery(
    { id: studyId! },
    { enabled: mode === "edit" && !!studyId },
  );

  // Set breadcrumbs
  const breadcrumbs = [
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(mode === "edit" && study
      ? [{ label: study.name, href: `/studies/${study.id}` }, { label: "Edit" }]
      : [{ label: "New Study" }]),
  ];

  useBreadcrumbsEffect(breadcrumbs);

  // Populate form with existing data in edit mode
  useEffect(() => {
    if (mode === "edit" && study) {
      form.reset({
        name: study.name,
        description: study.description ?? "",
        institution: study.institution ?? "",
        irbProtocolNumber: study.irbProtocol ?? "",
        status: study.status,
      });
    }
  }, [study, mode, form]);

  const createStudyMutation = api.studies.create.useMutation();
  const updateStudyMutation = api.studies.update.useMutation();
  const deleteStudyMutation = api.studies.delete.useMutation();

  // Form submission
  const onSubmit = async (data: StudyFormData) => {
    setIsSubmitting(true);
    setError(null);

    try {
      if (mode === "create") {
        const newStudy = await createStudyMutation.mutateAsync({
          name: data.name,
          description: data.description,
          institution: data.institution,
          irbProtocol: data.irbProtocolNumber ?? undefined,
        });
        router.push(`/studies/${newStudy.id}`);
      } else {
        const updatedStudy = await updateStudyMutation.mutateAsync({
          id: studyId!,
          name: data.name,
          description: data.description,
          institution: data.institution,
          irbProtocol: data.irbProtocolNumber ?? undefined,
          status: data.status,
        });
        router.push(`/studies/${updatedStudy.id}`);
      }
    } catch (error) {
      setError(
        `Failed to ${mode} study: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  // Delete handler
  const onDelete = async () => {
    if (!studyId) return;
    setIsDeleting(true);
    setError(null);

    try {
      await deleteStudyMutation.mutateAsync({ id: studyId });
      router.push("/studies");
    } catch (error) {
      setError(
        `Failed to delete study: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsDeleting(false);
    }
  };

  // Loading state for edit mode
  if (mode === "edit" && isLoading) {
    return <div>Loading study...</div>;
  }

  // Error state for edit mode
  if (mode === "edit" && fetchError) {
    return <div>Error loading study: {fetchError.message}</div>;
  }

  // Form fields
  const formFields = (
    <FormSection
      title="Study Details"
      description="Basic information about your research study."
    >
      <FormField>
        <Label htmlFor="tour-study-name">Study Name *</Label>
        <Input
          id="tour-study-name"
          {...form.register("name")}
          placeholder="Enter study name..."
          className={form.formState.errors.name ? "border-red-500" : ""}
        />
        {form.formState.errors.name && (
          <p className="text-sm text-red-600">
            {form.formState.errors.name.message}
          </p>
        )}
      </FormField>

      <FormField>
        <Label htmlFor="tour-study-description">Description *</Label>
        <Textarea
          id="tour-study-description"
          {...form.register("description")}
          placeholder="Describe the research objectives, methodology, and expected outcomes..."
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
        <Label htmlFor="institution">Institution *</Label>
        <Input
          id="institution"
          {...form.register("institution")}
          placeholder="e.g., University of Technology"
          className={form.formState.errors.institution ? "border-red-500" : ""}
        />
        {form.formState.errors.institution && (
          <p className="text-sm text-red-600">
            {form.formState.errors.institution.message}
          </p>
        )}
      </FormField>

      <FormField>
        <Label htmlFor="irbProtocolNumber">IRB Protocol Number</Label>
        <Input
          id="irbProtocolNumber"
          {...form.register("irbProtocolNumber")}
          placeholder="e.g., IRB-2024-001"
          className={
            form.formState.errors.irbProtocolNumber ? "border-red-500" : ""
          }
        />
        {form.formState.errors.irbProtocolNumber && (
          <p className="text-sm text-red-600">
            {form.formState.errors.irbProtocolNumber.message}
          </p>
        )}
        <p className="text-muted-foreground text-xs">
          Optional: Institutional Review Board protocol number if applicable
        </p>
      </FormField>

      <FormField>
        <Label htmlFor="status">Status</Label>
        <Select
          value={form.watch("status")}
          onValueChange={(value) =>
            form.setValue(
              "status",
              value as "draft" | "active" | "completed" | "archived",
            )
          }
        >
          <SelectTrigger>
            <SelectValue placeholder="Select status" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="draft">Draft - Study in preparation</SelectItem>
            <SelectItem value="active">
              Active - Currently recruiting/running
            </SelectItem>
            <SelectItem value="completed">
              Completed - Data collection finished
            </SelectItem>
            <SelectItem value="archived">Archived - Study concluded</SelectItem>
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
            title: "Invite Team Members",
            description:
              "Add researchers, wizards, and observers to collaborate",
            completed: mode === "edit",
          },
          {
            title: "Design Experiments",
            description:
              "Create experimental protocols using the visual designer",
          },
          {
            title: "Register Participants",
            description: "Add participants and manage consent forms",
          },
          {
            title: "Schedule Trials",
            description: "Begin data collection with participants",
          },
        ]}
      />
      <Tips
        tips={[
          "Define clear objectives: Well-defined research questions lead to better experimental design.",
          "Plan your team: Consider who will need access and what roles they'll have in the study.",
          "IRB approval: Make sure you have proper ethical approval before starting data collection.",
        ]}
      />
    </>
  );

  return (
    <EntityForm
      mode={mode}
      entityName="Study"
      entityNamePlural="Studies"
      backUrl="/studies"
      listUrl="/studies"
      title={
        mode === "create"
          ? "Create New Study"
          : `Edit ${study?.name ?? "Study"}`
      }
      description={
        mode === "create"
          ? "Set up a new Human-Robot Interaction research study"
          : "Update the details for this study"
      }
      icon={FlaskConical}
      form={form}
      onSubmit={onSubmit}
      isSubmitting={isSubmitting}
      error={error}
      onDelete={mode === "edit" ? onDelete : undefined}
      isDeleting={isDeleting}
      sidebar={sidebar}
      submitButtonId="tour-study-submit"
      extraActions={
        <Button variant="ghost" size="sm" onClick={() => startTour("study_creation")}>
          <div className="flex items-center gap-2">
            <span className="text-muted-foreground">Help</span>
            <div className="flex h-5 w-5 items-center justify-center rounded-full border text-xs text-muted-foreground">?</div>
          </div>
        </Button>
      }
    >
      {formFields}
    </EntityForm>
  );
}
