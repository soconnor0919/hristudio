"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { Users } from "lucide-react";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { useForm } from "react-hook-form";
import { z } from "zod";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { Checkbox } from "~/components/ui/checkbox";
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
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";

type DemographicsData = {
  age?: number;
  gender?: string;
  occupation?: string;
  education?: string;
  primaryLanguage?: string;
  language?: string;
  location?: string;
  city?: string;
  robotExperience?: string;
  experience?: string;
  grade?: number;
};

const participantSchema = z.object({
  participantCode: z
    .string()
    .min(1, "Participant code is required")
    .max(50, "Code too long")
    .regex(
      /^[A-Za-z0-9_-]+$/,
      "Code can only contain letters, numbers, hyphens, and underscores",
    ),
  name: z.string().max(100, "Name too long").optional(),
  email: z.string().email("Invalid email format").optional().or(z.literal("")),
  studyId: z.string().uuid("Please select a study"),
  age: z
    .number()
    .min(18, "Participant must be at least 18 years old")
    .max(120, "Invalid age")
    .optional(),
  gender: z
    .enum(["male", "female", "non_binary", "prefer_not_to_say", "other"])
    .optional(),
  consentGiven: z.boolean().refine((val) => val === true, {
    message: "Consent must be given before registration",
  }),
});

type ParticipantFormData = z.infer<typeof participantSchema>;

interface ParticipantFormProps {
  mode: "create" | "edit";
  participantId?: string;
  studyId?: string;
}

export function ParticipantForm({
  mode,
  participantId,
  studyId,
}: ParticipantFormProps) {
  const router = useRouter();
  const { selectedStudyId } = useStudyContext();
  const contextStudyId = studyId ?? selectedStudyId;
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isDeleting, setIsDeleting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<ParticipantFormData>({
    resolver: zodResolver(participantSchema),
    defaultValues: {
      consentGiven: false,
      studyId: contextStudyId ?? "",
    },
  });

  // Fetch participant data for edit mode
  const {
    data: participant,
    isLoading,
    error: fetchError,
  } = api.participants.get.useQuery(
    { id: participantId! },
    { enabled: mode === "edit" && !!participantId },
  );

  // Fetch user's studies for the dropdown
  const { data: studiesData, isLoading: studiesLoading } =
    api.studies.list.useQuery({ memberOnly: true });

  // Set breadcrumbs
  const breadcrumbs = [
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(contextStudyId
      ? [
        {
          label: participant?.study?.name ?? "Study",
          href: `/studies/${contextStudyId}`,
        },
        {
          label: "Participants",
          href: `/studies/${contextStudyId}/participants`,
        },
        ...(mode === "edit" && participant
          ? [
            {
              label: participant.name ?? participant.participantCode,
              href: `/studies/${contextStudyId}/participants/${participant.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Participant" }]),
      ]
      : [
        {
          label: "Participants",
          href: `/studies/${contextStudyId}/participants`,
        },
        ...(mode === "edit" && participant
          ? [
            {
              label: participant.name ?? participant.participantCode,
              href: `/studies/${contextStudyId}/participants/${participant.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Participant" }]),
      ]),
  ];

  useBreadcrumbsEffect(breadcrumbs);

  // Populate form with existing data in edit mode
  useEffect(() => {
    if (mode === "edit" && participant) {
      form.reset({
        participantCode: participant.participantCode,
        name: participant.name ?? "",
        email: participant.email ?? "",
        studyId: participant.studyId,
        age: (participant.demographics as DemographicsData)?.age ?? undefined,
        gender:
          ((participant.demographics as DemographicsData)?.gender as
            | "male"
            | "female"
            | "non_binary"
            | "prefer_not_to_say"
            | "other"
            | undefined) ?? undefined,
        consentGiven: true, // Assume consent was given if participant exists
      });
    }
  }, [participant, mode, form]);

  // Update studyId when contextStudyId changes (for create mode)
  useEffect(() => {
    if (mode === "create" && contextStudyId) {
      form.setValue("studyId", contextStudyId);
    }
  }, [contextStudyId, mode, form]);

  const createParticipantMutation = api.participants.create.useMutation();
  const updateParticipantMutation = api.participants.update.useMutation();
  const deleteParticipantMutation = api.participants.delete.useMutation();

  // Form submission
  const onSubmit = async (data: ParticipantFormData) => {
    setIsSubmitting(true);
    setError(null);

    try {
      const demographics = {
        age: data.age ?? null,
        gender: data.gender ?? null,
      };

      if (mode === "create") {
        const newParticipant = await createParticipantMutation.mutateAsync({
          studyId: data.studyId,
          participantCode: data.participantCode,
          name: data.name ?? undefined,
          email: data.email ?? undefined,
          demographics,
        });
        router.push(`/studies/${data.studyId}/participants/${newParticipant.id}`);
      } else {
        const updatedParticipant = await updateParticipantMutation.mutateAsync({
          id: participantId!,
          participantCode: data.participantCode,
          name: data.name ?? undefined,
          email: data.email ?? undefined,
          demographics,
        });
        router.push(`/studies/${contextStudyId}/participants/${updatedParticipant.id}`);
      }
    } catch (error) {
      setError(
        `Failed to ${mode} participant: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  // Delete handler
  const onDelete = async () => {
    if (!participantId) return;
    setIsDeleting(true);
    setError(null);

    try {
      await deleteParticipantMutation.mutateAsync({ id: participantId });
      router.push(`/studies/${contextStudyId}/participants`);
    } catch (error) {
      setError(
        `Failed to delete participant: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsDeleting(false);
    }
  };

  // Loading state for edit mode
  if (mode === "edit" && isLoading) {
    return <div>Loading participant...</div>;
  }

  // Error state for edit mode
  if (mode === "edit" && fetchError) {
    return <div>Error loading participant: {fetchError.message}</div>;
  }

  // Form fields
  const formFields = (
    <>
      <FormSection
        title="Participant Information"
        description="Basic identity and study association."
      >
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <FormField>
            <Label htmlFor="participantCode">Participant Code *</Label>
            <Input
              id="participantCode"
              {...form.register("participantCode")}
              placeholder="e.g., P001"
              className={
                form.formState.errors.participantCode ? "border-red-500" : ""
              }
            />
            {form.formState.errors.participantCode && (
              <p className="text-sm text-red-600">
                {form.formState.errors.participantCode.message}
              </p>
            )}
          </FormField>

          <FormField>
            <Label htmlFor="name">Full Name</Label>
            <Input
              id="name"
              {...form.register("name")}
              placeholder="Optional name"
              className={form.formState.errors.name ? "border-red-500" : ""}
            />
            {form.formState.errors.name && (
              <p className="text-sm text-red-600">
                {form.formState.errors.name.message}
              </p>
            )}
          </FormField>

          <FormField>
            <Label htmlFor="email">Email Address</Label>
            <Input
              id="email"
              type="email"
              {...form.register("email")}
              placeholder="participant@example.com"
              className={form.formState.errors.email ? "border-red-500" : ""}
            />
            {form.formState.errors.email && (
              <p className="text-sm text-red-600">
                {form.formState.errors.email.message}
              </p>
            )}
          </FormField>
        </div>
      </FormSection>

      <div className="my-6" />

      <FormSection
        title="Demographics & Study"
        description="study association and demographic details."
      >
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <FormField>
            <Label htmlFor="studyId">Study *</Label>
            <Select
              value={form.watch("studyId")}
              onValueChange={(value) => form.setValue("studyId", value)}
              disabled={studiesLoading || mode === "edit"}
            >
              <SelectTrigger
                className={
                  form.formState.errors.studyId ? "border-red-500" : ""
                }
              >
                <SelectValue
                  placeholder={
                    studiesLoading ? "Loading..." : "Select study"
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
          </FormField>

          <FormField>
            <Label htmlFor="age">Age</Label>
            <Input
              id="age"
              type="number"
              min="18"
              max="120"
              {...form.register("age", { valueAsNumber: true })}
              placeholder="e.g., 25"
              className={form.formState.errors.age ? "border-red-500" : ""}
            />
            {form.formState.errors.age && (
              <p className="text-sm text-red-600">
                {form.formState.errors.age.message}
              </p>
            )}
          </FormField>

          <FormField>
            <Label htmlFor="gender">Gender</Label>
            <Select
              value={form.watch("gender") ?? ""}
              onValueChange={(value) =>
                form.setValue(
                  "gender",
                  value as
                  | "male"
                  | "female"
                  | "non_binary"
                  | "prefer_not_to_say"
                  | "other",
                )
              }
            >
              <SelectTrigger>
                <SelectValue placeholder="Select gender" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="male">Male</SelectItem>
                <SelectItem value="female">Female</SelectItem>
                <SelectItem value="non_binary">Non-binary</SelectItem>
                <SelectItem value="prefer_not_to_say">
                  Prefer not to say
                </SelectItem>
                <SelectItem value="other">Other</SelectItem>
              </SelectContent>
            </Select>
          </FormField>
        </div>
      </FormSection>

      {mode === "create" && (
        <FormSection
          title="Consent"
          description="Participant consent and agreement to participate."
        >
          <FormField>
            <div className="flex items-center space-x-2">
              <Checkbox
                id="consentGiven"
                checked={form.watch("consentGiven")}
                onCheckedChange={(checked) =>
                  form.setValue("consentGiven", !!checked)
                }
              />
              <Label htmlFor="consentGiven" className="text-sm">
                I confirm that the participant has given informed consent to
                participate in this study *
              </Label>
            </div>
            {form.formState.errors.consentGiven && (
              <p className="text-sm text-red-600">
                {form.formState.errors.consentGiven.message}
              </p>
            )}
            <p className="text-muted-foreground text-xs">
              Required: Confirmation that proper consent procedures have been
              followed
            </p>
          </FormField>
        </FormSection>
      )}
    </>
  );

  // Sidebar content
  const sidebar = (
    <>
      <NextSteps
        steps={[
          {
            title: "Schedule Trials",
            description: "Assign participant to experimental trials",
            completed: mode === "edit",
          },
          {
            title: "Collect Data",
            description: "Execute trials and gather research data",
          },
          {
            title: "Monitor Progress",
            description: "Track participation and completion status",
          },
          {
            title: "Analyze Results",
            description: "Review participant data and outcomes",
          },
        ]}
      />
      <Tips
        tips={[
          "Use consistent codes: Establish a clear naming convention for participant codes.",
          "Protect privacy: Minimize collection of personally identifiable information.",
          "Verify consent: Ensure all consent forms are properly completed before registration.",
          "Plan ahead: Consider how many participants you'll need for statistical significance.",
        ]}
      />
    </>
  );

  return (
    <EntityForm
      mode={mode}
      entityName="Participant"
      entityNamePlural="Participants"
      backUrl={`/studies/${contextStudyId}/participants`}
      listUrl={`/studies/${contextStudyId}/participants`}
      title={
        mode === "create"
          ? "Register New Participant"
          : `Edit ${participant?.name ?? participant?.participantCode ?? "Participant"}`
      }
      description={
        mode === "create"
          ? "Register a new participant for your research study"
          : "Update participant information and demographics"
      }
      icon={Users}
      form={form}
      onSubmit={onSubmit}
      isSubmitting={isSubmitting}
      error={error}
      onDelete={mode === "edit" ? onDelete : undefined}
      isDeleting={isDeleting}
      sidebar={mode === "create" ? sidebar : undefined}
      submitText={mode === "create" ? "Register Participant" : "Save Changes"}
    >
      {formFields}
    </EntityForm>
  );
}
