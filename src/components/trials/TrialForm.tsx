"use client";

import { zodResolver } from "@hookform/resolvers/zod";
import { TestTube } from "lucide-react";
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

const trialSchema = z.object({
  experimentId: z.string().uuid("Please select an experiment"),
  participantId: z.string().uuid("Please select a participant"),
  scheduledAt: z.string().min(1, "Please select a date and time"),
  wizardId: z.string().uuid().optional(),
  notes: z.string().max(1000, "Notes cannot exceed 1000 characters").optional(),
  sessionNumber: z
    .number()
    .min(1, "Session number must be at least 1")
    .optional(),
});

type TrialFormData = z.infer<typeof trialSchema>;

interface TrialFormProps {
  mode: "create" | "edit";
  trialId?: string;
  studyId?: string;
}

export function TrialForm({ mode, trialId, studyId }: TrialFormProps) {
  const router = useRouter();
  const { selectedStudyId } = useStudyContext();
  const contextStudyId = studyId ?? selectedStudyId;
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isDeleting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const form = useForm<TrialFormData>({
    resolver: zodResolver(trialSchema),
    defaultValues: {
      sessionNumber: 1,
    },
  });

  // Fetch trial data for edit mode
  const {
    data: trial,
    isLoading,
    error: fetchError,
  } = api.trials.get.useQuery(
    { id: trialId! },
    { enabled: mode === "edit" && !!trialId },
  );

  // Fetch experiments for the selected study
  const { data: experimentsData, isLoading: experimentsLoading } =
    api.experiments.list.useQuery(
      { studyId: contextStudyId! },
      { enabled: !!contextStudyId },
    );

  // Fetch participants for the selected study
  const { data: participantsData, isLoading: participantsLoading } =
    api.participants.list.useQuery(
      { studyId: contextStudyId!, limit: 100 },
      { enabled: !!contextStudyId },
    );

  // Fetch users who can be wizards
  const { data: usersData, isLoading: usersLoading } =
    api.users.getWizards.useQuery();

  // Set breadcrumbs
  const breadcrumbs = [
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    ...(contextStudyId
      ? [
        {
          label: "Study",
          href: `/studies/${contextStudyId}`,
        },
        { label: "Trials", href: `/studies/${contextStudyId}/trials` },
        ...(mode === "edit" && trial
          ? [
            {
              label: `Trial ${trial.sessionNumber || trial.id.slice(-8)}`,
              href: `/studies/${contextStudyId}/trials/${trial.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Trial" }]),
      ]
      : [
        { label: "Trials", href: `/studies/${contextStudyId}/trials` },
        ...(mode === "edit" && trial
          ? [
            {
              label: `Trial ${trial.sessionNumber || trial.id.slice(-8)}`,
              href: `/studies/${contextStudyId}/trials/${trial.id}`,
            },
            { label: "Edit" },
          ]
          : [{ label: "New Trial" }]),
      ]),
  ];

  useBreadcrumbsEffect(breadcrumbs);

  // Populate form with existing data in edit mode
  useEffect(() => {
    if (mode === "edit" && trial) {
      form.reset({
        experimentId: trial.experimentId,
        participantId: trial?.participantId ?? "",
        scheduledAt: trial.scheduledAt
          ? new Date(trial.scheduledAt).toISOString().slice(0, 16)
          : "",
        wizardId: trial.wizardId ?? undefined,
        notes: trial.notes ?? "",
        sessionNumber: trial.sessionNumber ?? 1,
      });
    }
  }, [trial, mode, form]);

  const createTrialMutation = api.trials.create.useMutation();
  const updateTrialMutation = api.trials.update.useMutation();

  // Form submission
  const onSubmit = async (data: TrialFormData) => {
    setIsSubmitting(true);
    setError(null);

    try {
      if (mode === "create") {
        const newTrial = await createTrialMutation.mutateAsync({
          experimentId: data.experimentId,
          participantId: data.participantId,
          scheduledAt: new Date(data.scheduledAt),
          wizardId: data.wizardId,
          sessionNumber: data.sessionNumber ?? 1,
          notes: data.notes ?? undefined,
        });
        router.push(`/studies/${contextStudyId}/trials/${newTrial!.id}`);
      } else {
        const updatedTrial = await updateTrialMutation.mutateAsync({
          id: trialId!,
          scheduledAt: new Date(data.scheduledAt),
          wizardId: data.wizardId,
          sessionNumber: data.sessionNumber ?? 1,
          notes: data.notes ?? undefined,
        });
        router.push(`/studies/${contextStudyId}/trials/${updatedTrial!.id}`);
      }
    } catch (error) {
      setError(
        `Failed to ${mode} trial: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  // Delete handler (trials cannot be deleted in this version)
  const onDelete = undefined;

  // Loading state for edit mode
  if (mode === "edit" && isLoading) {
    return <div>Loading trial...</div>;
  }

  // Error state for edit mode
  if (mode === "edit" && fetchError) {
    return <div>Error loading trial: {fetchError.message}</div>;
  }

  // Form fields
  const formFields = (
    <>
      <FormSection
        title="Trial Setup"
        description="Configure the basic details for this experimental trial."
      >
        <FormField>
          <Label htmlFor="experimentId">Experiment *</Label>
          <Select
            value={form.watch("experimentId")}
            onValueChange={(value) => form.setValue("experimentId", value)}
            disabled={experimentsLoading || mode === "edit"}
          >
            <SelectTrigger
              className={
                form.formState.errors.experimentId ? "border-red-500" : ""
              }
            >
              <SelectValue
                placeholder={
                  experimentsLoading
                    ? "Loading experiments..."
                    : "Select an experiment"
                }
              />
            </SelectTrigger>
            <SelectContent>
              {experimentsData?.map((experiment) => (
                <SelectItem key={experiment.id} value={experiment.id}>
                  {experiment.name}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          {form.formState.errors.experimentId && (
            <p className="text-sm text-red-600">
              {form.formState.errors.experimentId.message}
            </p>
          )}
          {mode === "edit" && (
            <p className="text-muted-foreground text-xs">
              Experiment cannot be changed after creation
            </p>
          )}
        </FormField>

        <FormField>
          <Label htmlFor="participantId">Participant *</Label>
          <Select
            value={form.watch("participantId")}
            onValueChange={(value) => form.setValue("participantId", value)}
            disabled={participantsLoading || mode === "edit"}
          >
            <SelectTrigger
              className={
                form.formState.errors.participantId ? "border-red-500" : ""
              }
            >
              <SelectValue
                placeholder={
                  participantsLoading
                    ? "Loading participants..."
                    : "Select a participant"
                }
              />
            </SelectTrigger>
            <SelectContent>
              {participantsData?.participants?.map((participant) => (
                <SelectItem key={participant.id} value={participant.id}>
                  {participant.name ?? participant.participantCode} (
                  {participant.participantCode})
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          {form.formState.errors.participantId && (
            <p className="text-sm text-red-600">
              {form.formState.errors.participantId.message}
            </p>
          )}
          {mode === "edit" && (
            <p className="text-muted-foreground text-xs">
              Participant cannot be changed after creation
            </p>
          )}
        </FormField>

        <FormField>
          <Label htmlFor="scheduledAt">Scheduled Date & Time *</Label>
          <Input
            id="scheduledAt"
            type="datetime-local"
            {...form.register("scheduledAt")}
            className={
              form.formState.errors.scheduledAt ? "border-red-500" : ""
            }
          />
          {form.formState.errors.scheduledAt && (
            <p className="text-sm text-red-600">
              {form.formState.errors.scheduledAt.message}
            </p>
          )}
          <p className="text-muted-foreground text-xs">
            When should this trial be conducted?
          </p>
        </FormField>

        <FormField>
          <Label htmlFor="sessionNumber">Session Number</Label>
          <Input
            id="sessionNumber"
            type="number"
            min="1"
            {...form.register("sessionNumber", { valueAsNumber: true })}
            placeholder="1"
            className={
              form.formState.errors.sessionNumber ? "border-red-500" : ""
            }
          />
          {form.formState.errors.sessionNumber && (
            <p className="text-sm text-red-600">
              {form.formState.errors.sessionNumber.message}
            </p>
          )}
          <p className="text-muted-foreground text-xs">
            Session number for this participant (for multi-session studies)
          </p>
        </FormField>
      </FormSection>

      <FormSection
        title="Assignment & Notes"
        description="Optional wizard assignment and trial-specific notes."
      >
        <FormField>
          <Label htmlFor="wizardId">Assigned Wizard</Label>
          <Select
            value={form.watch("wizardId") ?? "none"}
            onValueChange={(value) =>
              form.setValue("wizardId", value === "none" ? undefined : value)
            }
            disabled={usersLoading}
          >
            <SelectTrigger>
              <SelectValue
                placeholder={
                  usersLoading
                    ? "Loading wizards..."
                    : "Select a wizard (optional)"
                }
              />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="none">No wizard assigned</SelectItem>
              {usersData?.map(
                (user: { id: string; name: string; email: string }) => (
                  <SelectItem key={user.id} value={user.id}>
                    {user.name} ({user.email})
                  </SelectItem>
                ),
              )}
            </SelectContent>
          </Select>
          <p className="text-muted-foreground text-xs">
            Optional: Assign a specific wizard to operate this trial
          </p>
        </FormField>

        <FormField>
          <Label htmlFor="notes">Trial Notes</Label>
          <Textarea
            id="notes"
            {...form.register("notes")}
            placeholder="Special instructions, conditions, or notes for this trial..."
            rows={3}
            className={form.formState.errors.notes ? "border-red-500" : ""}
          />
          {form.formState.errors.notes && (
            <p className="text-sm text-red-600">
              {form.formState.errors.notes.message}
            </p>
          )}
          <p className="text-muted-foreground text-xs">
            Optional: Notes about special conditions, instructions, or context
            for this trial
          </p>
        </FormField>
      </FormSection>
    </>
  );

  // Sidebar content
  const sidebar = (
    <>
      <NextSteps
        steps={[
          {
            title: "Execute Trial",
            description: "Use the wizard interface to run the trial",
            completed: mode === "edit",
          },
          {
            title: "Monitor Progress",
            description: "Track trial execution and data collection",
          },
          {
            title: "Review Data",
            description: "Analyze collected trial data and results",
          },
          {
            title: "Generate Reports",
            description: "Export data and create analysis reports",
          },
        ]}
      />
      <Tips
        tips={[
          "Schedule ahead: Allow sufficient time between trials for setup and data review.",
          "Assign wizards: Pre-assign experienced wizards to complex trials.",
          "Document conditions: Use notes to record any special circumstances or variations.",
          "Test connectivity: Verify robot and system connections before scheduled trials.",
        ]}
      />
    </>
  );

  return (
    <EntityForm
      mode={mode}
      entityName="Trial"
      entityNamePlural="Trials"
      backUrl={`/studies/${contextStudyId}/trials`}
      listUrl={`/studies/${contextStudyId}/trials`}
      title={
        mode === "create"
          ? "Schedule New Trial"
          : `Edit ${trial ? `Trial ${trial.sessionNumber || trial.id.slice(-8)}` : "Trial"}`
      }
      description={
        mode === "create"
          ? "Schedule a new experimental trial with a participant"
          : "Update trial scheduling and assignment details"
      }
      icon={TestTube}
      form={form}
      onSubmit={onSubmit}
      isSubmitting={isSubmitting}
      error={error}
      onDelete={
        mode === "edit" && trial?.status === "scheduled" ? onDelete : undefined
      }
      isDeleting={isDeleting}
      sidebar={sidebar}
      submitText={mode === "create" ? "Schedule Trial" : "Save Changes"}
    >
      {formFields}
    </EntityForm>
  );
}
