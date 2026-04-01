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

import { Calendar as CalendarIcon, Clock, Clock2 } from "lucide-react";
import { format } from "date-fns";
import { cn } from "~/lib/utils";
import { Button } from "~/components/ui/button";
import { Calendar } from "~/components/ui/calendar";
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from "~/components/ui/popover";
import { Controller } from "react-hook-form";

// Custom DatePickerTime component based on user request
function DateTimePicker({
  value,
  onChange,
}: {
  value: Date | undefined;
  onChange: (date: Date | undefined) => void;
}) {
  const [open, setOpen] = useState(false);

  // Parse time from value or default
  const timeValue = value ? format(value, "HH:mm") : "12:00";

  const onDateSelect = (newDate: Date | undefined) => {
    if (!newDate) {
      onChange(undefined);
      setOpen(false);
      return;
    }

    // Preserve existing time or use default
    const [hours, minutes] = timeValue.split(":").map(Number);
    const updatedDate = new Date(newDate);
    updatedDate.setHours(hours || 0);
    updatedDate.setMinutes(minutes || 0);
    updatedDate.setSeconds(0);

    onChange(updatedDate);
    setOpen(false);
  };

  const onTimeChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newTime = e.target.value;
    if (!value) return; // Can't set time without date

    const [hours, minutes] = newTime.split(":").map(Number);
    const updatedDate = new Date(value);
    updatedDate.setHours(hours || 0);
    updatedDate.setMinutes(minutes || 0);
    updatedDate.setSeconds(0);

    onChange(updatedDate);
  };

  return (
    <div className="flex items-end gap-2">
      <Popover open={open} onOpenChange={setOpen}>
        <PopoverTrigger asChild>
          <Button
            variant={"outline"}
            id="date-picker"
            className={cn(
              "w-[200px] justify-start text-left font-normal",
              !value && "text-muted-foreground",
            )}
          >
            <CalendarIcon className="mr-2 h-4 w-4" />
            {value ? format(value, "MMM d, yyyy") : <span>Pick a date</span>}
          </Button>
        </PopoverTrigger>
        <PopoverContent className="w-auto p-0" align="start">
          <Calendar
            mode="single"
            selected={value}
            onSelect={onDateSelect}
            initialFocus
          />
        </PopoverContent>
      </Popover>

      <div className="relative">
        <Input
          id="time-picker"
          type="time"
          value={timeValue}
          onChange={onTimeChange}
          disabled={!value}
          className="w-[110px]"
        />
        <Clock className="text-muted-foreground pointer-events-none absolute top-2.5 right-3 h-4 w-4" />
      </div>

      <Button
        type="button"
        variant="outline"
        size="sm"
        onClick={() => onChange(new Date())}
        className="h-10 gap-1.5"
      >
        <Clock2 className="h-4 w-4" />
        Now
      </Button>
    </div>
  );
}

const trialSchema = z.object({
  experimentId: z.string().min(1, "Please select an experiment *"),
  participantId: z.string().min(1, "Please select a participant *"),
  scheduledAt: z.date({ message: "Scheduled date and time is required *" }),
  wizardId: z.string().optional().or(z.literal("")),
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
  const [error, setError] = useState<string | null>(null);

  const form = useForm<TrialFormData>({
    resolver: zodResolver(trialSchema),
    defaultValues: {
      experimentId: "" as any,
      participantId: "" as any,
      scheduledAt: undefined,
      wizardId: undefined,
      notes: "",
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

  // Auto-increment session number
  const selectedParticipantId = form.watch("participantId");
  const { data: latestSession } = api.trials.getLatestSession.useQuery(
    { participantId: selectedParticipantId },
    {
      enabled: !!selectedParticipantId && mode === "create",
      refetchOnWindowFocus: false,
    },
  );

  useEffect(() => {
    if (latestSession !== undefined && mode === "create") {
      form.setValue("sessionNumber", latestSession + 1);
    }
  }, [latestSession, mode, form]);

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
          ? new Date(trial.scheduledAt)
          : undefined,
        wizardId: trial.wizardId ?? undefined,
        notes: trial.notes ?? "",
        sessionNumber: trial.sessionNumber ?? 1,
      });
    }
  }, [trial, mode, form]);

  const createTrialMutation = api.trials.create.useMutation({
    onError: (error) => {
      console.error("Create trial error:", error);
      setError(error.message);
    },
  });
  const updateTrialMutation = api.trials.update.useMutation({
    onError: (error) => {
      console.error("Update trial error:", error);
      setError(error.message);
    },
  });

  // Form submission
  const onSubmit = async (data: TrialFormData) => {
    setIsSubmitting(true);
    setError(null);

    try {
      if (mode === "create") {
        await createTrialMutation.mutateAsync({
          experimentId: data.experimentId,
          participantId: data.participantId,
          scheduledAt: data.scheduledAt,
          wizardId: data.wizardId || undefined,
          sessionNumber: data.sessionNumber ?? 1,
          notes: data.notes ?? undefined,
        });
        // Redirect to trials table instead of detail page
        router.push(`/studies/${contextStudyId}/trials`);
      } else {
        await updateTrialMutation.mutateAsync({
          id: trialId!,
          scheduledAt: data.scheduledAt,
          wizardId: data.wizardId,
          sessionNumber: data.sessionNumber ?? 1,
          notes: data.notes ?? undefined,
        });
        // Redirect to trials table on update too
        router.push(`/studies/${contextStudyId}/trials`);
      }
    } catch (error) {
      setError(
        `Failed to ${mode} trial: ${error instanceof Error ? error.message : "Unknown error"}`,
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  // Loading state for edit mode
  if (mode === "edit" && isLoading) {
    return <div>Loading trial...</div>;
  }

  // Error state for edit mode
  if (mode === "edit" && fetchError) {
    return <div>Error loading trial: {fetchError.message}</div>;
  }

  // Sidebar content
  const sidebar = (
    <>
      <NextSteps
        steps={[
          {
            title: "Configure Experiment",
            description: "Ensure the experiment protocol is designed and ready",
            completed: !!form.watch("experimentId"),
          },
          {
            title: "Select Participant",
            description: "Choose a participant for this trial",
            completed: !!form.watch("participantId"),
          },
          {
            title: "Assign Wizard",
            description: "Assign a wizard to operate the robot",
          },
          {
            title: "Run Trial",
            description: "Execute the trial and collect data",
          },
        ]}
      />
      <Tips
        tips={[
          "Verify experiment status: Only 'Ready' experiments can be used in trials.",
          "Check participant availability: Ensure participants are available at the scheduled time.",
          "Assign wizards early: Give wizards time to prepare before the trial.",
          "Prepare notes: Add any special instructions for the wizard.",
        ]}
      />
    </>
  );

  // Form fields
  const formFields = (
    <>
      <FormSection
        title="Trial Configuration"
        description="Select the experiment and participant for this trial."
      >
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2">
          <FormField>
            <Label htmlFor="experimentId">
              Experiment <span className="text-red-500">*</span>
            </Label>
            <Select
              value={form.watch("experimentId") ?? ""}
              onValueChange={(value) => form.setValue("experimentId", value)}
              disabled={experimentsLoading || mode === "edit"}
            >
              <SelectTrigger
                className={
                  form.formState.errors.experimentId ? "border-red-500 ring-1 ring-red-500" : ""
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
                    {experiment.name} ({experiment.status})
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            {form.formState.errors.experimentId && (
              <p className="mt-1 text-sm text-red-500 font-medium">
                {form.formState.errors.experimentId.message}
              </p>
            )}
            {mode === "edit" && (
              <p className="text-muted-foreground mt-1 text-xs">
                Experiment cannot be changed after creation
              </p>
            )}
          </FormField>

          <FormField>
            <Label htmlFor="participantId">
              Participant <span className="text-red-500">*</span>
            </Label>
            <Select
              value={form.watch("participantId") ?? ""}
              onValueChange={(value) => form.setValue("participantId", value)}
              disabled={participantsLoading || mode === "edit"}
            >
              <SelectTrigger
                className={
                  form.formState.errors.participantId ? "border-red-500 ring-1 ring-red-500" : ""
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
                    {participant.name ?? participant.participantCode}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            {form.formState.errors.participantId && (
              <p className="mt-1 text-sm text-red-500 font-medium">
                {form.formState.errors.participantId.message}
              </p>
            )}
            {mode === "edit" && (
              <p className="text-muted-foreground mt-1 text-xs">
                Participant cannot be changed after creation
              </p>
            )}
          </FormField>
        </div>
      </FormSection>

      <FormSection
        title="Scheduling"
        description="Set when this trial should be conducted."
      >
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2">
          <FormField>
            <Label htmlFor="scheduledAt">
              Scheduled Date & Time <span className="text-red-500">*</span>
            </Label>
            <Controller
              control={form.control}
              name="scheduledAt"
              render={({ field }) => (
                <DateTimePicker
                  value={field.value}
                  onChange={field.onChange}
                />
              )}
            />
            {form.formState.errors.scheduledAt && (
              <p className="mt-1 text-sm text-red-500 font-medium">
                {form.formState.errors.scheduledAt.message}
              </p>
            )}
            <p className="text-muted-foreground mt-1 text-xs">
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
                form.formState.errors.sessionNumber ? "border-red-500 ring-1 ring-red-500" : ""
              }
            />
            {form.formState.errors.sessionNumber && (
              <p className="mt-1 text-sm text-red-500 font-medium">
                {form.formState.errors.sessionNumber.message}
              </p>
            )}
            <p className="text-muted-foreground mt-1 text-xs">
              Auto-incremented based on participant history
            </p>
          </FormField>
        </div>
      </FormSection>

      <FormSection
        title="Assignment & Notes"
        description="Assign a wizard and add any special instructions."
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
          <p className="text-muted-foreground mt-1 text-xs">
            Who will operate the robot during this trial?
          </p>
        </FormField>

        <FormField>
          <Label htmlFor="notes">Notes</Label>
          <Textarea
            id="notes"
            {...form.register("notes")}
            placeholder="Special instructions for the wizard, environmental setup notes, or other relevant information..."
            rows={4}
            className={form.formState.errors.notes ? "border-red-500 ring-1 ring-red-500" : ""}
          />
          {form.formState.errors.notes && (
            <p className="mt-1 text-sm text-red-500 font-medium">
              {form.formState.errors.notes.message}
            </p>
          )}
          <p className="text-muted-foreground mt-1 text-xs">
            Optional: Add any special instructions for this trial
          </p>
        </FormField>
      </FormSection>
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
      sidebar={sidebar}
      submitText={mode === "create" ? "Schedule Trial" : "Save Changes"}
    >
      {formFields}
    </EntityForm>
  );
}
