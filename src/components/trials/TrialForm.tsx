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

import { Calendar as CalendarIcon, Clock } from "lucide-react";
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
      <div className="grid gap-1.5">
        <Label htmlFor="date-picker" className="text-xs">Date</Label>
        <Popover open={open} onOpenChange={setOpen}>
          <PopoverTrigger asChild>
            <Button
              variant={"outline"}
              id="date-picker"
              className={cn(
                "w-[240px] justify-start text-left font-normal",
                !value && "text-muted-foreground"
              )}
            >
              <CalendarIcon className="mr-2 h-4 w-4" />
              {value ? format(value, "PPP") : <span>Pick a date</span>}
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
      </div>

      <div className="grid gap-1.5">
        <Label htmlFor="time-picker" className="text-xs">Time</Label>
        <div className="relative">
          <Input
            id="time-picker"
            type="time"
            value={timeValue}
            onChange={onTimeChange}
            disabled={!value}
            className="w-[120px]"
          />
          <Clock className="absolute right-3 top-2.5 h-4 w-4 text-muted-foreground pointer-events-none" />
        </div>
      </div>
    </div>
  );
}

const trialSchema = z.object({
  experimentId: z.string().uuid("Please select an experiment"),
  participantId: z.string().uuid("Please select a participant"),
  scheduledAt: z.date(),
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

  // Auto-increment session number
  const selectedParticipantId = form.watch("participantId");
  const { data: latestSession } = api.trials.getLatestSession.useQuery(
    { participantId: selectedParticipantId },
    {
      enabled: !!selectedParticipantId && mode === "create",
      refetchOnWindowFocus: false
    }
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
        scheduledAt: trial.scheduledAt ? new Date(trial.scheduledAt) : undefined,
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
        await createTrialMutation.mutateAsync({
          experimentId: data.experimentId,
          participantId: data.participantId,
          scheduledAt: data.scheduledAt,
          wizardId: data.wizardId,
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
      sidebar={undefined}
      submitText={mode === "create" ? "Schedule Trial" : "Save Changes"}
      layout="full-width"
    >
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        {/* Left Column: Main Info (Spans 2) */}
        <div className="md:col-span-2 space-y-6">
          <div className="grid grid-cols-1 gap-6 md:grid-cols-2">
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
          </div>

          <div className="grid grid-cols-1 gap-6 md:grid-cols-2">
            <FormField>
              <Label htmlFor="scheduledAt">Scheduled Date & Time *</Label>
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
                Auto-incremented based on participant history
              </p>
            </FormField>
          </div>
        </div>

        {/* Right Column: Assignment & Notes (Spans 1) */}
        <div className="space-y-6">
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
              Who will operate the robot?
            </p>
          </FormField>

          <FormField>
            <Label htmlFor="notes">Notes</Label>
            <Textarea
              id="notes"
              {...form.register("notes")}
              placeholder="Special instructions..."
              rows={5}
              className={form.formState.errors.notes ? "border-red-500" : ""}
            />
            {form.formState.errors.notes && (
              <p className="text-sm text-red-600">
                {form.formState.errors.notes.message}
              </p>
            )}
          </FormField>
        </div>
      </div>
    </EntityForm>
  );
}
