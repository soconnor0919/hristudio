"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { useForm } from "react-hook-form";
import { zodResolver } from "@hookform/resolvers/zod";
import { z } from "zod";
import Link from "next/link";
import { ArrowLeft, Calendar, Users, FlaskConical, Clock } from "lucide-react";
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

const createTrialSchema = z.object({
  experimentId: z.string().uuid("Please select an experiment"),
  participantId: z.string().uuid("Please select a participant"),
  scheduledAt: z.string().min(1, "Please select a date and time"),
  wizardId: z.string().uuid().optional(),
  notes: z.string().max(1000, "Notes cannot exceed 1000 characters").optional(),
});

type CreateTrialFormData = z.infer<typeof createTrialSchema>;

export default function NewTrialPage() {
  const router = useRouter();
  const [isSubmitting, setIsSubmitting] = useState(false);

  const {
    register,
    handleSubmit,
    setValue,
    watch,
    formState: { errors },
  } = useForm<CreateTrialFormData>({
    resolver: zodResolver(createTrialSchema),
  });

  // Fetch available experiments
  const { data: experimentsData, isLoading: experimentsLoading } = api.experiments.getUserExperiments.useQuery(
    { page: 1, limit: 100 },
  );

  // Fetch available participants
  const { data: participantsData, isLoading: participantsLoading } = api.participants.list.useQuery(
    { page: 1, limit: 100 },
  );

  // Fetch potential wizards (users with wizard or researcher roles)
  const { data: wizardsData, isLoading: wizardsLoading } = api.users.getWizards.useQuery();

  const createTrialMutation = api.trials.create.useMutation({
    onSuccess: (trial) => {
      router.push(`/trials/${trial.id}`);
    },
    onError: (error) => {
      console.error("Failed to create trial:", error);
      setIsSubmitting(false);
    },
  });

  const onSubmit = async (data: CreateTrialFormData) => {
    setIsSubmitting(true);
    try {
      await createTrialMutation.mutateAsync({
        ...data,
        scheduledAt: new Date(data.scheduledAt),
        wizardId: data.wizardId || null,
        notes: data.notes || null,
      });
    } catch (error) {
      // Error handling is done in the mutation's onError callback
    }
  };

  const watchedExperimentId = watch("experimentId");
  const watchedParticipantId = watch("participantId");
  const watchedWizardId = watch("wizardId");

  const selectedExperiment = experimentsData?.experiments?.find(
    exp => exp.id === watchedExperimentId
  );

  const selectedParticipant = participantsData?.participants?.find(
    p => p.id === watchedParticipantId
  );

  // Generate datetime-local input min value (current time)
  const now = new Date();
  const minDateTime = new Date(now.getTime() - now.getTimezoneOffset() * 60000)
    .toISOString()
    .slice(0, 16);

  return (
    <div className="p-8">
      {/* Header */}
      <div className="mb-8">
        <div className="flex items-center space-x-2 text-sm text-slate-600 mb-4">
          <Link href="/trials" className="hover:text-slate-900 flex items-center">
            <ArrowLeft className="h-4 w-4 mr-1" />
            Trials
          </Link>
          <span>/</span>
          <span className="text-slate-900">Schedule New Trial</span>
        </div>

        <div className="flex items-center space-x-3">
          <div className="flex h-12 w-12 items-center justify-center rounded-lg bg-green-100">
            <Calendar className="h-6 w-6 text-green-600" />
          </div>
          <div>
            <h1 className="text-3xl font-bold text-slate-900">Schedule New Trial</h1>
            <p className="text-slate-600">Set up a research trial with a participant and experiment</p>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Main Form */}
        <div className="lg:col-span-2">
          <Card>
            <CardHeader>
              <CardTitle>Trial Details</CardTitle>
              <CardDescription>
                Configure the experiment, participant, and scheduling for this trial session.
              </CardDescription>
            </CardHeader>
            <CardContent>
              <form onSubmit={handleSubmit(onSubmit)} className="space-y-6">
                {/* Experiment Selection */}
                <div className="space-y-2">
                  <Label htmlFor="experimentId">Experiment *</Label>
                  <Select
                    value={watchedExperimentId}
                    onValueChange={(value) => setValue("experimentId", value)}
                    disabled={experimentsLoading}
                  >
                    <SelectTrigger className={errors.experimentId ? "border-red-500" : ""}>
                      <SelectValue placeholder={experimentsLoading ? "Loading experiments..." : "Select an experiment"} />
                    </SelectTrigger>
                    <SelectContent>
                      {experimentsData?.experiments?.map((experiment) => (
                        <SelectItem key={experiment.id} value={experiment.id}>
                          <div className="flex flex-col">
                            <span className="font-medium">{experiment.name}</span>
                            <span className="text-xs text-slate-500">{experiment.study.name}</span>
                          </div>
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  {errors.experimentId && (
                    <p className="text-sm text-red-600">{errors.experimentId.message}</p>
                  )}
                  {selectedExperiment && (
                    <div className="p-3 bg-blue-50 rounded-lg border border-blue-200">
                      <p className="text-sm text-blue-800">
                        <strong>Study:</strong> {selectedExperiment.study.name}
                      </p>
                      <p className="text-sm text-blue-700 mt-1">
                        {selectedExperiment.description}
                      </p>
                      {selectedExperiment.estimatedDuration && (
                        <p className="text-sm text-blue-700 mt-1">
                          <strong>Estimated Duration:</strong> {selectedExperiment.estimatedDuration} minutes
                        </p>
                      )}
                    </div>
                  )}
                </div>

                {/* Participant Selection */}
                <div className="space-y-2">
                  <Label htmlFor="participantId">Participant *</Label>
                  <Select
                    value={watchedParticipantId}
                    onValueChange={(value) => setValue("participantId", value)}
                    disabled={participantsLoading}
                  >
                    <SelectTrigger className={errors.participantId ? "border-red-500" : ""}>
                      <SelectValue placeholder={participantsLoading ? "Loading participants..." : "Select a participant"} />
                    </SelectTrigger>
                    <SelectContent>
                      {participantsData?.participants?.map((participant) => (
                        <SelectItem key={participant.id} value={participant.id}>
                          <div className="flex flex-col">
                            <span className="font-medium">{participant.participantCode}</span>
                            {participant.name && (
                              <span className="text-xs text-slate-500">{participant.name}</span>
                            )}
                          </div>
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  {errors.participantId && (
                    <p className="text-sm text-red-600">{errors.participantId.message}</p>
                  )}
                  {selectedParticipant && (
                    <div className="p-3 bg-green-50 rounded-lg border border-green-200">
                      <p className="text-sm text-green-800">
                        <strong>Code:</strong> {selectedParticipant.participantCode}
                      </p>
                      {selectedParticipant.name && (
                        <p className="text-sm text-green-700 mt-1">
                          <strong>Name:</strong> {selectedParticipant.name}
                        </p>
                      )}
                      {selectedParticipant.email && (
                        <p className="text-sm text-green-700 mt-1">
                          <strong>Email:</strong> {selectedParticipant.email}
                        </p>
                      )}
                    </div>
                  )}
                </div>

                {/* Scheduled Date & Time */}
                <div className="space-y-2">
                  <Label htmlFor="scheduledAt">Scheduled Date & Time *</Label>
                  <Input
                    id="scheduledAt"
                    type="datetime-local"
                    min={minDateTime}
                    {...register("scheduledAt")}
                    className={errors.scheduledAt ? "border-red-500" : ""}
                  />
                  {errors.scheduledAt && (
                    <p className="text-sm text-red-600">{errors.scheduledAt.message}</p>
                  )}
                  <p className="text-xs text-muted-foreground">
                    Select when this trial session should take place
                  </p>
                </div>

                {/* Wizard Assignment */}
                <div className="space-y-2">
                  <Label htmlFor="wizardId">Assigned Wizard (Optional)</Label>
                  <Select
                    value={watchedWizardId}
                    onValueChange={(value) => setValue("wizardId", value)}
                    disabled={wizardsLoading}
                  >
                    <SelectTrigger>
                      <SelectValue placeholder={wizardsLoading ? "Loading wizards..." : "Select a wizard (optional)"} />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="">No wizard assigned</SelectItem>
                      {wizardsData?.map((wizard) => (
                        <SelectItem key={wizard.id} value={wizard.id}>
                          <div className="flex flex-col">
                            <span className="font-medium">{wizard.name || wizard.email}</span>
                            <span className="text-xs text-slate-500 capitalize">{wizard.role}</span>
                          </div>
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                  <p className="text-xs text-muted-foreground">
                    Assign a specific team member to operate the wizard interface
                  </p>
                </div>

                {/* Notes */}
                <div className="space-y-2">
                  <Label htmlFor="notes">Notes (Optional)</Label>
                  <Textarea
                    id="notes"
                    {...register("notes")}
                    placeholder="Add any special instructions, participant details, or setup notes..."
                    rows={3}
                    className={errors.notes ? "border-red-500" : ""}
                  />
                  {errors.notes && (
                    <p className="text-sm text-red-600">{errors.notes.message}</p>
                  )}
                </div>

                {/* Error Message */}
                {createTrialMutation.error && (
                  <div className="rounded-md bg-red-50 p-3">
                    <p className="text-sm text-red-800">
                      Failed to create trial: {createTrialMutation.error.message}
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
                    disabled={isSubmitting || experimentsLoading || participantsLoading}
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
                        <span>Scheduling...</span>
                      </div>
                    ) : (
                      "Schedule Trial"
                    )}
                  </Button>
                </div>
              </form>
            </CardContent>
          </Card>
        </div>

        {/* Sidebar */}
        <div className="space-y-6">
          {/* Quick Stats */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center space-x-2">
                <FlaskConical className="h-5 w-5" />
                <span>Available Resources</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3 text-sm">
                <div className="flex justify-between">
                  <span className="text-slate-600">Experiments:</span>
                  <span className="font-medium">
                    {experimentsLoading ? "..." : experimentsData?.experiments?.length || 0}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-600">Participants:</span>
                  <span className="font-medium">
                    {participantsLoading ? "..." : participantsData?.participants?.length || 0}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-600">Available Wizards:</span>
                  <span className="font-medium">
                    {wizardsLoading ? "..." : wizardsData?.length || 0}
                  </span>
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Trial Process */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center space-x-2">
                <Clock className="h-5 w-5" />
                <span>Trial Process</span>
              </CardTitle>
            </CardHeader>
            <CardContent>
              <div className="space-y-3 text-sm">
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-blue-600"></div>
                  <div>
                    <p className="font-medium">Schedule Trial</p>
                    <p className="text-slate-600">Set up experiment and participant</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Check-in Participant</p>
                    <p className="text-slate-600">Verify consent and prepare setup</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Start Trial</p>
                    <p className="text-slate-600">Begin experiment execution</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Wizard Control</p>
                    <p className="text-slate-600">Real-time robot operation</p>
                  </div>
                </div>
                <div className="flex items-start space-x-3">
                  <div className="mt-1 h-2 w-2 rounded-full bg-slate-300"></div>
                  <div>
                    <p className="font-medium">Complete & Analyze</p>
                    <p className="text-slate-600">Review data and results</p>
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
                <strong>Preparation:</strong> Ensure all equipment is ready before the scheduled time.
              </p>
              <p>
                <strong>Participant Code:</strong> Use anonymous codes to protect participant privacy.
              </p>
              <p>
                <strong>Wizard Assignment:</strong> You can assign a wizard now or during the trial.
              </p>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}
