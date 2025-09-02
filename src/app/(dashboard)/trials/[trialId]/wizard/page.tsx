import { notFound, redirect } from "next/navigation";
import { WizardInterface } from "~/components/trials/wizard/WizardInterface";
import { auth } from "~/server/auth";
import { api } from "~/trpc/server";

interface WizardPageProps {
  params: Promise<{
    trialId: string;
  }>;
}

export default async function WizardPage({ params }: WizardPageProps) {
  const session = await auth();

  if (!session) {
    redirect("/auth/signin");
  }

  // Check if user has wizard/researcher permissions
  const userRole = session.user.roles?.[0]?.role;
  if (
    !userRole ||
    !["wizard", "researcher", "administrator"].includes(userRole)
  ) {
    redirect("/trials?error=insufficient_permissions");
  }

  const { trialId } = await params;
  let trial;
  try {
    trial = await api.trials.get({ id: trialId });
  } catch {
    notFound();
  }

  // Only allow wizard interface for scheduled or in-progress trials
  if (!["scheduled", "in_progress"].includes(trial.status)) {
    redirect(`/trials/${trialId}?error=trial_not_active`);
  }

  const normalizedTrial = {
    ...trial,
    metadata:
      typeof trial.metadata === "object" && trial.metadata !== null
        ? (trial.metadata as Record<string, unknown>)
        : null,
    participant: {
      ...trial.participant,
      demographics:
        typeof trial.participant.demographics === "object" &&
        trial.participant.demographics !== null
          ? (trial.participant.demographics as Record<string, unknown>)
          : null,
    },
  };

  return <WizardInterface trial={normalizedTrial} userRole={userRole} />;
}

// Generate metadata for the page
export async function generateMetadata({
  params,
}: WizardPageProps): Promise<{ title: string; description: string }> {
  try {
    const { trialId } = await params;
    const trial = await api.trials.get({ id: trialId });
    return {
      title: `Wizard Control - ${trial.experiment.name} | HRIStudio`,
      description: `Real-time wizard control interface for trial ${trial.participant.participantCode}`,
    };
  } catch {
    return {
      title: "Wizard Control | HRIStudio",
      description: "Real-time wizard control interface for HRI trials",
    };
  }
}
