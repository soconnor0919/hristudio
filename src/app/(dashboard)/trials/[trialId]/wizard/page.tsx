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
  } catch (_error) {
    notFound();
  }

  // Only allow wizard interface for scheduled or in-progress trials
  if (!["scheduled", "in_progress"].includes(trial.status)) {
    redirect(`/trials/${trialId}?error=trial_not_active`);
  }

  return (
    <div className="min-h-screen bg-slate-50">
      {/* Header */}
      <div className="border-b border-slate-200 bg-white px-6 py-4">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-2xl font-bold text-slate-900">
              Wizard Control Interface
            </h1>
            <p className="mt-1 text-sm text-slate-600">
              {trial.experiment.name} • Participant:{" "}
              {trial.participant.participantCode}
            </p>
          </div>
          <div className="flex items-center space-x-2">
            <div
              className={`flex items-center space-x-2 rounded-full px-3 py-1 text-sm font-medium ${
                trial.status === "in_progress"
                  ? "bg-green-100 text-green-800"
                  : "bg-blue-100 text-blue-800"
              }`}
            >
              <div
                className={`h-2 w-2 rounded-full ${
                  trial.status === "in_progress"
                    ? "animate-pulse bg-green-500"
                    : "bg-blue-500"
                }`}
              ></div>
              {trial.status === "in_progress"
                ? "Trial Active"
                : "Ready to Start"}
            </div>
          </div>
        </div>
      </div>

      {/* Main Wizard Interface */}
      <WizardInterface trial={trial} userRole={userRole} />
    </div>
  );
}

// Generate metadata for the page
export async function generateMetadata({ params }: WizardPageProps) {
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
