"use client";

import { useParams } from "next/navigation";
import { Suspense, useEffect } from "react";
import Link from "next/link";
import { Zap, ArrowLeft, Eye, User } from "lucide-react";
import { PageHeader } from "~/components/ui/page-header";
import { Button } from "~/components/ui/button";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyContext } from "~/lib/study-context";
import { useSelectedStudyDetails } from "~/hooks/useSelectedStudyDetails";
import { WizardView } from "~/components/trials/views/WizardView";
import { ObserverView } from "~/components/trials/views/ObserverView";
import { ParticipantView } from "~/components/trials/views/ParticipantView";
import { api } from "~/trpc/react";
import { useSession } from "next-auth/react";

function WizardPageContent() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const trialId: string =
    typeof params.trialId === "string" ? params.trialId : "";

  const { setSelectedStudyId, selectedStudyId } = useStudyContext();
  const { study } = useSelectedStudyDetails();
  const { data: session } = useSession();

  // Get trial data
  const {
    data: trial,
    isLoading,
    error,
  } = api.trials.get.useQuery({ id: trialId }, { enabled: !!trialId });

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${studyId}` },
    { label: "Trials", href: `/studies/${studyId}/trials` },
    {
      label: trial?.experiment.name ?? "Trial",
      href: `/studies/${studyId}/trials`,
    },
    { label: "Wizard Interface" },
  ]);

  // Sync selected study (unified study-context)
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  // Determine user role and view type
  const getUserRole = () => {
    if (!session?.user) return "observer";

    // Check URL parameters for role override (for testing)
    const urlParams = new URLSearchParams(window.location.search);
    const roleParam = urlParams.get("view");
    if (
      roleParam &&
      ["wizard", "observer", "participant"].includes(roleParam)
    ) {
      return roleParam;
    }

    // Default role logic based on user
    const userRole = session.user.roles?.[0]?.role ?? "observer";
    if (userRole === "administrator" || userRole === "researcher") {
      return "wizard";
    }

    return "observer";
  };

  const currentRole = getUserRole();

  if (isLoading) {
    return (
      <div className="flex h-96 items-center justify-center">
        <div className="text-muted-foreground">Loading trial...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Wizard Interface"
          description="Trial execution interface for wizards"
          icon={Zap}
          actions={
            <Button asChild variant="outline">
              <a href={`/studies/${studyId}/trials`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </a>
            </Button>
          }
        />
        <div className="flex h-96 items-center justify-center">
          <div className="text-center">
            <h3 className="text-destructive mb-2 text-lg font-semibold">
              Error Loading Trial
            </h3>
            <p className="text-muted-foreground">
              {error.message || "Failed to load trial data"}
            </p>
          </div>
        </div>
      </div>
    );
  }

  if (!trial) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Wizard Interface"
          description="Trial execution interface for wizards"
          icon={Zap}
          actions={
            <Button asChild variant="outline">
              <a href={`/studies/${studyId}/trials`}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to Trials
              </a>
            </Button>
          }
        />
        <div className="flex h-96 items-center justify-center">
          <div className="text-center">
            <h3 className="mb-2 text-lg font-semibold">Trial Not Found</h3>
            <p className="text-muted-foreground">
              The requested trial could not be found.
            </p>
          </div>
        </div>
      </div>
    );
  }

  const getViewTitle = (role: string) => {
    switch (role) {
      case "wizard":
        return `${trial.experiment.name} - Wizard Control`;
      case "observer":
        return `${trial.experiment.name} - Observer View`;
      case "participant":
        return `Research Session - ${trial.participant.participantCode}`;
      default:
        return `${trial.experiment.name} - Trial View`;
    }
  };

  const getViewIcon = (role: string) => {
    switch (role) {
      case "wizard":
        return Zap;
      case "observer":
        return Eye;
      case "participant":
        return User;
      default:
        return Zap;
    }
  };

  const renderView = () => {
    const trialData = {
      id: trial.id,
      status: trial.status,
      scheduledAt: trial.scheduledAt,
      startedAt: trial.startedAt,
      completedAt: trial.completedAt,
      duration: trial.duration,
      sessionNumber: trial.sessionNumber,
      notes: trial.notes,
      metadata: trial.metadata as Record<string, unknown> | null,
      experimentId: trial.experimentId,
      participantId: trial.participantId,
      wizardId: trial.wizardId,
      experiment: {
        id: trial.experiment.id,
        name: trial.experiment.name,
        description: trial.experiment.description,
        studyId: trial.experiment.studyId,
      },
      participant: {
        id: trial.participant.id,
        participantCode: trial.participant.participantCode,
        demographics: trial.participant.demographics as Record<
          string,
          unknown
        > | null,
      },
    };

    switch (currentRole) {
      case "wizard":
        return <WizardView trial={trialData} userRole={currentRole} />;
      case "observer":
        return <ObserverView trial={trialData} />;
      case "participant":
        return <ParticipantView trial={trialData} />;
      default:
        return <ObserverView trial={trialData} />;
    }
  };

  return (
    <div>
      {renderView()}
    </div>
  );
}

export default function TrialWizardPage() {
  return (
    <Suspense
      fallback={
        <div className="flex h-96 items-center justify-center">
          <div className="text-muted-foreground">Loading...</div>
        </div>
      }
    >
      <WizardPageContent />
    </Suspense>
  );
}
