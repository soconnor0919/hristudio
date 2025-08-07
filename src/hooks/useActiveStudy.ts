"use client";

import { useSession } from "next-auth/react";
import { useEffect, useState } from "react";
import { toast } from "sonner";
import { api } from "~/trpc/react";

const ACTIVE_STUDY_KEY = "hristudio-active-study";

// Helper function to validate UUID format
const isValidUUID = (id: string): boolean => {
  const uuidRegex =
    /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
  return uuidRegex.test(id);
};

export function useActiveStudy() {
  const { data: session } = useSession();
  const [activeStudyId, setActiveStudyId] = useState<string | null>(null);
  const [isSettingActiveStudy, setIsSettingActiveStudy] = useState(false);

  // Load active study from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem(ACTIVE_STUDY_KEY);
    if (stored && isValidUUID(stored)) {
      setActiveStudyId(stored);
    } else if (stored) {
      // Clear invalid UUID from localStorage
      localStorage.removeItem(ACTIVE_STUDY_KEY);
    }
  }, []);

  // Get active study details
  const { data: activeStudy, isLoading: isLoadingActiveStudy } =
    api.studies.get.useQuery(
      { id: activeStudyId! },
      {
        enabled: !!activeStudyId && isValidUUID(activeStudyId),
        staleTime: 5 * 60 * 1000, // 5 minutes
        retry: false, // Don't retry if study doesn't exist
      },
    );

  // Clear localStorage if study doesn't exist
  useEffect(() => {
    if (activeStudyId && !activeStudy && !isLoadingActiveStudy) {
      localStorage.removeItem(ACTIVE_STUDY_KEY);
      setActiveStudyId(null);
      toast.error(
        "Selected study no longer exists. Please select a new study.",
      );
    }
  }, [activeStudy, activeStudyId, isLoadingActiveStudy]);

  // Get user's studies for switching (always use memberOnly: true for security)
  const { data: studiesData, isLoading: isLoadingStudies } =
    api.studies.list.useQuery(
      { limit: 20, memberOnly: true },
      {
        staleTime: 2 * 60 * 1000, // 2 minutes
        enabled: !!session?.user?.id,
      },
    );

  const userStudies = studiesData?.studies ?? [];

  const utils = api.useUtils();

  const setActiveStudy = (studyId: string) => {
    if (!isValidUUID(studyId)) {
      toast.error("Invalid study ID format");
      return;
    }

    setIsSettingActiveStudy(true);
    setActiveStudyId(studyId);
    localStorage.setItem(ACTIVE_STUDY_KEY, studyId);

    // Invalidate all related queries when study changes
    void utils.participants.invalidate();
    void utils.trials.invalidate();
    void utils.experiments.invalidate();

    toast.success("Active study updated");

    // Reset loading state after a brief delay to allow queries to refetch
    setTimeout(() => {
      setIsSettingActiveStudy(false);
    }, 1000);
  };

  const clearActiveStudy = () => {
    setIsSettingActiveStudy(true);
    setActiveStudyId(null);
    localStorage.removeItem(ACTIVE_STUDY_KEY);

    // Invalidate all related queries when clearing study
    void utils.participants.invalidate();
    void utils.trials.invalidate();
    void utils.experiments.invalidate();

    toast.success("Active study cleared");

    // Reset loading state after a brief delay
    setTimeout(() => {
      setIsSettingActiveStudy(false);
    }, 500);
  };

  // Note: Auto-selection removed to require manual study selection

  return {
    // State
    activeStudyId,
    activeStudy:
      activeStudy && typeof activeStudy === "object"
        ? {
            id: activeStudy.id,
            title: (activeStudy as { name?: string }).name ?? "",
            description:
              (activeStudy as { description?: string }).description ?? "",
          }
        : null,
    userStudies: userStudies.map(
      (study: { id: string; name: string; description?: string | null }) => ({
        id: study.id,
        title: study.name,
        description: study.description ?? "",
      }),
    ),

    // Loading states
    isLoadingActiveStudy,
    isLoadingStudies,
    isSettingActiveStudy,
    isClearingActiveStudy: false,

    // Actions
    setActiveStudy,
    clearActiveStudy,

    // Utilities
    hasActiveStudy: !!activeStudyId,
    hasStudies: userStudies.length > 0,
  };
}
