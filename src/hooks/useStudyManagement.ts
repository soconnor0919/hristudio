"use client";

import { useCallback } from "react";
import * as React from "react";
import { useRouter } from "next/navigation";
import { toast } from "sonner";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";

/**
 * Custom hook for centralized study management across the platform.
 * Handles study creation, updates, deletion, and ensures all UI components
 * stay synchronized with the latest study data.
 */
export function useStudyManagement() {
  const router = useRouter();
  const { selectedStudyId, setSelectedStudyId } = useStudyContext();
  const utils = api.useUtils();

  /**
   * Invalidates all study-related queries to ensure UI consistency
   */
  const invalidateStudyQueries = useCallback(async () => {
    await Promise.all([
      utils.studies.list.invalidate(),
      utils.studies.get.invalidate(),
      utils.studies.getMembers.invalidate(),
    ]);
  }, [utils]);

  /**
   * Creates a new study and updates the entire platform UI
   */
  const createStudy = useCallback(
    async (data: {
      name: string;
      description?: string;
      institution?: string;
      irbProtocol?: string;
    }) => {
      try {
        const newStudy = await utils.client.studies.create.mutate(data);

        // Invalidate all study queries
        await invalidateStudyQueries();

        // Auto-select the newly created study
        setSelectedStudyId(newStudy.id);

        // Show success message
        toast.success(`Study "${newStudy.name}" created successfully!`);

        return newStudy;
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to create study";
        toast.error(message);
        throw error;
      }
    },
    [utils.client.studies.create, invalidateStudyQueries, setSelectedStudyId],
  );

  /**
   * Updates an existing study and refreshes the UI
   */
  const updateStudy = useCallback(
    async (
      studyId: string,
      data: {
        name?: string;
        description?: string;
        institution?: string;
        irbProtocol?: string;
        status?: "draft" | "active" | "completed" | "archived";
        settings?: Record<string, unknown>;
      },
    ) => {
      try {
        const updatedStudy = await utils.client.studies.update.mutate({
          id: studyId,
          ...data,
        });

        // Invalidate study queries
        await invalidateStudyQueries();

        // Show success message
        toast.success(`Study "${updatedStudy.name}" updated successfully!`);

        return updatedStudy;
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to update study";
        toast.error(message);
        throw error;
      }
    },
    [utils.client.studies.update, invalidateStudyQueries],
  );

  /**
   * Deletes a study and updates the UI
   */
  const deleteStudy = useCallback(
    async (studyId: string) => {
      try {
        await utils.client.studies.delete.mutate({ id: studyId });

        // Clear selected study if it was the deleted one
        if (selectedStudyId === studyId) {
          setSelectedStudyId(null);
        }

        // Invalidate study queries
        await invalidateStudyQueries();

        // Show success message
        toast.success("Study deleted successfully!");

        // Navigate to studies list
        router.push("/studies");
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to delete study";
        toast.error(message);
        throw error;
      }
    },
    [
      utils.client.studies.delete,
      selectedStudyId,
      setSelectedStudyId,
      invalidateStudyQueries,
      router,
    ],
  );

  /**
   * Adds a member to a study and refreshes related data
   */
  const addStudyMember = useCallback(
    async (
      studyId: string,
      email: string,
      role: "researcher" | "wizard" | "observer",
    ) => {
      try {
        await utils.client.studies.addMember.mutate({
          studyId,
          email,
          role,
        });

        // Invalidate relevant queries
        await Promise.all([
          utils.studies.get.invalidate({ id: studyId }),
          utils.studies.getMembers.invalidate({ studyId }),
          utils.studies.list.invalidate(),
        ]);

        // Show success message
        toast.success("Member added successfully!");
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to add member";
        toast.error(message);
        throw error;
      }
    },
    [utils],
  );

  /**
   * Removes a member from a study and refreshes related data
   */
  const removeStudyMember = useCallback(
    async (studyId: string, memberId: string) => {
      try {
        await utils.client.studies.removeMember.mutate({
          studyId,
          memberId,
        });

        // Invalidate relevant queries
        await Promise.all([
          utils.studies.get.invalidate({ id: studyId }),
          utils.studies.getMembers.invalidate({ studyId }),
          utils.studies.list.invalidate(),
        ]);

        // Show success message
        toast.success("Member removed successfully!");
      } catch (error) {
        const message =
          error instanceof Error ? error.message : "Failed to remove member";
        toast.error(message);
        throw error;
      }
    },
    [utils],
  );

  /**
   * Selects a study and ensures its data is fresh
   */
  const selectStudy = useCallback(
    async (studyId: string | null) => {
      try {
        // Optimistic update for better UX
        setSelectedStudyId(studyId);

        // Ensure selected study data is fresh and exists
        if (studyId) {
          // Try to fetch the study to verify it exists
          const studyExists = await utils.client.studies.get.query({
            id: studyId,
          });
          if (!studyExists) {
            // Study doesn't exist, clear the selection
            setSelectedStudyId(null);
            toast.error("Selected study no longer exists");
            return;
          }
          await utils.studies.get.invalidate({ id: studyId });
        }
      } catch (error) {
        // If study fetch fails, clear the selection
        setSelectedStudyId(null);
        console.warn("Failed to select study, clearing selection:", error);
        if (studyId) {
          toast.error("Study no longer available");
        }
      }
    },
    [setSelectedStudyId, utils.studies.get, utils.client.studies.get],
  );

  /**
   * Refreshes all study data (useful for manual refresh triggers)
   */
  const refreshStudyData = useCallback(async () => {
    await invalidateStudyQueries();
    toast.success("Study data refreshed!");
  }, [invalidateStudyQueries]);

  /**
   * Gets the currently selected study data
   */
  const selectedStudy = api.studies.get.useQuery(
    { id: selectedStudyId! },
    {
      enabled: !!selectedStudyId,
      staleTime: 1000 * 60 * 2, // 2 minutes
      retry: (failureCount, error) => {
        // Don't retry if study not found (404-like errors)
        if (
          error.message?.includes("not found") ||
          error.message?.includes("404")
        ) {
          // Clear the selection if study is not found
          setSelectedStudyId(null);
          return false;
        }
        return failureCount < 2;
      },
    },
  );

  // Handle study not found error
  React.useEffect(() => {
    if (selectedStudy.error) {
      const error = selectedStudy.error;
      if (
        error.message?.includes("not found") ||
        error.message?.includes("404")
      ) {
        console.warn("Selected study not found, clearing selection");
        setSelectedStudyId(null);
        toast.error("Study no longer exists");
      }
    }
  }, [selectedStudy.error, setSelectedStudyId]);

  /**
   * Gets the list of all user studies
   */
  const userStudies = api.studies.list.useQuery(
    { memberOnly: true, limit: 100 },
    {
      staleTime: 1000 * 60 * 2, // 2 minutes
      refetchOnWindowFocus: true,
    },
  );

  return {
    // State
    selectedStudyId,
    selectedStudy: selectedStudy.data,
    isLoadingSelectedStudy: selectedStudy.isLoading,
    userStudies: userStudies.data?.studies ?? [],
    isLoadingUserStudies: userStudies.isLoading,

    // Actions
    createStudy,
    updateStudy,
    deleteStudy,
    addStudyMember,
    removeStudyMember,
    selectStudy,
    refreshStudyData,
    invalidateStudyQueries,

    // Navigation helpers
    navigateToStudy: (studyId: string) => router.push(`/studies/${studyId}`),
    navigateToNewStudy: () => router.push("/studies/new"),
    navigateToStudiesList: () => router.push("/studies"),
  };
}

/**
 * Hook for components that require a selected study
 */
export function useRequiredStudy() {
  const { selectedStudyId, selectedStudy, isLoadingSelectedStudy } =
    useStudyManagement();

  if (!selectedStudyId) {
    throw new Error("This component requires a selected study");
  }

  return {
    studyId: selectedStudyId,
    study: selectedStudy,
    isLoading: isLoadingSelectedStudy,
  };
}

/**
 * Hook for optimistic study updates
 */
export function useOptimisticStudyUpdate() {
  const utils = api.useUtils();

  const optimisticUpdate = useCallback(
    (
      studyId: string,
      _updateData: Partial<{
        name: string;
        status: string;
        description: string;
      }>,
    ) => {
      // Simple optimistic update - just invalidate the queries
      // This is safer than trying to manually update the cache
      void utils.studies.get.invalidate({ id: studyId });
      void utils.studies.list.invalidate();
    },
    [utils],
  );

  return { optimisticUpdate };
}
