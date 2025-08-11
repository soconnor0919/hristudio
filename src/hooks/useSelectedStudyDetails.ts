import { useCallback, useMemo } from "react";
import { api } from "~/trpc/react";
import { useStudyContext } from "~/lib/study-context";

/**
 * useSelectedStudyDetails
 *
 * Strongly typed unified source of truth for the currently selected study.
 *
 * Provides a single hook to retrieve:
 * - selected study id
 * - lightweight summary counts
 * - role + createdAt
 * - loading / fetching flags
 * - mutation helpers
 */

interface StudyRelatedEntity {
  id: string;
}

interface StudyMember {
  id: string;
  userId?: string;
  role?: string;
}

interface StudyDetails {
  id: string;
  name: string;
  description: string | null;
  status: string;
  experiments?: StudyRelatedEntity[];
  participants?: StudyRelatedEntity[];
  members?: StudyMember[];
  userRole?: string;
  createdAt?: Date;
}

export interface StudySummary {
  id: string;
  name: string;
  description: string;
  status: string;
  experimentCount: number;
  participantCount: number;
  memberCount: number;
  userRole?: string;
  createdAt?: Date;
}

export interface UseSelectedStudyDetailsReturn {
  studyId: string | null;
  study: StudySummary | null;
  isLoading: boolean;
  isFetching: boolean;
  refetch: () => Promise<unknown>;
  setStudyId: (id: string | null) => void;
  clearStudy: () => void;
  hasStudy: boolean;
}

export function useSelectedStudyDetails(): UseSelectedStudyDetailsReturn {
  const { selectedStudyId, setSelectedStudyId } = useStudyContext();

  const { data, isLoading, isFetching, refetch } = api.studies.get.useQuery(
    { id: selectedStudyId ?? "" },
    {
      enabled: !!selectedStudyId,
      refetchOnWindowFocus: false,
      staleTime: 5 * 60 * 1000,
    },
  );

  const study: StudySummary | null = useMemo(() => {
    if (!data || !selectedStudyId) return null;

    // data is inferred from tRPC; we defensively narrow array fields
    const typed = data as StudyDetails;

    const experiments = Array.isArray(typed.experiments)
      ? typed.experiments
      : [];
    const participants = Array.isArray(typed.participants)
      ? typed.participants
      : [];
    const members = Array.isArray(typed.members) ? typed.members : [];

    return {
      id: typed.id,
      name: typed.name ?? "Unnamed Study",
      description: typed.description ?? "",
      status: typed.status ?? "active",
      experimentCount: experiments.length,
      participantCount: participants.length,
      memberCount: members.length,
      userRole: typed.userRole,
      createdAt: typed.createdAt,
    };
  }, [data, selectedStudyId]);

  const setStudyId = useCallback(
    (id: string | null) => {
      void setSelectedStudyId(id);
    },
    [setSelectedStudyId],
  );

  const clearStudy = useCallback(() => {
    void setSelectedStudyId(null);
  }, [setSelectedStudyId]);

  return {
    studyId: selectedStudyId,
    study,
    isLoading,
    isFetching,
    refetch,
    setStudyId,
    clearStudy,
    hasStudy: !!study,
  };
}
