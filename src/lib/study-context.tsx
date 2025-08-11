"use client";

import {
  createContext,
  useContext,
  useState,
  useEffect,
  type ReactNode,
} from "react";

interface StudyContextType {
  selectedStudyId: string | null;
  setSelectedStudyId: (studyId: string | null) => void;
  isLoading: boolean;
}

const StudyContext = createContext<StudyContextType | undefined>(undefined);

const STUDY_STORAGE_KEY = "hristudio-selected-study";

export function StudyProvider({
  children,
  initialStudyId,
}: {
  children: ReactNode;
  initialStudyId?: string | null;
}) {
  const [selectedStudyId, setSelectedStudyIdState] = useState<string | null>(
    initialStudyId ?? null,
  );
  const [isLoading, setIsLoading] = useState(true);

  // Load from localStorage on mount (only if no server-provided initial ID)
  useEffect(() => {
    if (initialStudyId) {
      setIsLoading(false);
      return;
    }
    try {
      const stored = localStorage.getItem(STUDY_STORAGE_KEY);
      if (stored && stored !== "null") {
        setSelectedStudyIdState(stored);
      }
    } catch (error) {
      console.warn("Failed to load study selection from localStorage:", error);
    } finally {
      setIsLoading(false);
    }
  }, [initialStudyId]);

  // Persist to localStorage & cookie when changed
  const setSelectedStudyId = (studyId: string | null): void => {
    setSelectedStudyIdState(studyId);
    try {
      if (studyId) {
        localStorage.setItem(STUDY_STORAGE_KEY, studyId);
        // 30 days
        document.cookie = `hristudio_selected_study=${studyId}; Path=/; Max-Age=2592000; SameSite=Lax`;
      } else {
        localStorage.removeItem(STUDY_STORAGE_KEY);
        document.cookie =
          "hristudio_selected_study=; Path=/; Max-Age=0; SameSite=Lax";
      }
    } catch (error) {
      console.warn("Failed to persist study selection:", error);
    }
  };

  return (
    <StudyContext.Provider
      value={{ selectedStudyId, setSelectedStudyId, isLoading }}
    >
      {children}
    </StudyContext.Provider>
  );
}

export function useStudyContext() {
  const context = useContext(StudyContext);
  if (context === undefined) {
    throw new Error("useStudyContext must be used within a StudyProvider");
  }
  return context;
}

export function useRequireStudy() {
  const { selectedStudyId, isLoading } = useStudyContext();
  return { selectedStudyId, isLoading };
}
