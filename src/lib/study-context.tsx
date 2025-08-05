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

export function StudyProvider({ children }: { children: ReactNode }) {
  const [selectedStudyId, setSelectedStudyIdState] = useState<string | null>(
    null,
  );
  const [isLoading, setIsLoading] = useState(true);

  // Load from localStorage on mount
  useEffect(() => {
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
  }, []);

  // Persist to localStorage when changed
  const setSelectedStudyId = (studyId: string | null) => {
    setSelectedStudyIdState(studyId);
    try {
      if (studyId) {
        localStorage.setItem(STUDY_STORAGE_KEY, studyId);
      } else {
        localStorage.removeItem(STUDY_STORAGE_KEY);
      }
    } catch (error) {
      console.warn("Failed to save study selection to localStorage:", error);
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
