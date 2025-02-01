"use client"

import * as React from "react"
import { api } from "~/trpc/react"

interface Study {
  id: number
  title: string
  description: string | null
  role: string
}

interface StudyContextType {
  studies: Study[]
  activeStudy: Study | null
  setActiveStudy: (study: Study | null) => void
  isLoading: boolean
}

const StudyContext = React.createContext<StudyContextType | undefined>(undefined)

const STORAGE_KEY = "activeStudyId"

function getStoredStudyId(): number | null {
  if (typeof window === "undefined") return null
  const stored = localStorage.getItem(STORAGE_KEY)
  return stored ? Number(stored) : null
}

export function StudyProvider({ children }: { children: React.ReactNode }) {
  // Initialize with stored study ID
  const [activeStudyId, setActiveStudyId] = React.useState<number | null>(() => getStoredStudyId())
  
  const { data: studies = [], isLoading } = api.study.getMyStudies.useQuery()

  // Find the active study from the studies array
  const activeStudy = React.useMemo(() => {
    if (!studies.length || !activeStudyId) return null
    return studies.find(s => s.id === activeStudyId) ?? studies[0]
  }, [studies, activeStudyId])

  // Update active study ID when studies load
  React.useEffect(() => {
    if (!studies.length) return;
    
    if (!activeStudyId || !studies.find(s => s.id === activeStudyId)) {
      // If no active study or it doesn't exist in the list, set the first study
      const id = studies[0]?.id;
      if (id) {
        setActiveStudyId(id);
        localStorage.setItem(STORAGE_KEY, String(id));
      }
    }
  }, [studies, activeStudyId]);

  const setActiveStudy = React.useCallback((study: Study | null) => {
    if (study) {
      setActiveStudyId(study.id)
      localStorage.setItem(STORAGE_KEY, String(study.id))
    } else {
      setActiveStudyId(null)
      localStorage.removeItem(STORAGE_KEY)
    }
  }, [])

  const value = React.useMemo(
    () => ({
      studies,
      activeStudy: activeStudy ?? null,
      setActiveStudy,
      isLoading,
    }),
    [studies, activeStudy, setActiveStudy, isLoading]
  )

  return (
    <StudyContext.Provider value={value}>
      {children}
    </StudyContext.Provider>
  )
}

export function useStudy() {
  const context = React.useContext(StudyContext)
  if (context === undefined) {
    throw new Error("useStudy must be used within a StudyProvider")
  }
  return context
} 