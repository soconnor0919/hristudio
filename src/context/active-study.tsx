'use client';

import { createContext, useContext, useEffect, useState, useCallback } from 'react';
import { usePathname, useRouter } from 'next/navigation';

interface Study {
  id: number;
  title: string;
  description: string | null;
  userId: string;
  environment: string;
  createdAt: Date;
  updatedAt: Date | null;
  permissions: string[];
}

interface ActiveStudyContextType {
  activeStudy: Study | null;
  setActiveStudy: (study: Study | null) => void;
  studies: Study[];
  isLoading: boolean;
  error: string | null;
  refreshStudies: () => Promise<void>;
}

const ActiveStudyContext = createContext<ActiveStudyContextType | undefined>(undefined);

export function ActiveStudyProvider({ children }: { children: React.ReactNode }) {
  const [activeStudy, setActiveStudy] = useState<Study | null>(null);
  const [studies, setStudies] = useState<Study[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const pathname = usePathname();
  const router = useRouter();

  const fetchStudies = useCallback(async () => {
    try {
      const response = await fetch('/api/studies', {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });
      
      if (!response.ok) throw new Error('Failed to fetch studies');
      const data = await response.json();
      
      const studiesWithDates = (data.data || []).map((study: any) => ({
        ...study,
        createdAt: new Date(study.createdAt),
        updatedAt: study.updatedAt ? new Date(study.updatedAt) : null,
      }));
      
      setStudies(studiesWithDates);
      
      if (studiesWithDates.length === 1 && !activeStudy) {
        setActiveStudy(studiesWithDates[0]);
        router.push(`/dashboard/studies/${studiesWithDates[0].id}`);
      }
    } catch (error) {
      console.error('Error fetching studies:', error);
      setError(error instanceof Error ? error.message : 'Failed to load studies');
    } finally {
      setIsLoading(false);
    }
  }, [router]);

  useEffect(() => {
    fetchStudies();
  }, [fetchStudies]);

  useEffect(() => {
    const studyIdMatch = pathname.match(/\/dashboard\/studies\/(\d+)/);
    if (studyIdMatch) {
      const studyId = parseInt(studyIdMatch[1]);
      const study = studies.find(s => s.id === studyId);
      if (study && (!activeStudy || activeStudy.id !== study.id)) {
        setActiveStudy(study);
      }
    } else if (!pathname.includes('/studies/new')) {
      if (activeStudy) {
        setActiveStudy(null);
      }
    }
  }, [pathname, studies]);

  const value = {
    activeStudy,
    setActiveStudy,
    studies,
    isLoading,
    error,
    refreshStudies: fetchStudies,
  };

  return (
    <ActiveStudyContext.Provider value={value}>
      {children}
    </ActiveStudyContext.Provider>
  );
}

export function useActiveStudy() {
  const context = useContext(ActiveStudyContext);
  if (context === undefined) {
    throw new Error('useActiveStudy must be used within an ActiveStudyProvider');
  }
  return context;
} 