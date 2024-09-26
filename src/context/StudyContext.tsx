"use client";

import React, { createContext, useContext, useState, useEffect } from 'react';
import { Study } from '~/types/Study';

interface StudyContextType {
  selectedStudy: Study | null;
  setSelectedStudy: (study: Study | null) => void;
  validateAndSetSelectedStudy: (studyId: number) => Promise<void>;
  studies: Study[];
  fetchAndSetStudies: () => Promise<void>;
}

const StudyContext = createContext<StudyContextType | undefined>(undefined);

export const StudyProvider: React.FC<React.PropsWithChildren> = ({ children }) => {
  const [selectedStudy, setSelectedStudy] = useState<Study | null>(null);
  const [studies, setStudies] = useState<Study[]>([]);

  const fetchAndSetStudies = async () => {
    try {
      const response = await fetch('/api/studies');
      if (!response.ok) {
        throw new Error('Failed to fetch studies');
      }
      const fetchedStudies = await response.json();
      setStudies(fetchedStudies);
    } catch (error) {
      console.error('Error fetching studies:', error);
      setStudies([]);
    }
  };

  const validateAndSetSelectedStudy = async (studyId: number) => {
    const existingStudy = studies.find(s => s.id === studyId);
    if (existingStudy) {
      setSelectedStudy(existingStudy);
      return;
    }

    try {
      const response = await fetch(`/api/studies/${studyId}`);
      if (!response.ok) {
        throw new Error('Study not found');
      }
      const study = await response.json();
      setSelectedStudy(study);
    } catch (error) {
      console.warn(`Study with id ${studyId} not found`);
      setSelectedStudy(null);
    }
  };

  useEffect(() => {
    fetchAndSetStudies();
  }, []);

  return (
    <StudyContext.Provider value={{ 
      selectedStudy, 
      setSelectedStudy, 
      validateAndSetSelectedStudy,
      studies, 
      fetchAndSetStudies 
    }}>
      {children}
    </StudyContext.Provider>
  );
};

export const useStudyContext = () => {
  const context = useContext(StudyContext);
  if (context === undefined) {
    throw new Error('useStudyContext must be used within a StudyProvider');
  }
  return context;
};