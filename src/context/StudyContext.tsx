"use client";

import React, { createContext, useContext, useState, ReactNode } from 'react';

interface StudyContextType {
  selectedStudyId: number | null;
  setSelectedStudyId: (id: number | null) => void;
}

const StudyContext = createContext<StudyContextType | undefined>(undefined);

export const StudyProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [selectedStudyId, setSelectedStudyId] = useState<number | null>(null);

  return (
    <StudyContext.Provider value={{ selectedStudyId, setSelectedStudyId }}>
      {children}
    </StudyContext.Provider>
  );
};

export const useStudy = () => {
  const context = useContext(StudyContext);
  if (!context) {
    throw new Error('useStudy must be used within a StudyProvider');
  }
  return context;
};
