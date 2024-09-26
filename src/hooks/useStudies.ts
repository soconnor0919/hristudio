import { useState, useEffect } from 'react';
import { Study } from '../types/Study';
import { useStudyContext } from '../context/StudyContext';

export function useStudies() {
  const [studies, setStudies] = useState<Study[]>([]);
  const { selectedStudy, setSelectedStudy } = useStudyContext();

  useEffect(() => {
    fetchStudies();
  }, []);

  const fetchStudies = async () => {
    const response = await fetch('/api/studies');
    if (!response.ok) {
      throw new Error('Failed to fetch studies');
    }
    const data = await response.json();
    setStudies(data);
  };

  const handleStudyChange = (studyId: string) => {
    const study = studies.find(s => s.id.toString() === studyId);
    setSelectedStudy(study || null);
  };

  const addStudy = async (newStudy: Omit<Study, 'id'>) => {
    const response = await fetch('/api/studies', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(newStudy),
    });
    if (!response.ok) {
      throw new Error('Failed to create study');
    }
    const createdStudy = await response.json();
    setStudies(prevStudies => [...prevStudies, createdStudy]);
    setSelectedStudy(createdStudy);
  };

  const deleteStudy = async (id: number) => {
    const response = await fetch(`/api/studies/${id}`, {
      method: 'DELETE',
    });
    if (!response.ok) {
      throw new Error('Failed to delete study');
    }
    setStudies(studies.filter(s => s.id !== id));
    if (selectedStudy?.id === id) {
      setSelectedStudy(null);
    }
  };

  return { studies, selectedStudy, handleStudyChange, addStudy, deleteStudy };
}