"use client";

import { useParams } from "next/navigation";
import { useEffect } from "react";
import { ExperimentForm } from "~/components/experiments/ExperimentForm";
import { useStudyContext } from "~/lib/study-context";

export default function NewStudyExperimentPage() {
  const params = useParams();
  const studyId: string = typeof params.id === "string" ? params.id : "";
  const { setSelectedStudyId, selectedStudyId } = useStudyContext();

  // Sync selected study (unified study-context)
  useEffect(() => {
    if (studyId && selectedStudyId !== studyId) {
      setSelectedStudyId(studyId);
    }
  }, [studyId, selectedStudyId, setSelectedStudyId]);

  return <ExperimentForm mode="create" />;
}
