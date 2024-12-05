'use client';

import { useParams } from "next/navigation";
import { useEffect } from "react";
import { useActiveStudy } from "~/context/active-study";
import { Skeleton } from "~/components/ui/skeleton";

export default function StudyLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const { id } = useParams();
  const { studies, activeStudy, setActiveStudy, isLoading } = useActiveStudy();

  useEffect(() => {
    if (studies.length > 0 && id) {
      const study = studies.find(s => s.id === parseInt(id as string));
      if (study && (!activeStudy || activeStudy.id !== study.id)) {
        setActiveStudy(study);
      }
    }
  }, [id, studies, activeStudy, setActiveStudy]);

  if (isLoading) {
    return (
      <div className="space-y-6">
        <div className="h-6">
          <Skeleton className="h-4 w-[250px]" />
        </div>
        <Skeleton className="h-[400px]" />
      </div>
    );
  }

  return children;
} 