import { StudyForm } from "~/components/studies/StudyForm";

interface EditStudyPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default async function EditStudyPage({ params }: EditStudyPageProps) {
  const { id } = await params;

  return <StudyForm mode="edit" studyId={id} />;
}
