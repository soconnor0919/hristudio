export default async function StudyPage({ params }: { params: { id: string } }) {
  const study = await db.query.studies.findFirst({
    where: (studies, { eq }) => eq(studies.id, params.id),
    with: { experiments: true }
  })

  return (
    <div className="max-w-6xl mx-auto p-6">
      <StudyHeader study={study} />
      <Suspense fallback={<ExperimentListSkeleton />}>
        <ExperimentList studyId={params.id} />
      </Suspense>
    </div>
  )
} 