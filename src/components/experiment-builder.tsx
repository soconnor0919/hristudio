export function ExperimentBuilder() {
  return (
    <DndContext>
      <SortableContext items={steps}>
        <div className="space-y-4">
          {steps.map((step) => (
            <ExperimentStep
              key={step.id}
              step={step}
              actions={actionsMap[step.id]}
            />
          ))}
        </div>
      </SortableContext>
      <DragOverlay>
        {activeStep && <StepPreview step={activeStep} />}
      </DragOverlay>
    </DndContext>
  )
} 