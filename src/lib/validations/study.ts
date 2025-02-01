export const studySchema = z.object({
  title: z.string().min(1).max(100),
  description: z.string().max(500).optional(),
  robotId: z.string().uuid(),
  estimatedDuration: z.number().positive(),
  parameters: z.record(z.unknown()).refine(data => {
    return JSON.stringify(data).length <= 5000;
  }, "Parameters too large")
}); 