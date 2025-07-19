import { TrialsGrid } from "~/components/trials/TrialsGrid";

export default function TrialsPage() {
  return (
    <div className="p-8">
      {/* Header */}
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-slate-900">Trials</h1>
        <p className="mt-2 text-slate-600">
          Schedule, execute, and monitor HRI experiment trials with real-time wizard control
        </p>
      </div>

      {/* Trials Grid */}
      <TrialsGrid />
    </div>
  );
}
