import { ExperimentsGrid } from "~/components/experiments/ExperimentsGrid";

export default function ExperimentsPage() {
  return (
    <div className="p-8">
      {/* Header */}
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-slate-900">Experiments</h1>
        <p className="mt-2 text-slate-600">
          Design and manage experimental protocols for your HRI studies
        </p>
      </div>

      {/* Experiments Grid */}
      <ExperimentsGrid />
    </div>
  );
}
