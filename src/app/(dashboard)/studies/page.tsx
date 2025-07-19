import { StudiesGrid } from "~/components/studies/StudiesGrid";

export default function StudiesPage() {
  return (
    <div className="p-8">
      {/* Header */}
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-slate-900">Studies</h1>
        <p className="mt-2 text-slate-600">
          Manage your Human-Robot Interaction research studies
        </p>
      </div>

      {/* Studies Grid */}
      <StudiesGrid />
    </div>
  );
}
