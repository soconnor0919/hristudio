import { ExperimentsDataTable } from "~/components/experiments/experiments-data-table";
import { StudyGuard } from "~/components/dashboard/study-guard";

export default function ExperimentsPage() {
  return (
    <StudyGuard>
      <ExperimentsDataTable />
    </StudyGuard>
  );
}
