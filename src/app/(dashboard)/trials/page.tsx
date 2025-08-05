import { TrialsDataTable } from "~/components/trials/trials-data-table";
import { StudyGuard } from "~/components/dashboard/study-guard";

export default function TrialsPage() {
  return (
    <StudyGuard>
      <TrialsDataTable />
    </StudyGuard>
  );
}
