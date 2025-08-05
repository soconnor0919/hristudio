import { ParticipantsDataTable } from "~/components/participants/participants-data-table";
import { StudyGuard } from "~/components/dashboard/study-guard";

export default function ParticipantsPage() {
  return (
    <StudyGuard>
      <ParticipantsDataTable />
    </StudyGuard>
  );
}
