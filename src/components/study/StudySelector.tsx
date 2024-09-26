import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "~/components/ui/select";
import { Study } from '../../types/Study';

interface StudySelectorProps {
  studies: Study[];
  selectedStudy: Study | null;
  onStudyChange: (studyId: string) => void;
}

export function StudySelector({ studies, selectedStudy, onStudyChange }: StudySelectorProps) {
  return (
    <Select onValueChange={onStudyChange} value={selectedStudy?.id?.toString() || ""}>
      <SelectTrigger className="w-[200px]">
        <SelectValue placeholder="Select a study" />
      </SelectTrigger>
      <SelectContent>
        {studies.length > 0 ? (
          studies.map((study) => (
            <SelectItem key={study.id} value={study.id.toString()}>
              {study.title}
            </SelectItem>
          ))
        ) : (
          <SelectItem value="no-studies" disabled className="text-gray-400 italic">
            No studies available
          </SelectItem>
        )}
      </SelectContent>
    </Select>
  );
}