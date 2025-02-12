import Link from "next/link";
import { Card, CardHeader, CardTitle, CardDescription } from "~/components/ui/card";

interface StudyCardProps {
  id: number;
  title: string;
  description?: string | null;
  role: string;
}

export function StudyCard({ id, title, description, role }: StudyCardProps) {
  return (
    <Link href={`/studies/${id}`}>
      <Card className="hover:bg-muted/50 transition-colors">
        <CardHeader>
          <CardTitle>{title}</CardTitle>
          <CardDescription>{description}</CardDescription>
          <div className="text-sm text-muted-foreground mt-2">
            Role: {role}
          </div>
        </CardHeader>
      </Card>
    </Link>
  );
} 