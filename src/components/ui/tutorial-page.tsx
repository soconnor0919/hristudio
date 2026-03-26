import { type ReactNode } from "react";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  ChevronLeft,
  ChevronRight,
  CheckCircle2,
} from "lucide-react";
import { PageLayout } from "~/components/ui/page-layout";

interface TutorialStep {
  title: string;
  description: string;
}

interface TutorialPageProps {
  children: ReactNode;
  title: string;
  description: string;
  duration: string;
  level: string;
  steps: TutorialStep[];
  prevTutorial?: {
    title: string;
    href: string;
  };
  nextTutorial?: {
    title: string;
    href: string;
  };
}

export function TutorialPage({
  children,
  title,
  description,
  duration,
  level,
  steps,
  prevTutorial,
  nextTutorial,
}: TutorialPageProps) {
  const levelColors: Record<string, string> = {
    Beginner: "bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-300",
    Intermediate: "bg-yellow-100 text-yellow-700 dark:bg-yellow-900 dark:text-yellow-300",
    Advanced: "bg-red-100 text-red-700 dark:bg-red-900 dark:text-red-300",
  };

  return (
    <PageLayout
      title={title}
      description={description}
      breadcrumb={[
        { label: "Help", href: "/help" },
        { label: "Tutorials", href: "/help/tutorials" },
        { label: title },
      ]}
    >
      <div className="grid gap-8 lg:grid-cols-[1fr_250px]">
        <div className="prose prose-slate dark:prose-invert max-w-none">
          {children}
        </div>

        <aside className="hidden lg:block">
          <div className="sticky top-4 space-y-6">
            <Card className="p-4">
              <div className="mb-4 flex items-center justify-between">
                <span className="text-sm font-medium">Tutorial Info</span>
                <span
                  className={`rounded-full px-2 py-0.5 text-xs font-medium ${levelColors[level]}`}
                >
                  {level}
                </span>
              </div>
              <div className="space-y-2 text-sm">
                <div className="flex justify-between">
                  <span className="text-muted-foreground">Duration</span>
                  <span>{duration}</span>
                </div>
              </div>
            </Card>

            <div>
              <h3 className="mb-3 text-sm font-medium">In This Tutorial</h3>
              <ul className="space-y-2">
                {steps.map((step, index) => (
                  <li key={index} className="flex items-start gap-2 text-sm">
                    <CheckCircle2 className="text-primary mt-0.5 h-4 w-4 flex-shrink-0" />
                    <span>{step.title}</span>
                  </li>
                ))}
              </ul>
            </div>

            <div className="flex flex-col gap-2">
              {prevTutorial && (
                <Button variant="outline" size="sm" className="justify-start" asChild>
                  <Link href={prevTutorial.href}>
                    <ChevronLeft className="mr-1 h-4 w-4" />
                    {prevTutorial.title}
                  </Link>
                </Button>
              )}
              {nextTutorial && (
                <Button variant="outline" size="sm" className="justify-start" asChild>
                  <Link href={nextTutorial.href}>
                    {nextTutorial.title}
                    <ChevronRight className="ml-1 h-4 w-4" />
                  </Link>
                </Button>
              )}
            </div>
          </div>
        </aside>
      </div>
    </PageLayout>
  );
}

function Card({ children, className }: { children: ReactNode; className?: string }) {
  return (
    <div className={`rounded-lg border bg-card text-card-foreground shadow-sm ${className}`}>
      {children}
    </div>
  );
}
