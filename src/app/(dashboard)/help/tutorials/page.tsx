import {
  BookOpen,
  FlaskConical,
  PlayCircle,
  BarChart3,
  Bot,
  FileText,
  ClipboardList,
  Layers,
  ArrowRight,
} from "lucide-react";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { PageLayout } from "~/components/ui/page-layout";
import Link from "next/link";

const tutorials = [
  {
    slug: "getting-started",
    title: "Getting Started",
    description: "Set up HRIStudio and learn the basics",
    icon: BookOpen,
    duration: "10 min",
    level: "Beginner",
    href: "/help/tutorials/getting-started",
  },
  {
    slug: "your-first-study",
    title: "Your First Study",
    description: "Create a research study and manage team members",
    icon: Layers,
    duration: "15 min",
    level: "Beginner",
    href: "/help/tutorials/your-first-study",
  },
  {
    slug: "designing-experiments",
    title: "Designing Experiments",
    description: "Build experiment protocols with the visual designer",
    icon: FlaskConical,
    duration: "25 min",
    level: "Intermediate",
    href: "/help/tutorials/designing-experiments",
  },
  {
    slug: "running-trials",
    title: "Running Trials",
    description: "Execute experiments and manage participants",
    icon: PlayCircle,
    duration: "20 min",
    level: "Intermediate",
    href: "/help/tutorials/running-trials",
  },
  {
    slug: "wizard-interface",
    title: "Wizard Interface",
    description: "Real-time trial control and monitoring",
    icon: Bot,
    duration: "15 min",
    level: "Intermediate",
    href: "/help/tutorials/wizard-interface",
  },
  {
    slug: "robot-integration",
    title: "Robot Integration",
    description: "Connect NAO6 and configure robot plugins",
    icon: ClipboardList,
    duration: "20 min",
    level: "Advanced",
    href: "/help/tutorials/robot-integration",
  },
  {
    slug: "forms-and-surveys",
    title: "Forms & Surveys",
    description: "Create consent forms and questionnaires",
    icon: FileText,
    duration: "15 min",
    level: "Intermediate",
    href: "/help/tutorials/forms-and-surveys",
  },
  {
    slug: "data-and-analysis",
    title: "Data & Analysis",
    description: "Collect and export trial data",
    icon: BarChart3,
    duration: "15 min",
    level: "Intermediate",
    href: "/help/tutorials/data-and-analysis",
  },
];

const levelColors: Record<string, string> = {
  Beginner: "bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-300",
  Intermediate: "bg-yellow-100 text-yellow-700 dark:bg-yellow-900 dark:text-yellow-300",
  Advanced: "bg-red-100 text-red-700 dark:bg-red-900 dark:text-red-300",
};

export default function TutorialsPage() {
  return (
    <PageLayout
      title="Tutorials"
      description="Step-by-step guides for learning HRIStudio"
      breadcrumb={[
        { label: "Help", href: "/help" },
        { label: "Tutorials" },
      ]}
    >
      <div className="mb-8">
        <h2 className="mb-2 text-lg font-semibold">Quick Start Path</h2>
        <p className="text-muted-foreground mb-4">
          Follow this sequence to go from setup to running your first trial.
        </p>
        <div className="flex flex-wrap items-center gap-2">
          {tutorials.slice(0, 5).map((tutorial, index) => (
            <div key={tutorial.slug} className="flex items-center gap-2">
              <Link href={tutorial.href}>
                <Button variant="outline" size="sm" className="gap-2">
                  <span className="flex h-6 w-6 items-center justify-center rounded-full bg-primary text-xs text-primary-foreground">
                    {index + 1}
                  </span>
                  {tutorial.title}
                </Button>
              </Link>
              {index < 4 && <ArrowRight className="text-muted-foreground h-4 w-4" />}
            </div>
          ))}
        </div>
      </div>

      <div className="grid gap-6 md:grid-cols-2 lg:grid-cols-3">
        {tutorials.map((tutorial) => (
          <Link key={tutorial.slug} href={tutorial.href}>
            <Card className="h-full transition-all hover:border-primary hover:shadow-md">
              <CardHeader>
                <div className="mb-2 flex items-center justify-between">
                  <div className="bg-primary/10 rounded-lg p-2">
                    <tutorial.icon className="text-primary h-5 w-5" />
                  </div>
                  <span
                    className={`rounded-full px-2 py-0.5 text-xs font-medium ${levelColors[tutorial.level]}`}
                  >
                    {tutorial.level}
                  </span>
                </div>
                <CardTitle className="text-lg">{tutorial.title}</CardTitle>
                <CardDescription>{tutorial.description}</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex items-center justify-between">
                  <span className="text-sm text-muted-foreground">
                    {tutorial.duration}
                  </span>
                  <ArrowRight className="text-muted-foreground h-4 w-4" />
                </div>
              </CardContent>
            </Card>
          </Link>
        ))}
      </div>

      <div className="mt-12">
        <h2 className="mb-4 text-lg font-semibold">By Role</h2>
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Researchers</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 text-sm">
              <Link href="/help/tutorials/getting-started" className="block text-muted-foreground hover:text-foreground">
                Getting Started
              </Link>
              <Link href="/help/tutorials/your-first-study" className="block text-muted-foreground hover:text-foreground">
                Your First Study
              </Link>
              <Link href="/help/tutorials/designing-experiments" className="block text-muted-foreground hover:text-foreground">
                Designing Experiments
              </Link>
              <Link href="/help/tutorials/data-and-analysis" className="block text-muted-foreground hover:text-foreground">
                Data & Analysis
              </Link>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Wizards</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 text-sm">
              <Link href="/help/tutorials/getting-started" className="block text-muted-foreground hover:text-foreground">
                Getting Started
              </Link>
              <Link href="/help/tutorials/wizard-interface" className="block text-muted-foreground hover:text-foreground">
                Wizard Interface
              </Link>
              <Link href="/help/tutorials/robot-integration" className="block text-muted-foreground hover:text-foreground">
                Robot Integration
              </Link>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Administrators</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 text-sm">
              <Link href="/help/tutorials/getting-started" className="block text-muted-foreground hover:text-foreground">
                Getting Started
              </Link>
              <Link href="/help/tutorials/robot-integration" className="block text-muted-foreground hover:text-foreground">
                Robot Integration
              </Link>
              <Link href="/help/tutorials/forms-and-surveys" className="block text-muted-foreground hover:text-foreground">
                Forms & Surveys
              </Link>
            </CardContent>
          </Card>
          <Card>
            <CardHeader className="pb-2">
              <CardTitle className="text-base">Pilot Testing</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2 text-sm">
              <Link href="/help/tutorials/getting-started" className="block text-muted-foreground hover:text-foreground">
                Getting Started
              </Link>
              <Link href="/help/tutorials/designing-experiments" className="block text-muted-foreground hover:text-foreground">
                Designing Experiments
              </Link>
              <Link href="/help/tutorials/running-trials" className="block text-muted-foreground hover:text-foreground">
                Running Trials
              </Link>
            </CardContent>
          </Card>
        </div>
      </div>
    </PageLayout>
  );
}
