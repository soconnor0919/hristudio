import {
  BookOpen,
  FlaskConical,
  PlayCircle,
  BarChart3,
  HelpCircle,
  FileText,
  Video,
  ExternalLink,
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

export default function HelpCenterPage() {
  const guides = [
    {
      title: "Getting Started",
      description: "Learn the basics of HRIStudio and set up your first study.",
      icon: BookOpen,
      items: [
        { label: "Tutorials Overview", href: "/help/tutorials" },
        { label: "Getting Started Guide", href: "/help/tutorials/getting-started" },
        { label: "Your First Study", href: "/help/tutorials/your-first-study" },
      ],
    },
    {
      title: "Designing Experiments",
      description: "Master the visual experiment designer and flow control.",
      icon: FlaskConical,
      items: [
        { label: "Visual Designer Guide", href: "/help/tutorials/designing-experiments" },
        { label: "Robot Actions & Plugins", href: "/help/tutorials/robot-integration" },
        { label: "Wizard Interface", href: "/help/tutorials/wizard-interface" },
      ],
    },
    {
      title: "Running Trials",
      description: "Execute experiments and manage Wizard of Oz sessions.",
      icon: PlayCircle,
      items: [
        { label: "Running Trials Guide", href: "/help/tutorials/running-trials" },
        { label: "Participant Management", href: "/help/tutorials/your-first-study" },
        { label: "Simulation Mode", href: "/help/tutorials/simulation-mode" },
      ],
    },
    {
      title: "Analysis & Data",
      description: "Analyze trial results and export research data.",
      icon: BarChart3,
      items: [
        { label: "Data & Analysis Guide", href: "/help/tutorials/data-and-analysis" },
        { label: "Forms & Surveys", href: "/help/tutorials/forms-and-surveys" },
        { label: "Exporting Data", href: "/help/tutorials/data-and-analysis" },
      ],
    },
  ];

  return (
    <PageLayout
      title="Help Center"
      description="Documentation, guides, and support for HRIStudio researchers."
    >
      <div className="grid gap-6 md:grid-cols-2">
        {guides.map((guide, index) => (
          <Card key={index}>
            <CardHeader>
              <div className="flex items-center gap-2">
                <div className="bg-primary/10 rounded-lg p-2">
                  <guide.icon className="text-primary h-5 w-5" />
                </div>
                <CardTitle className="text-xl">{guide.title}</CardTitle>
              </div>
              <CardDescription>{guide.description}</CardDescription>
            </CardHeader>
            <CardContent>
              <ul className="space-y-2">
                {guide.items.map((item, i) => (
                  <li key={i}>
                    <Button
                      variant="link"
                      className="text-foreground hover:text-primary h-auto justify-start p-0 font-normal"
                      asChild
                    >
                      <Link href={item.href}>
                        <FileText className="text-muted-foreground mr-2 h-4 w-4" />
                        {item.label}
                      </Link>
                    </Button>
                  </li>
                ))}
              </ul>
            </CardContent>
          </Card>
        ))}
      </div>

      <div className="mt-8">
        <h2 className="mb-4 text-2xl font-bold tracking-tight">
          Video Tutorials
        </h2>
        <div className="grid gap-6 md:grid-cols-3">
          {[
            "Introduction to HRIStudio",
            "Advanced Flow Control",
            "ROS2 Integration Deep Dive",
          ].map((title, i) => (
            <Card key={i} className="overflow-hidden">
              <div className="bg-muted group hover:bg-muted/80 relative flex aspect-video cursor-pointer items-center justify-center transition-colors">
                <PlayCircle className="text-muted-foreground group-hover:text-primary h-12 w-12 transition-colors" />
              </div>
              <CardHeader className="p-4">
                <CardTitle className="text-base">{title}</CardTitle>
              </CardHeader>
            </Card>
          ))}
        </div>
      </div>

      <div className="bg-muted/50 mt-8 rounded-xl border p-8 text-center">
        <div className="bg-background mx-auto mb-4 flex h-12 w-12 items-center justify-center rounded-full shadow-sm">
          <HelpCircle className="text-primary h-6 w-6" />
        </div>
        <h2 className="mb-2 text-xl font-semibold">Still need help?</h2>
        <p className="text-muted-foreground mx-auto mb-6 max-w-md">
          Contact your system administrator or check the official documentation
          for technical support.
        </p>
        <div className="flex justify-center gap-4">
          <Button variant="outline" className="gap-2">
            <ExternalLink className="h-4 w-4" />
            Official Docs
          </Button>
          <Button className="gap-2">Contact Support</Button>
        </div>
      </div>
    </PageLayout>
  );
}
