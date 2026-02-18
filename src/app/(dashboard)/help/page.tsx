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
                { label: "Platform Overview", href: "#" },
                { label: "Creating a New Study", href: "#" },
                { label: "Managing Team Members", href: "#" },
            ],
        },
        {
            title: "Designing Experiments",
            description: "Master the visual experiment designer and flow control.",
            icon: FlaskConical,
            items: [
                { label: "Using the Visual Designer", href: "#" },
                { label: "Robot Actions & Plugins", href: "#" },
                { label: "Variables & Logic", href: "#" },
            ],
        },
        {
            title: "Running Trials",
            description: "Execute experiments and manage Wizard of Oz sessions.",
            icon: PlayCircle,
            items: [
                { label: "Wizard Interface Guide", href: "#" },
                { label: "Participant Management", href: "#" },
                { label: "Handling Robot Errors", href: "#" },
            ],
        },
        {
            title: "Analysis & Data",
            description: "Analyze trial results and export research data.",
            icon: BarChart3,
            items: [
                { label: "Understanding Analytics", href: "#" },
                { label: "Exporting Data (CSV/JSON)", href: "#" },
                { label: "Video Replay & Annotation", href: "#" },
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
                                <div className="p-2 bg-primary/10 rounded-lg">
                                    <guide.icon className="h-5 w-5 text-primary" />
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
                                            className="h-auto p-0 text-foreground hover:text-primary justify-start font-normal"
                                            asChild
                                        >
                                            <Link href={item.href}>
                                                <FileText className="mr-2 h-4 w-4 text-muted-foreground" />
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
                <h2 className="text-2xl font-bold tracking-tight mb-4">
                    Video Tutorials
                </h2>
                <div className="grid gap-6 md:grid-cols-3">
                    {[
                        "Introduction to HRIStudio",
                        "Advanced Flow Control",
                        "ROS2 Integration Deep Dive",
                    ].map((title, i) => (
                        <Card key={i} className="overflow-hidden">
                            <div className="aspect-video bg-muted flex items-center justify-center relative group cursor-pointer hover:bg-muted/80 transition-colors">
                                <PlayCircle className="h-12 w-12 text-muted-foreground group-hover:text-primary transition-colors" />
                            </div>
                            <CardHeader className="p-4">
                                <CardTitle className="text-base">{title}</CardTitle>
                            </CardHeader>
                        </Card>
                    ))}
                </div>
            </div>

            <div className="mt-8 bg-muted/50 rounded-xl p-8 text-center border">
                <div className="mx-auto w-12 h-12 bg-background rounded-full flex items-center justify-center mb-4 shadow-sm">
                    <HelpCircle className="h-6 w-6 text-primary" />
                </div>
                <h2 className="text-xl font-semibold mb-2">Still need help?</h2>
                <p className="text-muted-foreground mb-6 max-w-md mx-auto">
                    Contact your system administrator or check the official documentation for technical support.
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
