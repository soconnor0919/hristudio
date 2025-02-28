"use client";

import { ExperimentDesigner } from "~/components/experiments/experiment-designer";
import { PageContent } from "~/components/layout/page-content";
import { PageHeader } from "~/components/layout/page-header";
import { Button } from "~/components/ui/button";
import { PanelLeft } from "lucide-react";
import { useSidebar } from "~/components/ui/sidebar";
import { cn } from "~/lib/utils";

export default function ExperimentDesignerPage() {
    const { state, setOpen } = useSidebar();

    return (
        <div className="flex h-[calc(100vh-4rem)] flex-col">
            <PageHeader className="flex items-center justify-between">
                <div>
                    <h1 className="text-2xl font-semibold">Experiment Designer</h1>
                    <p className="text-muted-foreground">
                        Design your experiment workflow by dragging and connecting actions
                    </p>
                </div>
                <Button
                    variant="outline"
                    size="icon"
                    onClick={() => setOpen(state === "expanded" ? false : true)}
                >
                    <PanelLeft className="h-4 w-4" />
                </Button>
            </PageHeader>
            <PageContent className={cn(
                "flex-1 overflow-hidden p-0",
                // Adjust margin when app sidebar is collapsed
                state === "collapsed" && "ml-[calc(var(--sidebar-width-icon)_+_theme(spacing.4))]",
                state === "expanded" && "ml-[calc(var(--sidebar-width))]"
            )}>
                <ExperimentDesigner />
            </PageContent>
        </div>
    );
} 