import { notFound } from "next/navigation";
import { type experiments } from "~/server/db/schema";
import { type InferSelectModel } from "drizzle-orm";

type Experiment = InferSelectModel<typeof experiments>;
import { api } from "~/trpc/server";
import { ExperimentForm } from "./experiment-form";
import {
    EntityView,
    EntityViewHeader,
    EntityViewSection,
} from "~/components/ui/entity-view";
import { Button } from "~/components/ui/button";
import Link from "next/link";
import { ArrowLeft } from "lucide-react";

interface ExperimentEditPageProps {
    params: Promise<{ id: string; experimentId: string }>;
}

export default async function ExperimentEditPage({
    params,
}: ExperimentEditPageProps) {
    const { id: studyId, experimentId } = await params;

    const experiment = await api.experiments.get({ id: experimentId });

    if (!experiment) {
        notFound();
    }

    // Ensure experiment belongs to study
    if (experiment.studyId !== studyId) {
        notFound();
    }

    // Convert to type expected by form
    const experimentData: Experiment = {
        ...experiment,
        status: experiment.status as Experiment["status"],
    };

    return (
        <EntityView>
            <EntityViewHeader
                title="Edit Experiment"
                subtitle={`Update settings for ${experiment.name}`}
                icon="Edit"
            />

            <div className="max-w-2xl">
                <EntityViewSection title="Experiment Details" icon="Settings">
                    <ExperimentForm experiment={experimentData} />
                </EntityViewSection>
            </div>
        </EntityView>
    );
}
