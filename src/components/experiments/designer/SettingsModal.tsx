"use client";

import { SettingsTab } from "./tabs/SettingsTab";
import {
    Dialog,
    DialogContent,
    DialogDescription,
    DialogHeader,
    DialogTitle,
} from "~/components/ui/dialog";

interface SettingsModalProps {
    open: boolean;
    onOpenChange: (open: boolean) => void;
    experiment: {
        id: string;
        name: string;
        description: string | null;
        status: string;
        studyId: string;
        createdAt: Date;
        updatedAt: Date;
        study: {
            id: string;
            name: string;
        };
    };
    designStats?: {
        stepCount: number;
        actionCount: number;
    };
}

export function SettingsModal({
    open,
    onOpenChange,
    experiment,
    designStats,
}: SettingsModalProps) {
    return (
        <Dialog open={open} onOpenChange={onOpenChange}>
            <DialogContent className="max-w-4xl max-h-[90vh] p-0">
                <DialogHeader className="sr-only">
                    <DialogTitle>Experiment Settings</DialogTitle>
                    <DialogDescription>
                        Configure experiment metadata and status
                    </DialogDescription>
                </DialogHeader>
                <SettingsTab experiment={experiment} designStats={designStats} />
            </DialogContent>
        </Dialog>
    );
}
