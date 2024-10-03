"use client";

import React, { useEffect } from 'react';
import { Card, CardContent } from "~/components/ui/card";
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from "~/components/ui/tooltip";
import { useStudyContext } from '~/context/StudyContext';
import { StudySelector } from './StudySelector';
import { CreateStudyDialog } from '~/components/study/CreateStudyDialog';
import { Study } from '~/types/Study';

interface StudyHeaderProps {
    pageTitle: string;
}

export const StudyHeader: React.FC<StudyHeaderProps> = ({ pageTitle }) => {
    const { studies, selectedStudy, setSelectedStudy, validateAndSetSelectedStudy, fetchAndSetStudies } = useStudyContext();

    useEffect(() => {
        const savedStudyId = localStorage.getItem('selectedStudyId');
        if (savedStudyId) {
            validateAndSetSelectedStudy(parseInt(savedStudyId, 10));
        }
    }, [validateAndSetSelectedStudy]);

    const handleStudyChange = (studyId: string) => {
        const study = studies.find(s => s.id.toString() === studyId);
        if (study) {
            setSelectedStudy(study);
            localStorage.setItem('selectedStudyId', studyId);
        }
    };

    const createStudy = async (newStudy: Omit<Study, "id">) => {
        const response = await fetch('/api/studies', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(newStudy),
        });
        if (!response.ok) {
            throw new Error('Failed to create study');
        }
        const createdStudy = await response.json();
        await fetchAndSetStudies();
        return createdStudy;
    };

    const handleCreateStudy = async (newStudy: Omit<Study, "id">) => {
        const createdStudy = await createStudy(newStudy);
        setSelectedStudy(createdStudy);
        localStorage.setItem('selectedStudyId', createdStudy.id.toString());
    };

    return (
        <Card className="mt-2 lg:mt-0">
            <CardContent className="flex justify-between items-center p-4">
                <TooltipProvider>
                    <Tooltip>
                        <TooltipTrigger asChild>
                            <h2 className="text-2xl font-bold truncate max-w-[200px]">
                                {pageTitle}
                            </h2>
                        </TooltipTrigger>
                        <TooltipContent>
                            <p>{selectedStudy ? selectedStudy.title : 'No study selected'}</p>
                        </TooltipContent>
                    </Tooltip>
                </TooltipProvider>
                <div className="flex items-center space-x-2">
                    <StudySelector
                        studies={studies}
                        selectedStudy={selectedStudy}
                        onStudyChange={handleStudyChange}
                    />
                    <CreateStudyDialog onCreateStudy={(study: Omit<Study, "id">) => handleCreateStudy(study as Study)} />
                </div>
            </CardContent>
        </Card>
    );
};