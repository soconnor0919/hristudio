"use client";

import React, { useState } from "react";
import { Send, Hash, Tag, Clock, Flag, CheckCircle, Bot, User, MessageSquare, AlertTriangle, Activity } from "lucide-react";
import { Button } from "~/components/ui/button";
import { Textarea } from "~/components/ui/textarea";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Badge } from "~/components/ui/badge";
import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
} from "~/components/ui/select";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "~/components/ui/tabs";


interface TrialEvent {
    type: string;
    timestamp: Date;
    data?: unknown;
    message?: string;
}

interface WizardObservationPaneProps {
    onAddAnnotation: (
        description: string,
        category?: string,
        tags?: string[],
    ) => Promise<void>;
    onFlagIntervention?: () => Promise<void> | void;
    isSubmitting?: boolean;
    readOnly?: boolean;

}

export function WizardObservationPane({
    onAddAnnotation,
    onFlagIntervention,
    isSubmitting = false,
    trialEvents = [],
    readOnly = false,
}: WizardObservationPaneProps & { trialEvents?: TrialEvent[] }) {
    const [note, setNote] = useState("");
    const [category, setCategory] = useState("observation");
    const [tags, setTags] = useState<string[]>([]);
    const [currentTag, setCurrentTag] = useState("");

    const placeholders: Record<string, string> = {
        observation: "Type your observation here...",
        participant_behavior: "Describe the participant's behavior...",
        system_issue: "Describe the system issue...",
        success: "Describe the success...",
        failure: "Describe the failure...",
    };

    const handleSubmit = async () => {
        if (!note.trim()) return;

        await onAddAnnotation(note, category, tags);
        setNote("");
        setTags([]);
        setCurrentTag("");
    };

    const handleKeyDown = (e: React.KeyboardEvent) => {
        if (e.key === "Enter" && (e.metaKey || e.ctrlKey)) {
            handleSubmit();
        }
    };

    const addTag = () => {
        const trimmed = currentTag.trim();
        if (trimmed && !tags.includes(trimmed)) {
            setTags([...tags, trimmed]);
            setCurrentTag("");
        }
    };

    return (
        <div className="flex h-full flex-col bg-background">
            <div className="flex-1 flex flex-col p-4 m-0 overflow-hidden">
                <div className="flex flex-1 flex-col gap-2">
                    <Textarea
                        placeholder={readOnly ? "Session is read-only" : (placeholders[category] || "Type your observation here...")}
                        className="flex-1 resize-none font-mono text-sm"
                        value={note}
                        onChange={(e) => setNote(e.target.value)}
                        onKeyDown={handleKeyDown}
                        disabled={readOnly}
                    />

                    <div className="flex flex-col gap-2 shrink-0">
                        {/* Top Line: Category & Tags */}
                        <div className="flex items-center gap-2 w-full">
                            <Select value={category} onValueChange={setCategory} disabled={readOnly}>
                                <SelectTrigger className="w-[140px] h-8 text-xs shrink-0">
                                    <SelectValue placeholder="Category" />
                                </SelectTrigger>
                                <SelectContent>
                                    <SelectItem value="observation">Observation</SelectItem>
                                    <SelectItem value="participant_behavior">Behavior</SelectItem>
                                    <SelectItem value="system_issue">System Issue</SelectItem>
                                    <SelectItem value="success">Success</SelectItem>
                                    <SelectItem value="failure">Failure</SelectItem>
                                </SelectContent>
                            </Select>

                            <div className="flex flex-1 min-w-[80px] items-center gap-2 rounded-md border px-2 h-8">
                                <Tag className={`h-3 w-3 shrink-0 ${readOnly ? "text-muted-foreground/50" : "text-muted-foreground"}`} />
                                <input
                                    type="text"
                                    placeholder={readOnly ? "" : "Add tags..."}
                                    className="flex-1 bg-transparent text-xs outline-none placeholder:text-muted-foreground disabled:cursor-not-allowed w-full min-w-0"
                                    value={currentTag}
                                    onChange={(e) => setCurrentTag(e.target.value)}
                                    onKeyDown={(e) => {
                                        if (e.key === "Enter") {
                                            e.preventDefault();
                                            addTag();
                                        }
                                    }}
                                    onBlur={addTag}
                                    disabled={readOnly}
                                />
                            </div>
                        </div>

                        {/* Bottom Line: Actions */}
                        <div className="flex items-center justify-end gap-2 w-full">
                            {onFlagIntervention && (
                                <Button
                                    size="sm"
                                    variant="outline"
                                    onClick={() => onFlagIntervention()}
                                    disabled={readOnly}
                                    className="h-8 shrink-0 flex-1 sm:flex-none border-yellow-200 bg-yellow-50 text-yellow-700 hover:bg-yellow-100 hover:text-yellow-800 dark:bg-yellow-900/20 dark:text-yellow-300 dark:border-yellow-700/50 dark:hover:bg-yellow-900/40"
                                >
                                    <AlertTriangle className="mr-2 h-3 w-3" />
                                    Intervention
                                </Button>
                            )}
                            <Button
                                size="sm"
                                onClick={handleSubmit}
                                disabled={isSubmitting || !note.trim() || readOnly}
                                className="h-8 shrink-0 flex-1 sm:flex-none"
                            >
                                <Send className="mr-2 h-3 w-3" />
                                Save Note
                            </Button>
                        </div>
                    </div>

                    {tags.length > 0 && (
                        <div className="flex flex-wrap gap-1">
                            {tags.map((tag) => (
                                <Badge
                                    key={tag}
                                    variant="secondary"
                                    className="px-1 py-0 text-[10px] cursor-pointer hover:bg-destructive/10 hover:text-destructive"
                                    onClick={() => setTags(tags.filter((t) => t !== tag))}
                                >
                                    #{tag}
                                </Badge>
                            ))}
                        </div>
                    )}
                </div>
            </div>
        </div>
    );
}
