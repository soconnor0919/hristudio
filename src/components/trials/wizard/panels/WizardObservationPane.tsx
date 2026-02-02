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
import { HorizontalTimeline } from "~/components/trials/timeline/HorizontalTimeline";

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
    isSubmitting?: boolean;
}

export function WizardObservationPane({
    onAddAnnotation,
    isSubmitting = false,
    trialEvents = [],
}: WizardObservationPaneProps & { trialEvents?: TrialEvent[] }) {
    const [note, setNote] = useState("");
    const [category, setCategory] = useState("observation");
    const [tags, setTags] = useState<string[]>([]);
    const [currentTag, setCurrentTag] = useState("");

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
        <div className="flex h-full flex-col border-t bg-background">
            <Tabs defaultValue="notes" className="flex h-full flex-col">
                <div className="border-b px-4 bg-muted/30">
                    <TabsList className="h-9 -mb-px bg-transparent p-0">
                        <TabsTrigger value="notes" className="h-9 rounded-none border-b-2 border-transparent data-[state=active]:border-primary data-[state=active]:bg-transparent px-4 pb-2 pt-2 font-medium text-muted-foreground data-[state=active]:text-foreground shadow-none">
                            Notes & Observations
                        </TabsTrigger>
                        <TabsTrigger value="timeline" className="h-9 rounded-none border-b-2 border-transparent data-[state=active]:border-primary data-[state=active]:bg-transparent px-4 pb-2 pt-2 font-medium text-muted-foreground data-[state=active]:text-foreground shadow-none">
                            Timeline
                        </TabsTrigger>
                    </TabsList>
                </div>

                <TabsContent value="notes" className="flex-1 flex flex-col p-4 m-0 data-[state=inactive]:hidden">
                    <div className="flex flex-1 flex-col gap-2">
                        <Textarea
                            placeholder="Type your observation here..."
                            className="flex-1 resize-none font-mono text-sm"
                            value={note}
                            onChange={(e) => setNote(e.target.value)}
                            onKeyDown={handleKeyDown}
                        />

                        <div className="flex items-center gap-2">
                            <Select value={category} onValueChange={setCategory}>
                                <SelectTrigger className="w-[140px] h-8 text-xs">
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

                            <div className="flex flex-1 items-center gap-2 rounded-md border px-2 h-8">
                                <Tag className="h-3 w-3 text-muted-foreground" />
                                <input
                                    type="text"
                                    placeholder="Add tags..."
                                    className="flex-1 bg-transparent text-xs outline-none placeholder:text-muted-foreground"
                                    value={currentTag}
                                    onChange={(e) => setCurrentTag(e.target.value)}
                                    onKeyDown={(e) => {
                                        if (e.key === "Enter") {
                                            e.preventDefault();
                                            addTag();
                                        }
                                    }}
                                    onBlur={addTag}
                                />
                            </div>

                            <Button
                                size="sm"
                                onClick={handleSubmit}
                                disabled={isSubmitting || !note.trim()}
                                className="h-8"
                            >
                                <Send className="mr-2 h-3 w-3" />
                                Add Note
                            </Button>
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
                </TabsContent>

                <TabsContent value="timeline" className="flex-1 m-0 min-h-0 p-4 data-[state=inactive]:hidden">
                    <HorizontalTimeline events={trialEvents} />
                </TabsContent>
            </Tabs>
        </div>
    );
}
