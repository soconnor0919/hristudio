"use client";

import React, { useState } from "react";
import {
  Send,
  Hash,
  Tag,
  Clock,
  Flag,
  CheckCircle,
  Bot,
  User,
  MessageSquare,
  AlertTriangle,
  Activity,
} from "lucide-react";
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
    <div className="bg-background flex h-full flex-col">
      <div className="m-0 flex flex-1 flex-col overflow-hidden p-4">
        <div className="flex flex-1 flex-col gap-2">
          <Textarea
            placeholder={
              readOnly
                ? "Session is read-only"
                : placeholders[category] || "Type your observation here..."
            }
            className="flex-1 resize-none font-mono text-sm"
            value={note}
            onChange={(e) => setNote(e.target.value)}
            onKeyDown={handleKeyDown}
            disabled={readOnly}
          />

          <div className="flex shrink-0 flex-col gap-2">
            {/* Top Line: Category & Tags */}
            <div className="flex w-full items-center gap-2">
              <Select
                value={category}
                onValueChange={setCategory}
                disabled={readOnly}
              >
                <SelectTrigger className="h-8 w-[140px] shrink-0 text-xs">
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

              <div className="flex h-8 min-w-[80px] flex-1 items-center gap-2 rounded-md border px-2">
                <Tag
                  className={`h-3 w-3 shrink-0 ${readOnly ? "text-muted-foreground/50" : "text-muted-foreground"}`}
                />
                <input
                  type="text"
                  placeholder={readOnly ? "" : "Add tags..."}
                  className="placeholder:text-muted-foreground w-full min-w-0 flex-1 bg-transparent text-xs outline-none disabled:cursor-not-allowed"
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
            <div className="flex w-full items-center justify-end gap-2">
              {onFlagIntervention && (
                <Button
                  size="sm"
                  variant="outline"
                  onClick={() => onFlagIntervention()}
                  disabled={readOnly}
                  className="h-8 flex-1 shrink-0 border-yellow-200 bg-yellow-50 text-yellow-700 hover:bg-yellow-100 hover:text-yellow-800 sm:flex-none dark:border-yellow-700/50 dark:bg-yellow-900/20 dark:text-yellow-300 dark:hover:bg-yellow-900/40"
                >
                  <AlertTriangle className="mr-2 h-3 w-3" />
                  Intervention
                </Button>
              )}
              <Button
                size="sm"
                onClick={handleSubmit}
                disabled={isSubmitting || !note.trim() || readOnly}
                className="h-8 flex-1 shrink-0 sm:flex-none"
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
                  className="hover:bg-destructive/10 hover:text-destructive cursor-pointer px-1 py-0 text-[10px]"
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
