"use client";

import { Clock, Activity, User, Bot, AlertCircle } from "lucide-react";
import { formatDistanceToNow } from "date-fns";
import { ScrollArea } from "~/components/ui/scroll-area";
import { Badge } from "~/components/ui/badge";
import { useEffect, useState } from "react";
import type { WebSocketMessage } from "~/hooks/useWebSocket";

interface EventsLogSidebarProps {
  events: WebSocketMessage[];
  maxEvents?: number;
  showTimestamps?: boolean;
}

const getEventIcon = (eventType: string) => {
  switch (eventType) {
    case "trial_status":
    case "trial_action_executed":
      return Activity;
    case "step_changed":
      return Clock;
    case "wizard_intervention":
    case "intervention_logged":
      return User;
    case "robot_action":
      return Bot;
    case "error":
      return AlertCircle;
    default:
      return Activity;
  }
};

const getEventVariant = (eventType: string) => {
  switch (eventType) {
    case "error":
      return "destructive" as const;
    case "wizard_intervention":
    case "intervention_logged":
      return "secondary" as const;
    case "trial_status":
      return "default" as const;
    default:
      return "outline" as const;
  }
};

const formatEventData = (event: WebSocketMessage): string => {
  switch (event.type) {
    case "trial_status":
      const trialData = event.data as { trial: { status: string } };
      return `Trial status: ${trialData.trial.status}`;

    case "step_changed":
      const stepData = event.data as {
        to_step: number;
        step_name?: string;
      };
      return `Step ${stepData.to_step + 1}${stepData.step_name ? `: ${stepData.step_name}` : ""}`;

    case "trial_action_executed":
      const actionData = event.data as { action_type: string };
      return `Action: ${actionData.action_type}`;

    case "wizard_intervention":
    case "intervention_logged":
      const interventionData = event.data as { content?: string };
      return interventionData.content ?? "Wizard intervention";

    case "error":
      const errorData = event.data as { message?: string };
      return errorData.message ?? "System error";

    default:
      return `Event: ${event.type}`;
  }
};

const getEventTimestamp = (event: WebSocketMessage): Date => {
  const data = event.data as { timestamp?: number };
  return data.timestamp ? new Date(data.timestamp) : new Date();
};

export function EventsLogSidebar({
  events,
  maxEvents = 10,
  showTimestamps = true,
}: EventsLogSidebarProps) {
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  const displayEvents = events.slice(-maxEvents).reverse();

  if (displayEvents.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center py-8 text-center">
        <Clock className="text-muted-foreground mb-3 h-8 w-8" />
        <p className="text-muted-foreground text-sm">No events yet</p>
        <p className="text-muted-foreground mt-1 text-xs">
          Events will appear here during trial execution
        </p>
      </div>
    );
  }

  return (
    <ScrollArea className="h-64">
      <div className="space-y-3">
        {displayEvents.map((event, index) => {
          const Icon = getEventIcon(event.type);
          const timestamp = getEventTimestamp(event);
          const eventText = formatEventData(event);

          return (
            <div key={index} className="flex items-start gap-3">
              <div className="mt-0.5 flex-shrink-0">
                <div className="bg-muted rounded-full p-1.5">
                  <Icon className="h-3 w-3" />
                </div>
              </div>

              <div className="min-w-0 flex-1">
                <div className="mb-1 flex items-center gap-2">
                  <Badge
                    variant={getEventVariant(event.type)}
                    className="text-xs"
                  >
                    {event.type.replace(/_/g, " ")}
                  </Badge>
                  {showTimestamps && isClient && (
                    <span className="text-muted-foreground text-xs">
                      {formatDistanceToNow(timestamp, { addSuffix: true })}
                    </span>
                  )}
                </div>

                <p className="text-foreground text-sm break-words">
                  {eventText}
                </p>
              </div>
            </div>
          );
        })}
      </div>
    </ScrollArea>
  );
}
