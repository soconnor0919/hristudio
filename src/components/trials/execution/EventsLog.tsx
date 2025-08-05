"use client";

import { format, formatDistanceToNow } from "date-fns";
import {
    Activity, AlertTriangle, ArrowRight, Bot, Camera, CheckCircle, Eye, Hand, MessageSquare, Pause, Play, Settings, User, Volume2, XCircle
} from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { ScrollArea } from "~/components/ui/scroll-area";
import { api } from "~/trpc/react";

interface EventsLogProps {
  trialId: string;
  refreshKey: number;
  isLive: boolean;
  maxEvents?: number;
  realtimeEvents?: any[];
  isWebSocketConnected?: boolean;
}

interface TrialEvent {
  id: string;
  trialId: string;
  eventType: string;
  timestamp: Date;
  data: any;
  notes: string | null;
  createdAt: Date;
}

const eventTypeConfig = {
  trial_started: {
    label: "Trial Started",
    icon: Play,
    color: "text-green-600",
    bgColor: "bg-green-100",
    importance: "high",
  },
  trial_completed: {
    label: "Trial Completed",
    icon: CheckCircle,
    color: "text-blue-600",
    bgColor: "bg-blue-100",
    importance: "high",
  },
  trial_aborted: {
    label: "Trial Aborted",
    icon: XCircle,
    color: "text-red-600",
    bgColor: "bg-red-100",
    importance: "high",
  },
  step_transition: {
    label: "Step Change",
    icon: ArrowRight,
    color: "text-purple-600",
    bgColor: "bg-purple-100",
    importance: "medium",
  },
  wizard_action: {
    label: "Wizard Action",
    icon: User,
    color: "text-blue-600",
    bgColor: "bg-blue-100",
    importance: "medium",
  },
  robot_action: {
    label: "Robot Action",
    icon: Bot,
    color: "text-green-600",
    bgColor: "bg-green-100",
    importance: "medium",
  },
  wizard_intervention: {
    label: "Intervention",
    icon: Hand,
    color: "text-orange-600",
    bgColor: "bg-orange-100",
    importance: "high",
  },
  manual_intervention: {
    label: "Manual Control",
    icon: Hand,
    color: "text-orange-600",
    bgColor: "bg-orange-100",
    importance: "high",
  },
  emergency_action: {
    label: "Emergency",
    icon: AlertTriangle,
    color: "text-red-600",
    bgColor: "bg-red-100",
    importance: "critical",
  },
  emergency_stop: {
    label: "Emergency Stop",
    icon: AlertTriangle,
    color: "text-red-600",
    bgColor: "bg-red-100",
    importance: "critical",
  },
  recording_control: {
    label: "Recording",
    icon: Camera,
    color: "text-indigo-600",
    bgColor: "bg-indigo-100",
    importance: "low",
  },
  video_control: {
    label: "Video Control",
    icon: Camera,
    color: "text-indigo-600",
    bgColor: "bg-indigo-100",
    importance: "low",
  },
  audio_control: {
    label: "Audio Control",
    icon: Volume2,
    color: "text-indigo-600",
    bgColor: "bg-indigo-100",
    importance: "low",
  },
  pause_interaction: {
    label: "Paused",
    icon: Pause,
    color: "text-yellow-600",
    bgColor: "bg-yellow-100",
    importance: "medium",
  },
  participant_response: {
    label: "Participant",
    icon: MessageSquare,
    color: "text-slate-600",
    bgColor: "bg-slate-100",
    importance: "medium",
  },
  system_event: {
    label: "System",
    icon: Settings,
    color: "text-slate-600",
    bgColor: "bg-slate-100",
    importance: "low",
  },
  annotation: {
    label: "Annotation",
    icon: MessageSquare,
    color: "text-blue-600",
    bgColor: "bg-blue-100",
    importance: "medium",
  },
  default: {
    label: "Event",
    icon: Activity,
    color: "text-slate-600",
    bgColor: "bg-slate-100",
    importance: "low",
  },
};

export function EventsLog({
  trialId,
  refreshKey,
  isLive,
  maxEvents = 100,
  realtimeEvents = [],
  isWebSocketConnected = false,
}: EventsLogProps) {
  const [events, setEvents] = useState<TrialEvent[]>([]);
  const [isAutoScrollEnabled, setIsAutoScrollEnabled] = useState(true);
  const [filter, setFilter] = useState<string>("all");
  const scrollAreaRef = useRef<HTMLDivElement>(null);
  const bottomRef = useRef<HTMLDivElement>(null);

  // Fetch trial events (less frequent when WebSocket is connected)
  const { data: eventsData, isLoading } = api.trials.getEvents.useQuery(
    {
      trialId,
      limit: maxEvents,
      type: filter === "all" ? undefined : filter as "error" | "custom" | "trial_start" | "trial_end" | "step_start" | "step_end" | "wizard_intervention",
    },
    {
      refetchInterval: isLive && !isWebSocketConnected ? 2000 : 10000, // Less frequent polling when WebSocket is active
      refetchOnWindowFocus: false,
      enabled: !isWebSocketConnected || !isLive, // Reduce API calls when WebSocket is connected
    },
  );

  // Convert WebSocket events to trial events format
  const convertWebSocketEvent = (wsEvent: any): TrialEvent => ({
    id: `ws-${Date.now()}-${Math.random()}`,
    trialId,
    eventType:
      wsEvent.type === "trial_action_executed"
        ? "wizard_action"
        : wsEvent.type === "intervention_logged"
          ? "wizard_intervention"
          : wsEvent.type === "step_changed"
            ? "step_transition"
            : wsEvent.type || "system_event",
    timestamp: new Date(wsEvent.data?.timestamp || Date.now()),
    data: wsEvent.data || {},
    notes: wsEvent.data?.notes || null,
    createdAt: new Date(wsEvent.data?.timestamp || Date.now()),
  });

  // Update events when data changes (prioritize WebSocket events)
  useEffect(() => {
    let newEvents: TrialEvent[] = [];

    // Add database events
    if (eventsData) {
      newEvents = eventsData.map((event) => ({
        ...event,
        timestamp: new Date(event.timestamp),
        createdAt: new Date(event.timestamp),
        notes: null, // Add required field
      }));
    }

    // Add real-time WebSocket events
    if (realtimeEvents.length > 0) {
      const wsEvents = realtimeEvents.map(convertWebSocketEvent);
      newEvents = [...newEvents, ...wsEvents];
    }

    // Sort by timestamp and remove duplicates
    const uniqueEvents = newEvents
      .sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime())
      .filter(
        (event, index, arr) =>
          index ===
          arr.findIndex(
            (e) =>
              e.eventType === event.eventType &&
              Math.abs(e.timestamp.getTime() - event.timestamp.getTime()) <
                1000,
          ),
      )
      .slice(-maxEvents); // Keep only the most recent events

    setEvents(uniqueEvents);
  }, [eventsData, refreshKey, realtimeEvents, trialId, maxEvents]);

  // Auto-scroll to bottom when new events arrive
  useEffect(() => {
    if (isAutoScrollEnabled && bottomRef.current) {
      bottomRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [events, isAutoScrollEnabled]);

  const getEventConfig = (eventType: string) => {
    return (
      eventTypeConfig[eventType as keyof typeof eventTypeConfig] ||
      eventTypeConfig.default
    );
  };

  const formatEventData = (eventType: string, data: any) => {
    if (!data) return null;

    switch (eventType) {
      case "step_transition":
        return `Step ${data.from_step + 1} → Step ${data.to_step + 1}${data.step_name ? `: ${data.step_name}` : ""}`;

      case "wizard_action":
        return `${data.action_type ? data.action_type.replace(/_/g, " ") : "Action executed"}${data.step_name ? ` in ${data.step_name}` : ""}`;

      case "robot_action":
        return `${data.action_name || "Robot action"}${data.parameters ? ` with parameters` : ""}`;

      case "emergency_action":
        return `Emergency: ${data.emergency_type ? data.emergency_type.replace(/_/g, " ") : "Unknown"}`;

      case "recording_control":
        return `Recording ${data.action === "start_recording" ? "started" : "stopped"}`;

      case "video_control":
        return `Video ${data.action === "video_on" ? "enabled" : "disabled"}`;

      case "audio_control":
        return `Audio ${data.action === "audio_on" ? "enabled" : "disabled"}`;

      case "wizard_intervention":
        return (
          data.content || data.intervention_type || "Intervention recorded"
        );

      default:
        if (typeof data === "string") return data;
        if (data.message) return data.message;
        if (data.description) return data.description;
        return null;
    }
  };

  const getEventImportanceOrder = (importance: string) => {
    const order = { critical: 0, high: 1, medium: 2, low: 3 };
    return order[importance as keyof typeof order] || 4;
  };

  // Group events by time proximity (within 30 seconds)
  const groupedEvents = events.reduce(
    (groups: TrialEvent[][], event, index) => {
      if (
        index === 0 ||
        Math.abs(
          event.timestamp.getTime() - (events[index - 1]?.timestamp.getTime() ?? 0),
        ) > 30000
      ) {
        groups.push([event]);
      } else {
        groups[groups.length - 1]?.push(event);
      }
      return groups;
    },
    [],
  );

  const uniqueEventTypes = Array.from(new Set(events.map((e) => e.eventType)));

  if (isLoading) {
    return (
      <div className="flex h-full flex-col">
        <div className="border-b border-slate-200 p-4">
          <h3 className="flex items-center space-x-2 font-medium text-slate-900">
            <Activity className="h-4 w-4" />
            <span>Events Log</span>
          </h3>
        </div>
        <div className="flex flex-1 items-center justify-center">
          <div className="text-center">
            <Activity className="mx-auto mb-2 h-6 w-6 animate-pulse text-slate-400" />
            <p className="text-sm text-slate-500">Loading events...</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="flex h-full flex-col">
      {/* Header */}
      <div className="border-b border-slate-200 p-4">
        <div className="mb-3 flex items-center justify-between">
          <h3 className="flex items-center space-x-2 font-medium text-slate-900">
            <Activity className="h-4 w-4" />
            <span>Events Log</span>
            {isLive && (
              <div className="flex items-center space-x-1">
                <div
                  className={`h-2 w-2 animate-pulse rounded-full ${
                    isWebSocketConnected ? "bg-green-500" : "bg-red-500"
                  }`}
                ></div>
                <span
                  className={`text-xs ${
                    isWebSocketConnected ? "text-green-600" : "text-red-600"
                  }`}
                >
                  {isWebSocketConnected ? "REAL-TIME" : "LIVE"}
                </span>
              </div>
            )}
          </h3>
          <div className="flex items-center space-x-2">
            <Badge variant="outline" className="text-xs">
              {events.length} events
            </Badge>
            {isWebSocketConnected && (
              <Badge className="bg-green-100 text-xs text-green-800">
                Real-time
              </Badge>
            )}
          </div>
        </div>

        {/* Filter Controls */}
        <div className="flex items-center space-x-2">
          <Button
            variant={filter === "all" ? "default" : "outline"}
            size="sm"
            onClick={() => setFilter("all")}
            className="h-7 text-xs"
          >
            All
          </Button>
          <Button
            variant={filter === "wizard_action" ? "default" : "outline"}
            size="sm"
            onClick={() => setFilter("wizard_action")}
            className="h-7 text-xs"
          >
            Wizard
          </Button>
          <Button
            variant={filter === "robot_action" ? "default" : "outline"}
            size="sm"
            onClick={() => setFilter("robot_action")}
            className="h-7 text-xs"
          >
            Robot
          </Button>
          <Button
            variant={filter === "emergency_action" ? "default" : "outline"}
            size="sm"
            onClick={() => setFilter("emergency_action")}
            className="h-7 text-xs"
          >
            Emergency
          </Button>
        </div>
      </div>

      {/* Events List */}
      <ScrollArea className="flex-1" ref={scrollAreaRef}>
        <div className="space-y-4 p-4">
          {events.length === 0 ? (
            <div className="py-8 text-center">
              <Activity className="mx-auto mb-2 h-8 w-8 text-slate-300" />
              <p className="text-sm text-slate-500">No events yet</p>
              <p className="mt-1 text-xs text-slate-400">
                Events will appear here as the trial progresses
              </p>
            </div>
          ) : (
            groupedEvents.map((group, groupIndex) => (
              <div key={groupIndex} className="space-y-2">
                {/* Time Header */}
                <div className="flex items-center space-x-2">
                  <div className="text-xs font-medium text-slate-500">
                    {group[0] ? format(group[0].timestamp, "HH:mm:ss") : ""}
                  </div>
                  <div className="h-px flex-1 bg-slate-200"></div>
                  <div className="text-xs text-slate-400">
                    {group[0] ? formatDistanceToNow(group[0].timestamp, {
                      addSuffix: true,
                    }) : ""}
                  </div>
                </div>

                {/* Events in Group */}
                {group
                  .sort(
                    (a, b) =>
                      getEventImportanceOrder(
                        getEventConfig(a.eventType).importance,
                      ) -
                      getEventImportanceOrder(
                        getEventConfig(b.eventType).importance,
                      ),
                  )
                  .map((event) => {
                    const config = getEventConfig(event.eventType);
                    const EventIcon = config.icon;
                    const eventData = formatEventData(
                      event.eventType,
                      event.data,
                    );

                    return (
                      <div
                        key={event.id}
                        className={`flex items-start space-x-3 rounded-lg border p-3 transition-colors ${
                          config.importance === "critical"
                            ? "border-red-200 bg-red-50"
                            : config.importance === "high"
                              ? "border-amber-200 bg-amber-50"
                              : "border-slate-200 bg-slate-50 hover:bg-slate-100"
                        }`}
                      >
                        <div
                          className={`flex h-6 w-6 flex-shrink-0 items-center justify-center rounded-full ${config.bgColor}`}
                        >
                          <EventIcon className={`h-3 w-3 ${config.color}`} />
                        </div>

                        <div className="min-w-0 flex-1">
                          <div className="flex items-center space-x-2">
                            <span className="text-sm font-medium text-slate-900">
                              {config.label}
                            </span>
                            {config.importance === "critical" && (
                              <Badge variant="destructive" className="text-xs">
                                CRITICAL
                              </Badge>
                            )}
                            {config.importance === "high" && (
                              <Badge
                                variant="outline"
                                className="border-amber-300 text-xs text-amber-600"
                              >
                                HIGH
                              </Badge>
                            )}
                          </div>

                          {eventData && (
                            <p className="mt-1 text-sm break-words text-slate-600">
                              {eventData}
                            </p>
                          )}

                          {event.notes && (
                            <p className="mt-1 text-xs text-slate-500 italic">
                              "{event.notes}"
                            </p>
                          )}

                          {event.data && Object.keys(event.data).length > 0 && (
                            <details className="mt-2">
                              <summary className="cursor-pointer text-xs text-blue-600 hover:text-blue-800">
                                View details
                              </summary>
                              <pre className="mt-1 overflow-x-auto rounded border bg-white p-2 text-xs text-slate-600">
                                {JSON.stringify(event.data, null, 2)}
                              </pre>
                            </details>
                          )}
                        </div>

                        <div className="flex-shrink-0 text-xs text-slate-400">
                          {format(event.timestamp, "HH:mm")}
                        </div>
                      </div>
                    );
                  })}
              </div>
            ))
          )}

          <div ref={bottomRef} />
        </div>
      </ScrollArea>

      {/* Auto-scroll Control */}
      {events.length > 0 && (
        <div className="border-t border-slate-200 p-2">
          <Button
            variant="ghost"
            size="sm"
            onClick={() => setIsAutoScrollEnabled(!isAutoScrollEnabled)}
            className="w-full text-xs"
          >
            <Eye className="mr-1 h-3 w-3" />
            Auto-scroll: {isAutoScrollEnabled ? "ON" : "OFF"}
          </Button>
        </div>
      )}
    </div>
  );
}
