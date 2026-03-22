"use client";

import { useSession } from "~/lib/auth-client";
import { useCallback, useEffect, useRef, useState } from "react";

export type TrialStatus =
  | "scheduled"
  | "in_progress"
  | "completed"
  | "aborted"
  | "failed";

export interface TrialSnapshot {
  id: string;
  status: TrialStatus;
  startedAt?: string | Date | null;
  completedAt?: string | Date | null;
}

interface ConnectionEstablishedMessage {
  type: "connection_established";
  data: {
    trialId: string;
    userId: string | null;
    role: string;
    connectedAt: number;
  };
}

interface HeartbeatResponseMessage {
  type: "heartbeat_response";
  data: {
    timestamp: number;
  };
}

interface TrialStatusMessage {
  type: "trial_status";
  data: {
    trial: TrialSnapshot;
    current_step_index: number;
    timestamp: number;
  };
}

interface TrialActionExecutedMessage {
  type: "trial_action_executed";
  data: {
    action_type: string;
    timestamp: number;
  } & Record<string, unknown>;
}

interface InterventionLoggedMessage {
  type: "intervention_logged";
  data: {
    intervention?: unknown;
    timestamp: number;
  } & Record<string, unknown>;
}

interface TrialEventMessage {
  type: "trial_event";
  data: {
    event: unknown;
    timestamp: number;
  };
}

interface TrialEventsSnapshotMessage {
  type: "trial_events_snapshot";
  data: {
    events: unknown[];
    timestamp: number;
  };
}

interface AnnotationAddedMessage {
  type: "annotation_added";
  data: {
    annotation: unknown;
    timestamp: number;
  };
}

interface PongMessage {
  type: "pong";
  data: {
    timestamp: number;
  };
}

interface StepChangedMessage {
  type: "step_changed";
  data: {
    from_step?: number;
    to_step: number;
    step_name?: string;
    timestamp: number;
  } & Record<string, unknown>;
}

interface ErrorMessage {
  type: "error";
  data: {
    message?: string;
  };
}

type KnownInboundMessage =
  | ConnectionEstablishedMessage
  | HeartbeatResponseMessage
  | TrialStatusMessage
  | TrialActionExecutedMessage
  | InterventionLoggedMessage
  | TrialEventMessage
  | TrialEventsSnapshotMessage
  | AnnotationAddedMessage
  | PongMessage
  | StepChangedMessage
  | ErrorMessage;

export type WebSocketMessage =
  | KnownInboundMessage
  | {
      type: string;
      data: unknown;
    };

export interface OutgoingMessage {
  type: string;
  data: Record<string, unknown>;
}

interface Subscription {
  trialId: string;
  onMessage?: (message: WebSocketMessage) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: Event) => void;
}

interface GlobalWSState {
  isConnected: boolean;
  isConnecting: boolean;
  connectionError: string | null;
  lastMessage: WebSocketMessage | null;
}

type StateListener = (state: GlobalWSState) => void;

class GlobalWebSocketManager {
  private ws: WebSocket | null = null;
  private subscriptions: Map<string, Subscription> = new Map();
  private stateListeners: Set<StateListener> = new Set();
  private sessionRef: { user: { id: string } } | null = null;
  private heartbeatInterval: ReturnType<typeof setInterval> | null = null;
  private reconnectTimeout: ReturnType<typeof setTimeout> | null = null;
  private attemptCount = 0;
  private maxAttempts = 5;

  private state: GlobalWSState = {
    isConnected: false,
    isConnecting: false,
    connectionError: null,
    lastMessage: null,
  };

  private setState(partial: Partial<GlobalWSState>) {
    this.state = { ...this.state, ...partial };
    this.notifyListeners();
  }

  private notifyListeners() {
    this.stateListeners.forEach((listener) => listener(this.state));
  }

  subscribe(
    session: { user: { id: string } } | null,
    subscription: Subscription,
  ) {
    this.sessionRef = session;
    this.subscriptions.set(subscription.trialId, subscription);

    if (this.subscriptions.size === 1 && !this.ws) {
      this.connect();
    }

    return () => {
      this.subscriptions.delete(subscription.trialId);
      // Don't auto-disconnect - keep global connection alive
    };
  }

  sendMessage(message: OutgoingMessage) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    }
  }

  connect() {
    if (
      this.ws?.readyState === WebSocket.CONNECTING ||
      this.ws?.readyState === WebSocket.OPEN
    ) {
      return;
    }

    if (!this.sessionRef?.user) {
      this.setState({ connectionError: "No session", isConnecting: false });
      return;
    }

    this.setState({ isConnecting: true, connectionError: null });

    const token = btoa(JSON.stringify({ userId: this.sessionRef.user.id }));
    const wsPort = process.env.NEXT_PUBLIC_WS_PORT || "3001";

    // Collect all trial IDs from subscriptions
    const trialIds = Array.from(this.subscriptions.keys());
    const trialIdParam = trialIds.length > 0 ? `&trialId=${trialIds[0]}` : "";
    const url = `ws://${typeof window !== "undefined" ? window.location.hostname : "localhost"}:${wsPort}/api/websocket?token=${token}${trialIdParam}`;

    try {
      this.ws = new WebSocket(url);

      this.ws.onopen = () => {
        console.log("[GlobalWS] Connected");
        this.setState({ isConnected: true, isConnecting: false });
        this.attemptCount = 0;
        this.startHeartbeat();

        // Subscribe to all subscribed trials
        this.subscriptions.forEach((sub) => {
          this.ws?.send(
            JSON.stringify({
              type: "subscribe",
              data: { trialId: sub.trialId },
            }),
          );
        });

        this.subscriptions.forEach((sub) => sub.onConnect?.());
      };

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data) as WebSocketMessage;
          this.setState({ lastMessage: message });

          if (message.type === "connection_established") {
            const data = (message as ConnectionEstablishedMessage).data;
            const sub = this.subscriptions.get(data.trialId);
            if (sub) {
              sub.onMessage?.(message);
            }
          } else if (
            message.type === "trial_event" ||
            message.type === "trial_status"
          ) {
            const data = (message as TrialEventMessage).data;
            const event = data.event as { trialId?: string };
            if (event?.trialId) {
              const sub = this.subscriptions.get(event.trialId);
              sub?.onMessage?.(message);
            }
          } else {
            // Broadcast to all subscriptions
            this.subscriptions.forEach((sub) => sub.onMessage?.(message));
          }
        } catch (error) {
          console.error("[GlobalWS] Failed to parse message:", error);
        }
      };

      this.ws.onclose = (event) => {
        console.log("[GlobalWS] Disconnected:", event.code);
        this.setState({ isConnected: false, isConnecting: false });
        this.stopHeartbeat();
        this.subscriptions.forEach((sub) => sub.onDisconnect?.());

        // Auto-reconnect if not intentionally closed
        if (event.code !== 1000 && this.subscriptions.size > 0) {
          this.scheduleReconnect();
        }
      };

      this.ws.onerror = (error) => {
        console.error("[GlobalWS] Error:", error);
        this.setState({
          connectionError: "Connection error",
          isConnecting: false,
        });
        this.subscriptions.forEach((sub) =>
          sub.onError?.(new Event("ws_error")),
        );
      };
    } catch (error) {
      console.error("[GlobalWS] Failed to create:", error);
      this.setState({
        connectionError: "Failed to create connection",
        isConnecting: false,
      });
    }
  }

  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
    this.stopHeartbeat();
    if (this.ws) {
      this.ws.close(1000, "Manual disconnect");
      this.ws = null;
    }
    this.setState({ isConnected: false, isConnecting: false });
  }

  private startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: "heartbeat", data: {} }));
      }
    }, 30000);
  }

  private stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  private scheduleReconnect() {
    if (this.attemptCount >= this.maxAttempts) {
      this.setState({ connectionError: "Max reconnection attempts reached" });
      return;
    }

    const delay = Math.min(30000, 1000 * Math.pow(1.5, this.attemptCount));
    this.attemptCount++;

    console.log(
      `[GlobalWS] Reconnecting in ${delay}ms (attempt ${this.attemptCount})`,
    );

    this.reconnectTimeout = setTimeout(() => {
      if (this.subscriptions.size > 0) {
        this.connect();
      }
    }, delay);
  }

  getState(): GlobalWSState {
    return this.state;
  }

  addListener(listener: StateListener) {
    this.stateListeners.add(listener);
    return () => this.stateListeners.delete(listener);
  }
}

const globalWS = new GlobalWebSocketManager();

export interface UseGlobalWebSocketOptions {
  trialId: string;
  onMessage?: (message: WebSocketMessage) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: Event) => void;
}

export interface UseGlobalWebSocketReturn {
  isConnected: boolean;
  isConnecting: boolean;
  connectionError: string | null;
  sendMessage: (message: OutgoingMessage) => void;
  disconnect: () => void;
  reconnect: () => void;
  lastMessage: WebSocketMessage | null;
}

export function useGlobalWebSocket({
  trialId,
  onMessage,
  onConnect,
  onDisconnect,
  onError,
}: UseGlobalWebSocketOptions): UseGlobalWebSocketReturn {
  const { data: session } = useSession();
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [lastMessage, setLastMessage] = useState<WebSocketMessage | null>(null);

  const onMessageRef = useRef(onMessage);
  const onConnectRef = useRef(onConnect);
  const onDisconnectRef = useRef(onDisconnect);
  const onErrorRef = useRef(onError);

  onMessageRef.current = onMessage;
  onConnectRef.current = onConnect;
  onDisconnectRef.current = onDisconnect;
  onErrorRef.current = onError;

  useEffect(() => {
    const unsubscribe = globalWS.subscribe(session, {
      trialId,
      onMessage: (msg) => {
        setLastMessage(msg);
        onMessageRef.current?.(msg);
      },
      onConnect: () => {
        setIsConnected(true);
        setIsConnecting(false);
        setConnectionError(null);
        onConnectRef.current?.();
      },
      onDisconnect: () => {
        setIsConnected(false);
        onDisconnectRef.current?.();
      },
      onError: (err) => {
        setConnectionError("Connection error");
        onErrorRef.current?.(err);
      },
    });

    return unsubscribe;
  }, [trialId, session]);

  const sendMessage = useCallback((message: OutgoingMessage) => {
    globalWS.sendMessage(message);
  }, []);

  const disconnect = useCallback(() => {
    globalWS.disconnect();
  }, []);

  const reconnect = useCallback(() => {
    globalWS.connect();
  }, []);

  return {
    isConnected,
    isConnecting,
    connectionError,
    sendMessage,
    disconnect,
    reconnect,
    lastMessage,
  };
}

// Legacy alias
export const useWebSocket = useGlobalWebSocket;

// Trial-specific hook
export interface TrialEvent {
  id: string;
  trialId: string;
  eventType: string;
  data: Record<string, unknown> | null;
  timestamp: Date;
  createdBy?: string | null;
}

export interface TrialWebSocketState {
  trialEvents: TrialEvent[];
  currentTrialStatus: TrialSnapshot | null;
  wizardActions: WebSocketMessage[];
}

export function useTrialWebSocket(
  trialId: string,
  options?: {
    onTrialEvent?: (event: TrialEvent) => void;
    onStatusChange?: (status: TrialSnapshot) => void;
    initialEvents?: TrialEvent[];
    initialStatus?: TrialSnapshot | null;
  },
) {
  const [state, setState] = useState<TrialWebSocketState>({
    trialEvents: options?.initialEvents ?? [],
    currentTrialStatus: options?.initialStatus ?? null,
    wizardActions: [],
  });

  const handleMessage = useCallback(
    (message: WebSocketMessage): void => {
      switch (message.type) {
        case "trial_status": {
          const data = (message as TrialStatusMessage).data;
          const status = data.trial as TrialSnapshot;
          setState((prev) => ({
            ...prev,
            currentTrialStatus: status,
          }));
          options?.onStatusChange?.(status);
          break;
        }

        case "trial_events_snapshot": {
          const data = (message as TrialEventsSnapshotMessage).data;
          const events = (
            data.events as Array<{
              id: string;
              trialId: string;
              eventType: string;
              data: Record<string, unknown> | null;
              timestamp: Date | string;
              createdBy?: string | null;
            }>
          ).map((e) => ({
            ...e,
            timestamp:
              typeof e.timestamp === "string"
                ? new Date(e.timestamp)
                : e.timestamp,
          }));
          setState((prev) => ({
            ...prev,
            trialEvents: events,
          }));
          break;
        }

        case "trial_event": {
          const data = (message as TrialEventMessage).data;
          const event = data.event as {
            id: string;
            trialId: string;
            eventType: string;
            data: Record<string, unknown> | null;
            timestamp: Date | string;
            createdBy?: string | null;
          };
          const newEvent: TrialEvent = {
            ...event,
            timestamp:
              typeof event.timestamp === "string"
                ? new Date(event.timestamp)
                : event.timestamp,
          };
          setState((prev) => ({
            ...prev,
            trialEvents: [...prev.trialEvents, newEvent].slice(-500),
          }));
          options?.onTrialEvent?.(newEvent);
          break;
        }

        case "trial_action_executed":
        case "intervention_logged":
        case "annotation_added":
        case "step_changed": {
          setState((prev) => ({
            ...prev,
            wizardActions: [...prev.wizardActions, message].slice(-100),
          }));
          break;
        }

        case "pong":
          break;

        default:
          if (process.env.NODE_ENV === "development") {
            console.log(`[WS] Unknown message type: ${message.type}`);
          }
      }
    },
    [options],
  );

  const webSocket = useGlobalWebSocket({
    trialId,
    onMessage: handleMessage,
    onConnect: () => {
      if (process.env.NODE_ENV === "development") {
        console.log(`[WS] Connected to trial ${trialId}`);
      }
    },
    onDisconnect: () => {
      if (process.env.NODE_ENV === "development") {
        console.log(`[WS] Disconnected from trial ${trialId}`);
      }
    },
    onError: () => {
      if (process.env.NODE_ENV !== "development") {
        console.error(`[WS] Trial ${trialId} WebSocket connection failed`);
      }
    },
  });

  // Request initial data after connection is established
  useEffect(() => {
    if (webSocket.isConnected) {
      webSocket.sendMessage({ type: "request_trial_status", data: {} });
      webSocket.sendMessage({
        type: "request_trial_events",
        data: { limit: 500 },
      });
    }
  }, [webSocket.isConnected]);

  // Helper to add an event locally (for optimistic updates)
  const addLocalEvent = useCallback((event: TrialEvent) => {
    setState((prev) => ({
      ...prev,
      trialEvents: [...prev.trialEvents, event].slice(-500),
    }));
  }, []);

  // Helper to update trial status locally
  const updateLocalStatus = useCallback((status: TrialSnapshot) => {
    setState((prev) => ({
      ...prev,
      currentTrialStatus: status,
    }));
  }, []);

  return {
    ...webSocket,
    trialEvents: state.trialEvents,
    currentTrialStatus: state.currentTrialStatus,
    wizardActions: state.wizardActions,
    addLocalEvent,
    updateLocalStatus,
  };
}
