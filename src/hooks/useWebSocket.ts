"use client";

/* eslint-disable react-hooks/exhaustive-deps */

import { useSession } from "next-auth/react";
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
    timestamp: number;
  } & Record<string, unknown>;
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

export interface UseWebSocketOptions {
  trialId: string;
  onMessage?: (message: WebSocketMessage) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onError?: (error: Event) => void;
  reconnectAttempts?: number;
  reconnectInterval?: number;
  heartbeatInterval?: number;
}

export interface UseWebSocketReturn {
  isConnected: boolean;
  isConnecting: boolean;
  connectionError: string | null;
  sendMessage: (message: OutgoingMessage) => void;
  disconnect: () => void;
  reconnect: () => void;
  lastMessage: WebSocketMessage | null;
}

export function useWebSocket({
  trialId,
  onMessage,
  onConnect,
  onDisconnect,
  onError,
  reconnectAttempts = 5,
  reconnectInterval = 3000,
  heartbeatInterval = 30000,
}: UseWebSocketOptions): UseWebSocketReturn {
  const { data: session } = useSession();
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [isConnecting, setIsConnecting] = useState<boolean>(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [hasAttemptedConnection, setHasAttemptedConnection] =
    useState<boolean>(false);
  const [lastMessage, setLastMessage] = useState<WebSocketMessage | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const heartbeatTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const attemptCountRef = useRef<number>(0);
  const mountedRef = useRef<boolean>(true);
  const connectionStableTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Generate auth token (simplified - in production use proper JWT)
  const getAuthToken = useCallback((): string | null => {
    if (!session?.user) return null;
    // In production, this would be a proper JWT token
    return btoa(
      JSON.stringify({ userId: session.user.id, timestamp: Date.now() }),
    );
  }, [session]);

  const sendMessage = useCallback((message: OutgoingMessage): void => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      console.warn("WebSocket not connected, message not sent:", message);
    }
  }, []);

  const sendHeartbeat = useCallback((): void => {
    sendMessage({ type: "heartbeat", data: {} });
  }, [sendMessage]);

  const scheduleHeartbeat = useCallback((): void => {
    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
    }
    heartbeatTimeoutRef.current = setTimeout(() => {
      if (isConnected && mountedRef.current) {
        sendHeartbeat();
        scheduleHeartbeat();
      }
    }, heartbeatInterval);
  }, [isConnected, sendHeartbeat, heartbeatInterval]);

  const handleMessage = useCallback(
    (event: MessageEvent<string>): void => {
      try {
        const message = JSON.parse(event.data) as WebSocketMessage;
        setLastMessage(message);

        // Handle system messages
        switch (message.type) {
          case "connection_established": {
            console.log(
              "WebSocket connection established:",
              (message as ConnectionEstablishedMessage).data,
            );
            setIsConnected(true);
            setIsConnecting(false);
            setConnectionError(null);
            attemptCountRef.current = 0;
            scheduleHeartbeat();
            onConnect?.();
            break;
          }

          case "heartbeat_response":
            // Heartbeat acknowledged, connection is alive
            break;

          case "error": {
            console.error("WebSocket server error:", message);
            const msg =
              (message as ErrorMessage).data?.message ?? "Server error";
            setConnectionError(msg);
            onError?.(new Event("server_error"));
            break;
          }

          default:
            // Pass to user-defined message handler
            onMessage?.(message);
            break;
        }
      } catch (error) {
        console.error("Error parsing WebSocket message:", error);
        setConnectionError("Failed to parse message");
      }
    },
    [onMessage, onConnect, onError, scheduleHeartbeat],
  );

  const handleClose = useCallback(
    (event: CloseEvent): void => {
      console.log("WebSocket connection closed:", event.code, event.reason);
      setIsConnected(false);
      setIsConnecting(false);

      if (heartbeatTimeoutRef.current) {
        clearTimeout(heartbeatTimeoutRef.current);
      }

      onDisconnect?.();

      // Attempt reconnection if not manually closed and component is still mounted
      // In development, don't aggressively reconnect to prevent UI flashing
      if (
        event.code !== 1000 &&
        mountedRef.current &&
        attemptCountRef.current < reconnectAttempts &&
        process.env.NODE_ENV !== "development"
      ) {
        attemptCountRef.current++;
        const delay =
          reconnectInterval * Math.pow(1.5, attemptCountRef.current - 1); // Exponential backoff

        console.log(
          `Attempting reconnection ${attemptCountRef.current}/${reconnectAttempts} in ${delay}ms`,
        );
        setConnectionError(
          `Connection lost. Reconnecting... (${attemptCountRef.current}/${reconnectAttempts})`,
        );

        reconnectTimeoutRef.current = setTimeout(() => {
          if (mountedRef.current) {
            attemptCountRef.current = 0;
            setIsConnecting(true);
            setConnectionError(null);
          }
        }, delay);
      } else if (attemptCountRef.current >= reconnectAttempts) {
        setConnectionError("Failed to reconnect after maximum attempts");
      } else if (
        process.env.NODE_ENV === "development" &&
        event.code !== 1000
      ) {
        // In development, set a stable error message without reconnection attempts
        setConnectionError("WebSocket unavailable - using polling mode");
      }
    },
    [onDisconnect, reconnectAttempts, reconnectInterval],
  );

  const handleError = useCallback(
    (event: Event): void => {
      // In development, WebSocket failures are expected with Edge Runtime
      if (process.env.NODE_ENV === "development") {
        // Only set error state after the first failed attempt to prevent flashing
        if (!hasAttemptedConnection) {
          setHasAttemptedConnection(true);
          // Debounce the error state to prevent UI flashing
          if (connectionStableTimeoutRef.current) {
            clearTimeout(connectionStableTimeoutRef.current);
          }
          connectionStableTimeoutRef.current = setTimeout(() => {
            setConnectionError("WebSocket unavailable - using polling mode");
            setIsConnecting(false);
          }, 1000);
        }
      } else {
        console.error("WebSocket error:", event);
        setConnectionError("Connection error");
        setIsConnecting(false);
      }
      onError?.(event);
    },
    [onError, hasAttemptedConnection],
  );

  const connectInternal = useCallback((): void => {
    if (!session?.user || !trialId) {
      if (!hasAttemptedConnection) {
        setConnectionError("Missing authentication or trial ID");
        setHasAttemptedConnection(true);
      }
      return;
    }

    if (
      wsRef.current &&
      (wsRef.current.readyState === WebSocket.CONNECTING ||
        wsRef.current.readyState === WebSocket.OPEN)
    ) {
      return; // Already connecting or connected
    }

    const token = getAuthToken();
    if (!token) {
      if (!hasAttemptedConnection) {
        setConnectionError("Failed to generate auth token");
        setHasAttemptedConnection(true);
      }
      return;
    }

    // Only show connecting state for the first attempt or if we've been stable
    if (!hasAttemptedConnection || isConnected) {
      setIsConnecting(true);
    }

    // Clear any pending error updates
    if (connectionStableTimeoutRef.current) {
      clearTimeout(connectionStableTimeoutRef.current);
    }

    setConnectionError(null);

    try {
      // Use appropriate WebSocket URL based on environment
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      const wsUrl = `${protocol}//${window.location.host}/api/websocket?trialId=${trialId}&token=${token}`;

      wsRef.current = new WebSocket(wsUrl);
      wsRef.current.onmessage = handleMessage;
      wsRef.current.onclose = handleClose;
      wsRef.current.onerror = handleError;

      wsRef.current.onopen = () => {
        console.log("WebSocket connection opened");
        // Connection establishment is handled in handleMessage
      };
    } catch (error) {
      console.error("Failed to create WebSocket connection:", error);
      if (!hasAttemptedConnection) {
        setConnectionError("Failed to create connection");
        setHasAttemptedConnection(true);
      }
      setIsConnecting(false);
    }
  }, [
    session,
    trialId,
    getAuthToken,
    handleMessage,
    handleClose,
    handleError,
    hasAttemptedConnection,
    isConnected,
  ]);

  const disconnect = useCallback((): void => {
    mountedRef.current = false;

    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
    }

    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
    }

    if (connectionStableTimeoutRef.current) {
      clearTimeout(connectionStableTimeoutRef.current);
    }

    if (wsRef.current) {
      wsRef.current.close(1000, "Manual disconnect");
      wsRef.current = null;
    }

    setIsConnected(false);
    setIsConnecting(false);
    setConnectionError(null);
    setHasAttemptedConnection(false);
    attemptCountRef.current = 0;
  }, []);

  const reconnect = useCallback((): void => {
    disconnect();
    mountedRef.current = true;
    attemptCountRef.current = 0;
    setHasAttemptedConnection(false);
    setTimeout(() => {
      if (mountedRef.current) {
        void connectInternal();
      }
    }, 100); // Small delay to ensure cleanup
  }, [disconnect, connectInternal]);

  // Effect to establish initial connection
  useEffect(() => {
    if (session?.user?.id && trialId) {
      // In development, only attempt connection once to prevent flashing
      if (process.env.NODE_ENV === "development" && hasAttemptedConnection) {
        return;
      }

      // Trigger reconnection if timeout was set
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
        void connectInternal();
      } else {
        void connectInternal();
      }
    }

    return () => {
      mountedRef.current = false;
      disconnect();
    };
  }, [session?.user?.id, trialId, hasAttemptedConnection]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
      if (connectionStableTimeoutRef.current) {
        clearTimeout(connectionStableTimeoutRef.current);
      }
      disconnect();
    };
  }, [disconnect]);

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

// Hook for trial-specific WebSocket events
export function useTrialWebSocket(trialId: string) {
  const [trialEvents, setTrialEvents] = useState<WebSocketMessage[]>([]);
  const [currentTrialStatus, setCurrentTrialStatus] =
    useState<TrialSnapshot | null>(null);
  const [wizardActions, setWizardActions] = useState<WebSocketMessage[]>([]);

  const handleMessage = useCallback((message: WebSocketMessage): void => {
    // Add to events log
    setTrialEvents((prev) => [...prev, message].slice(-100)); // Keep last 100 events

    switch (message.type) {
      case "trial_status": {
        const data = (message as TrialStatusMessage).data;
        setCurrentTrialStatus(data.trial);
        break;
      }

      case "trial_action_executed":
      case "intervention_logged":
      case "step_changed":
        setWizardActions((prev) => [...prev, message].slice(-50)); // Keep last 50 actions
        break;

      case "step_changed":
        // Handle step transitions (optional logging)
        console.log("Step changed:", (message as StepChangedMessage).data);
        break;

      default:
        // Handle other trial-specific messages
        break;
    }
  }, []);

  const webSocket = useWebSocket({
    trialId,
    onMessage: handleMessage,
    onConnect: () => {
      if (process.env.NODE_ENV === "development") {
        console.log(`Connected to trial ${trialId} WebSocket`);
      }
    },
    onDisconnect: () => {
      if (process.env.NODE_ENV === "development") {
        console.log(`Disconnected from trial ${trialId} WebSocket`);
      }
    },
    onError: () => {
      // Suppress noisy WebSocket errors in development
      if (process.env.NODE_ENV !== "development") {
        console.error(`Trial ${trialId} WebSocket connection failed`);
      }
    },
  });

  // Request trial status after connection is established
  useEffect(() => {
    if (webSocket.isConnected) {
      webSocket.sendMessage({ type: "request_trial_status", data: {} });
    }
  }, [webSocket.isConnected, webSocket]);

  // Trial-specific actions
  const executeTrialAction = useCallback(
    (actionType: string, actionData: Record<string, unknown>): void => {
      webSocket.sendMessage({
        type: "trial_action",
        data: {
          actionType,
          ...actionData,
        },
      });
    },
    [webSocket],
  );

  const logWizardIntervention = useCallback(
    (interventionData: Record<string, unknown>): void => {
      webSocket.sendMessage({
        type: "wizard_intervention",
        data: interventionData,
      });
    },
    [webSocket],
  );

  const transitionStep = useCallback(
    (stepData: {
      from_step?: number;
      to_step: number;
      step_name?: string;
      [k: string]: unknown;
    }): void => {
      webSocket.sendMessage({
        type: "step_transition",
        data: stepData,
      });
    },
    [webSocket],
  );

  return {
    ...webSocket,
    trialEvents,
    currentTrialStatus,
    wizardActions,
    executeTrialAction,
    logWizardIntervention,
    transitionStep,
  };
}
