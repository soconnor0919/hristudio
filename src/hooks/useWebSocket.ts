"use client";

import { useSession } from "next-auth/react";
import { useCallback, useEffect, useRef, useState } from "react";

export interface WebSocketMessage {
  type: string;
  data: any;
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
  sendMessage: (message: WebSocketMessage) => void;
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
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [lastMessage, setLastMessage] = useState<WebSocketMessage | null>(null);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const heartbeatTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const attemptCountRef = useRef(0);
  const mountedRef = useRef(true);

  // Generate auth token (simplified - in production use proper JWT)
  const getAuthToken = useCallback(() => {
    if (!session?.user) return null;
    // In production, this would be a proper JWT token
    return btoa(JSON.stringify({ userId: session.user.id, timestamp: Date.now() }));
  }, [session]);

  const sendMessage = useCallback((message: WebSocketMessage) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      console.warn("WebSocket not connected, message not sent:", message);
    }
  }, []);

  const sendHeartbeat = useCallback(() => {
    sendMessage({ type: "heartbeat", data: {} });
  }, [sendMessage]);

  const scheduleHeartbeat = useCallback(() => {
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

  const handleMessage = useCallback((event: MessageEvent) => {
    try {
      const message: WebSocketMessage = JSON.parse(event.data);
      setLastMessage(message);

      // Handle system messages
      switch (message.type) {
        case "connection_established":
          console.log("WebSocket connection established:", message.data);
          setIsConnected(true);
          setIsConnecting(false);
          setConnectionError(null);
          attemptCountRef.current = 0;
          scheduleHeartbeat();
          onConnect?.();
          break;

        case "heartbeat_response":
          // Heartbeat acknowledged, connection is alive
          break;

        case "error":
          console.error("WebSocket server error:", message.data);
          setConnectionError(message.data.message || "Server error");
          onError?.(new Event("server_error"));
          break;

        default:
          // Pass to user-defined message handler
          onMessage?.(message);
          break;
      }
    } catch (error) {
      console.error("Error parsing WebSocket message:", error);
      setConnectionError("Failed to parse message");
    }
  }, [onMessage, onConnect, onError, scheduleHeartbeat]);

  const handleClose = useCallback((event: CloseEvent) => {
    console.log("WebSocket connection closed:", event.code, event.reason);
    setIsConnected(false);
    setIsConnecting(false);

    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
    }

    onDisconnect?.();

    // Attempt reconnection if not manually closed and component is still mounted
    if (event.code !== 1000 && mountedRef.current && attemptCountRef.current < reconnectAttempts) {
      attemptCountRef.current++;
      const delay = reconnectInterval * Math.pow(1.5, attemptCountRef.current - 1); // Exponential backoff

      console.log(`Attempting reconnection ${attemptCountRef.current}/${reconnectAttempts} in ${delay}ms`);
      setConnectionError(`Connection lost. Reconnecting... (${attemptCountRef.current}/${reconnectAttempts})`);

      reconnectTimeoutRef.current = setTimeout(() => {
        if (mountedRef.current) {
          connect();
        }
      }, delay);
    } else if (attemptCountRef.current >= reconnectAttempts) {
      setConnectionError("Failed to reconnect after maximum attempts");
    }
  }, [onDisconnect, reconnectAttempts, reconnectInterval]);

  const handleError = useCallback((event: Event) => {
    console.error("WebSocket error:", event);
    setConnectionError("Connection error");
    setIsConnecting(false);
    onError?.(event);
  }, [onError]);

  const connect = useCallback(() => {
    if (!session?.user || !trialId) {
      setConnectionError("Missing authentication or trial ID");
      return;
    }

    if (wsRef.current &&
        (wsRef.current.readyState === WebSocket.CONNECTING ||
         wsRef.current.readyState === WebSocket.OPEN)) {
      return; // Already connecting or connected
    }

    const token = getAuthToken();
    if (!token) {
      setConnectionError("Failed to generate auth token");
      return;
    }

    setIsConnecting(true);
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
      setConnectionError("Failed to create connection");
      setIsConnecting(false);
    }
  }, [session, trialId, getAuthToken, handleMessage, handleClose, handleError]);

  const disconnect = useCallback(() => {
    mountedRef.current = false;

    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
    }

    if (heartbeatTimeoutRef.current) {
      clearTimeout(heartbeatTimeoutRef.current);
    }

    if (wsRef.current) {
      wsRef.current.close(1000, "Manual disconnect");
      wsRef.current = null;
    }

    setIsConnected(false);
    setIsConnecting(false);
    setConnectionError(null);
    attemptCountRef.current = 0;
  }, []);

  const reconnect = useCallback(() => {
    disconnect();
    mountedRef.current = true;
    attemptCountRef.current = 0;
    setTimeout(connect, 100); // Small delay to ensure cleanup
  }, [disconnect, connect]);

  // Effect to establish initial connection
  useEffect(() => {
    if (session?.user && trialId) {
      connect();
    }

    return () => {
      mountedRef.current = false;
      disconnect();
    };
  }, [session?.user?.id, trialId]); // Reconnect if user or trial changes

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mountedRef.current = false;
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
  const [currentTrialStatus, setCurrentTrialStatus] = useState<any>(null);
  const [wizardActions, setWizardActions] = useState<any[]>([]);

  const handleMessage = useCallback((message: WebSocketMessage) => {
    // Add to events log
    setTrialEvents(prev => [...prev, message].slice(-100)); // Keep last 100 events

    switch (message.type) {
      case "trial_status":
        setCurrentTrialStatus(message.data.trial);
        break;

      case "trial_action_executed":
      case "intervention_logged":
      case "step_changed":
        setWizardActions(prev => [...prev, message].slice(-50)); // Keep last 50 actions
        break;

      case "step_changed":
        // Handle step transitions
        console.log("Step changed:", message.data);
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
      console.log(`Connected to trial ${trialId} WebSocket`);
      // Request current trial status on connect
      webSocket.sendMessage({ type: "request_trial_status", data: {} });
    },
    onDisconnect: () => {
      console.log(`Disconnected from trial ${trialId} WebSocket`);
    },
    onError: (error) => {
      console.error(`Trial ${trialId} WebSocket error:`, error);
    },
  });

  // Trial-specific actions
  const executeTrialAction = useCallback((actionType: string, actionData: any) => {
    webSocket.sendMessage({
      type: "trial_action",
      data: {
        actionType,
        ...actionData,
      },
    });
  }, [webSocket]);

  const logWizardIntervention = useCallback((interventionData: any) => {
    webSocket.sendMessage({
      type: "wizard_intervention",
      data: interventionData,
    });
  }, [webSocket]);

  const transitionStep = useCallback((stepData: any) => {
    webSocket.sendMessage({
      type: "step_transition",
      data: stepData,
    });
  }, [webSocket]);

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
