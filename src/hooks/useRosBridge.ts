"use client";

/* eslint-disable @typescript-eslint/no-floating-promises */
/* eslint-disable react-hooks/exhaustive-deps */

import { useEffect, useState, useCallback, useRef } from "react";
import {
  getRosBridge,
  type RosBridge,
  type RobotStatus,
  type RobotAction,
} from "~/lib/ros-bridge";

export interface UseRosBridgeOptions {
  /** ROS bridge WebSocket URL */
  url?: string;
  /** Auto-connect on mount */
  autoConnect?: boolean;
  /** Reconnect attempts */
  maxReconnectAttempts?: number;
  /** Topics to subscribe to */
  subscriptions?: Array<{ topic: string; messageType: string }>;
}

export interface UseRosBridgeReturn {
  /** ROS bridge instance */
  bridge: RosBridge | null;
  /** Connection status */
  isConnected: boolean;
  /** Connection loading state */
  isConnecting: boolean;
  /** Connection error */
  error: string | null;
  /** Current robot status */
  robotStatus: RobotStatus | null;
  /** Active robot actions */
  activeActions: RobotAction[];
  /** Last received topic message */
  lastMessage: { topic: string; message: Record<string, unknown> } | null;

  // Actions
  /** Connect to ROS bridge */
  connect: () => Promise<void>;
  /** Disconnect from ROS bridge */
  disconnect: () => void;
  /** Execute robot action */
  executeAction: (
    actionType: string,
    parameters: Record<string, unknown>,
  ) => Promise<RobotAction>;
  /** Publish message to topic */
  publish: (
    topic: string,
    messageType: string,
    message: Record<string, unknown>,
  ) => void;
  /** Call ROS service */
  callService: (
    service: string,
    serviceType: string,
    args?: Record<string, unknown>,
  ) => Promise<Record<string, unknown>>;
  /** Subscribe to topic */
  subscribe: (topic: string, messageType: string) => string;
  /** Unsubscribe from topic */
  unsubscribe: (topic: string) => void;
}

export function useRosBridge(
  options: UseRosBridgeOptions = {},
): UseRosBridgeReturn {
  const {
    url = "ws://localhost:9090",
    autoConnect = false,
    maxReconnectAttempts = 5,
    subscriptions = [],
  } = options;

  const [bridge, setBridge] = useState<RosBridge | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [robotStatus, setRobotStatus] = useState<RobotStatus | null>(null);
  const [activeActions, setActiveActions] = useState<RobotAction[]>([]);
  const [lastMessage, setLastMessage] = useState<{
    topic: string;
    message: Record<string, unknown>;
  } | null>(null);

  const reconnectAttempts = useRef(0);
  const subscriptionIds = useRef<Set<string>>(new Set());

  // Initialize bridge
  useEffect(() => {
    const rosBridge = getRosBridge(url);
    setBridge(rosBridge);

    // Set up event listeners
    const handleConnected = () => {
      setIsConnected(true);
      setIsConnecting(false);
      setError(null);
      reconnectAttempts.current = 0;

      // Set up initial subscriptions
      subscriptions.forEach(({ topic, messageType }) => {
        const id = rosBridge.subscribe(topic, messageType);
        subscriptionIds.current.add(id);
      });
    };

    const handleDisconnected = () => {
      setIsConnected(false);
      setIsConnecting(false);
      subscriptionIds.current.clear();

      // Attempt reconnect if within limits
      if (reconnectAttempts.current < maxReconnectAttempts) {
        reconnectAttempts.current++;
        setError(
          `Connection lost. Reconnecting... (${reconnectAttempts.current}/${maxReconnectAttempts})`,
        );

        setTimeout(() => {
          if (reconnectAttempts.current <= maxReconnectAttempts) {
            connect();
          }
        }, 3000 * reconnectAttempts.current); // Exponential backoff
      } else {
        setError("Connection failed after maximum attempts");
      }
    };

    const handleError = (err: Error) => {
      console.error("[useRosBridge] Error:", err);
      setError(err.message);
      setIsConnecting(false);
    };

    const handleStatusUpdate = (status: RobotStatus) => {
      setRobotStatus(status);
    };

    const handleTopicMessage = (
      topic: string,
      message: Record<string, unknown>,
    ) => {
      setLastMessage({ topic, message });
    };

    const handleActionStarted = (action: RobotAction) => {
      setActiveActions((prev) => {
        const filtered = prev.filter((a) => a.id !== action.id);
        return [...filtered, action];
      });
    };

    const handleActionCompleted = (action: RobotAction) => {
      setActiveActions((prev) =>
        prev.map((a) => (a.id === action.id ? action : a)),
      );
    };

    const handleActionFailed = (action: RobotAction) => {
      setActiveActions((prev) =>
        prev.map((a) => (a.id === action.id ? action : a)),
      );
    };

    rosBridge.on("connected", handleConnected);
    rosBridge.on("disconnected", handleDisconnected);
    rosBridge.on("error", handleError);
    rosBridge.on("status_update", handleStatusUpdate);
    rosBridge.on("topic_message", handleTopicMessage);
    rosBridge.on("action_started", handleActionStarted);
    rosBridge.on("action_completed", handleActionCompleted);
    rosBridge.on("action_failed", handleActionFailed);

    // Auto-connect if requested
    if (autoConnect && !rosBridge.isConnected()) {
      connect();
    }

    return () => {
      rosBridge.off("connected", handleConnected);
      rosBridge.off("disconnected", handleDisconnected);
      rosBridge.off("error", handleError);
      rosBridge.off("status_update", handleStatusUpdate);
      rosBridge.off("topic_message", handleTopicMessage);
      rosBridge.off("action_started", handleActionStarted);
      rosBridge.off("action_completed", handleActionCompleted);
      rosBridge.off("action_failed", handleActionFailed);
    };
  }, [url, autoConnect, maxReconnectAttempts]);

  const connect = useCallback(async () => {
    if (!bridge || isConnecting || isConnected) return;

    setIsConnecting(true);
    setError(null);

    try {
      await bridge.connect();
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err.message : "Connection failed";
      setError(errorMessage);
      setIsConnecting(false);
    }
  }, [bridge, isConnecting, isConnected]);

  const disconnect = useCallback(() => {
    if (!bridge) return;

    bridge.disconnect();
    subscriptionIds.current.clear();
    reconnectAttempts.current = maxReconnectAttempts; // Prevent auto-reconnect
  }, [bridge, maxReconnectAttempts]);

  const executeAction = useCallback(
    async (
      actionType: string,
      parameters: Record<string, unknown>,
    ): Promise<RobotAction> => {
      if (!bridge || !isConnected) {
        throw new Error("ROS bridge not connected");
      }

      return bridge.executeAction(actionType, parameters);
    },
    [bridge, isConnected],
  );

  const publish = useCallback(
    (topic: string, messageType: string, message: Record<string, unknown>) => {
      if (!bridge || !isConnected) {
        console.warn("[useRosBridge] Cannot publish - not connected");
        return;
      }

      bridge.publish(topic, messageType, message);
    },
    [bridge, isConnected],
  );

  const callService = useCallback(
    async (
      service: string,
      serviceType: string,
      args: Record<string, unknown> = {},
    ): Promise<Record<string, unknown>> => {
      if (!bridge || !isConnected) {
        throw new Error("ROS bridge not connected");
      }

      return bridge.callService(service, serviceType, args);
    },
    [bridge, isConnected],
  );

  const subscribe = useCallback(
    (topic: string, messageType: string): string => {
      if (!bridge) {
        throw new Error("ROS bridge not initialized");
      }

      const id = bridge.subscribe(topic, messageType);
      subscriptionIds.current.add(id);
      return id;
    },
    [bridge],
  );

  const unsubscribe = useCallback(
    (topic: string) => {
      if (!bridge) return;

      bridge.unsubscribe(topic);
      // Remove from our tracking (note: we track by topic, not ID)
      subscriptionIds.current.forEach((id) => {
        if (id.includes(topic)) {
          subscriptionIds.current.delete(id);
        }
      });
    },
    [bridge],
  );

  return {
    bridge,
    isConnected,
    isConnecting,
    error,
    robotStatus,
    activeActions,
    lastMessage,
    connect,
    disconnect,
    executeAction,
    publish,
    callService,
    subscribe,
    unsubscribe,
  };
}

export default useRosBridge;
