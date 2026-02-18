"use client";

import { useEffect, useState, useCallback, useRef } from "react";
import {
  WizardRosService,
  type RobotStatus,
  type RobotActionExecution,
  getWizardRosService,
} from "~/lib/ros/wizard-ros-service";

export interface UseWizardRosOptions {
  autoConnect?: boolean;
  onConnected?: () => void;
  onDisconnected?: () => void;
  onError?: (error: unknown) => void;
  onActionCompleted?: (execution: RobotActionExecution) => void;
  onActionFailed?: (execution: RobotActionExecution) => void;
}

export interface UseWizardRosReturn {
  isConnected: boolean;
  isConnecting: boolean;
  connectionError: string | null;
  robotStatus: RobotStatus;
  activeActions: RobotActionExecution[];
  connect: () => Promise<void>;
  disconnect: () => void;
  executeRobotAction: (
    pluginName: string,
    actionId: string,
    parameters: Record<string, unknown>,
    actionConfig?: {
      topic: string;
      messageType: string;
      payloadMapping: {
        type: string;
        payload?: Record<string, unknown>;
        transformFn?: string;
      };
    },
  ) => Promise<RobotActionExecution>;
  callService: (service: string, args?: Record<string, unknown>) => Promise<any>;
  setAutonomousLife: (enabled: boolean) => Promise<boolean>;
}


export function useWizardRos(
  options: UseWizardRosOptions = {},
): UseWizardRosReturn {
  const {
    autoConnect = true,
    onConnected,
    onDisconnected,
    onError,
    onActionCompleted,
    onActionFailed,
  } = options;

  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({
    connected: false,
    battery: 0,
    position: { x: 0, y: 0, theta: 0 },
    joints: {},
    sensors: {},
    lastUpdate: new Date(),
  });
  const [activeActions, setActiveActions] = useState<RobotActionExecution[]>(
    [],
  );

  // Prevent multiple connections
  const isInitializedRef = useRef(false);
  const connectAttemptRef = useRef(false);

  const serviceRef = useRef<WizardRosService | null>(null);
  const mountedRef = useRef(true);

  // Use refs for callbacks to prevent infinite re-renders
  const onConnectedRef = useRef(onConnected);
  const onDisconnectedRef = useRef(onDisconnected);
  const onErrorRef = useRef(onError);
  const onActionCompletedRef = useRef(onActionCompleted);
  const onActionFailedRef = useRef(onActionFailed);

  // Update refs when callbacks change
  onConnectedRef.current = onConnected;
  onDisconnectedRef.current = onDisconnected;
  onErrorRef.current = onError;
  onActionCompletedRef.current = onActionCompleted;
  onActionFailedRef.current = onActionFailed;

  // Initialize service (only once)
  useEffect(() => {
    if (!isInitializedRef.current) {
      serviceRef.current = getWizardRosService();
      isInitializedRef.current = true;
    }

    return () => {
      mountedRef.current = false;
    };
  }, []);

  // Set up event listeners with stable callbacks
  useEffect(() => {
    const service = serviceRef.current;
    if (!service) return;

    const handleConnected = () => {
      console.log("[useWizardRos] handleConnected called, mountedRef:", mountedRef.current);
      // Set state immediately, before checking mounted status
      setIsConnected(true);
      setIsConnecting(false);
      setConnectionError(null);

      if (mountedRef.current) {
        onConnectedRef.current?.();
      }
    };

    const handleDisconnected = () => {
      if (!mountedRef.current) return;
      console.log("[useWizardRos] Disconnected from ROS bridge");
      setIsConnected(false);
      setIsConnecting(false);
      onDisconnectedRef.current?.();
    };

    const handleError = (error: unknown) => {
      if (!mountedRef.current) return;
      console.error("[useWizardRos] ROS connection error:", error);
      setConnectionError(
        error instanceof Error ? error.message : "Connection error",
      );
      setIsConnecting(false);
      onErrorRef.current?.(error);
    };

    const handleRobotStatusUpdated = (status: RobotStatus) => {
      if (!mountedRef.current) return;
      setRobotStatus(status);
    };

    const handleActionStarted = (execution: RobotActionExecution) => {
      if (!mountedRef.current) return;
      setActiveActions((prev) => {
        const filtered = prev.filter((action) => action.id !== execution.id);
        return [...filtered, execution];
      });
    };

    const handleActionCompleted = (execution: RobotActionExecution) => {
      if (!mountedRef.current) return;
      setActiveActions((prev) =>
        prev.map((action) => (action.id === execution.id ? execution : action)),
      );
      onActionCompletedRef.current?.(execution);
    };

    const handleActionFailed = (execution: RobotActionExecution) => {
      if (!mountedRef.current) return;
      setActiveActions((prev) =>
        prev.map((action) => (action.id === execution.id ? execution : action)),
      );
      onActionFailedRef.current?.(execution);
    };

    const handleMaxReconnectsReached = () => {
      if (!mountedRef.current) return;
      setConnectionError("Maximum reconnection attempts reached");
      setIsConnecting(false);
    };

    // Add event listeners
    service.on("connected", handleConnected);
    service.on("disconnected", handleDisconnected);
    service.on("error", handleError);
    service.on("robot_status_updated", handleRobotStatusUpdated);
    service.on("action_started", handleActionStarted);
    service.on("action_completed", handleActionCompleted);
    service.on("action_failed", handleActionFailed);
    service.on("max_reconnects_reached", handleMaxReconnectsReached);

    // Initialize connection status
    setIsConnected(service.getConnectionStatus());
    setRobotStatus(service.getRobotStatus());
    setActiveActions(service.getActiveActions());

    return () => {
      service.off("connected", handleConnected);
      service.off("disconnected", handleDisconnected);
      service.off("error", handleError);
      service.off("robot_status_updated", handleRobotStatusUpdated);
      service.off("action_started", handleActionStarted);
      service.off("action_completed", handleActionCompleted);
      service.off("action_failed", handleActionFailed);
      service.off("max_reconnects_reached", handleMaxReconnectsReached);
    };
  }, []); // Empty deps since we use refs

  const connect = useCallback(async (): Promise<void> => {
    const service = serviceRef.current;
    if (!service || isConnected || isConnecting || connectAttemptRef.current)
      return;

    connectAttemptRef.current = true;
    setIsConnecting(true);
    setConnectionError(null);

    try {
      await service.connect();
      // Connection successful - state will be updated by event handler
    } catch (error) {
      if (mountedRef.current) {
        setIsConnecting(false);
        setConnectionError(
          error instanceof Error ? error.message : "Connection failed",
        );
      }
      throw error;
    } finally {
      connectAttemptRef.current = false;
    }
  }, [isConnected, isConnecting]);

  // Auto-connect if enabled (only once per hook instance)
  useEffect(() => {
    if (
      autoConnect &&
      serviceRef.current &&
      !isConnected &&
      !isConnecting &&
      !connectAttemptRef.current
    ) {
      const timeoutId = setTimeout(() => {
        connect().catch((error) => {
          console.warn("[useWizardRos] Auto-connect failed (retrying manually):", error instanceof Error ? error.message : error);
          // Don't retry automatically - let user manually connect
        });
      }, 100); // Small delay to prevent immediate connection attempts

      return () => clearTimeout(timeoutId);
    }
  }, [autoConnect, isConnected, isConnecting, connect]);

  const disconnect = useCallback((): void => {
    const service = serviceRef.current;
    if (!service) return;

    connectAttemptRef.current = false;
    service.disconnect();
    setIsConnected(false);
    setIsConnecting(false);
    setConnectionError(null);
  }, []);

  const executeRobotAction = useCallback(
    async (
      pluginName: string,
      actionId: string,
      parameters: Record<string, unknown>,
      actionConfig?: {
        topic: string;
        messageType: string;
        payloadMapping: {
          type: string;
          payload?: Record<string, unknown>;
          transformFn?: string;
        };
      },
    ): Promise<RobotActionExecution> => {
      const service = serviceRef.current;
      if (!service) {
        throw new Error("ROS service not initialized");
      }

      if (!isConnected) {
        throw new Error("Not connected to ROS bridge");
      }

      return service.executeRobotAction(
        pluginName,
        actionId,
        parameters,
        actionConfig,
      );
    },
    [isConnected],
  );

  const callService = useCallback(
    async (service: string, args?: Record<string, unknown>): Promise<any> => {
      const srv = serviceRef.current;
      if (!srv || !isConnected) throw new Error("Not connected");
      return srv.callService(service, args);
    },
    [isConnected],
  );

  const setAutonomousLife = useCallback(
    async (enabled: boolean): Promise<boolean> => {
      const srv = serviceRef.current;
      if (!srv || !isConnected) throw new Error("Not connected");
      return srv.setAutonomousLife(enabled);
    },
    [isConnected],
  );

  return {
    isConnected,
    isConnecting,
    connectionError,
    robotStatus,
    activeActions,
    connect,
    disconnect,
    executeRobotAction,
    callService,
    setAutonomousLife,
  };
}
