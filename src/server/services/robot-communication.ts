/* eslint-disable @typescript-eslint/no-explicit-any */
/* eslint-disable @typescript-eslint/no-unsafe-assignment */
/* eslint-disable @typescript-eslint/no-unsafe-member-access */
/* eslint-disable @typescript-eslint/no-unsafe-call */
/* eslint-disable @typescript-eslint/no-unsafe-return */

import WebSocket from "ws";
import { EventEmitter } from "events";

export interface RobotCommunicationConfig {
  rosBridgeUrl: string;
  connectionTimeout: number;
  reconnectInterval: number;
  maxReconnectAttempts: number;
}

export interface RobotAction {
  pluginName: string;
  actionId: string;
  parameters: Record<string, unknown>;
  implementation: {
    topic: string;
    messageType: string;
    messageTemplate: Record<string, unknown>;
  };
}

export interface RobotActionResult {
  success: boolean;
  duration: number;
  data?: Record<string, unknown>;
  error?: string;
}

/**
 * Server-side robot communication service for ROS integration
 *
 * This service manages WebSocket connections to rosbridge_server and provides
 * a high-level interface for executing robot actions during trial execution.
 */
export class RobotCommunicationService extends EventEmitter {
  private ws: WebSocket | null = null;
  private config: RobotCommunicationConfig;
  private messageId = 0;
  private pendingActions = new Map<
    string,
    {
      resolve: (result: RobotActionResult) => void;
      reject: (error: Error) => void;
      timeout: NodeJS.Timeout;
      startTime: number;
    }
  >();
  private reconnectAttempts = 0;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private isConnected = false;

  constructor(config: Partial<RobotCommunicationConfig> = {}) {
    super();

    this.config = {
      rosBridgeUrl: process.env.ROS_BRIDGE_URL || "ws://localhost:9090",
      connectionTimeout: 10000,
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      ...config,
    };
  }

  /**
   * Initialize connection to ROS bridge
   */
  async connect(): Promise<void> {
    if (this.isConnected) {
      return;
    }

    return new Promise((resolve, reject) => {
      console.log(
        `[RobotComm] Connecting to ROS bridge: ${this.config.rosBridgeUrl}`,
      );

      try {
        this.ws = new WebSocket(this.config.rosBridgeUrl);

        const connectionTimeout = setTimeout(() => {
          reject(new Error("Connection timeout"));
          this.cleanup();
        }, this.config.connectionTimeout);

        this.ws.on("open", () => {
          clearTimeout(connectionTimeout);
          this.isConnected = true;
          this.reconnectAttempts = 0;

          console.log("[RobotComm] Connected to ROS bridge");
          this.emit("connected");
          resolve();
        });

        this.ws.on("message", (data: WebSocket.Data) => {
          try {
            const message = JSON.parse(data.toString());
            this.handleMessage(message);
          } catch (error) {
            console.error("[RobotComm] Failed to parse message:", error);
          }
        });

        this.ws.on("close", (code: number, reason: string) => {
          this.isConnected = false;
          console.log(`[RobotComm] Connection closed: ${code} - ${reason}`);

          this.emit("disconnected");

          // Reject all pending actions
          this.rejectAllPendingActions(new Error("Connection lost"));

          // Schedule reconnection if not intentionally closed
          if (
            code !== 1000 &&
            this.reconnectAttempts < this.config.maxReconnectAttempts
          ) {
            this.scheduleReconnect();
          }
        });

        this.ws.on("error", (error: Error) => {
          console.error("[RobotComm] WebSocket error:", error);
          clearTimeout(connectionTimeout);
          this.emit("error", error);
          reject(error);
        });
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Disconnect from ROS bridge
   */
  disconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    this.rejectAllPendingActions(new Error("Service disconnected"));

    if (this.ws) {
      this.ws.close(1000, "Normal closure");
      this.ws = null;
    }

    this.isConnected = false;
    this.emit("disconnected");
  }

  /**
   * Execute a robot action
   */
  async executeAction(action: RobotAction): Promise<RobotActionResult> {
    if (!this.isConnected) {
      throw new Error("Not connected to ROS bridge");
    }

    const startTime = Date.now();
    const actionId = `action_${this.messageId++}`;

    return new Promise((resolve, reject) => {
      // Set up timeout
      const timeout = setTimeout(() => {
        this.pendingActions.delete(actionId);
        reject(new Error(`Action timeout: ${action.actionId}`));
      }, 30000); // 30 second timeout

      // Store pending action
      this.pendingActions.set(actionId, {
        resolve,
        reject,
        timeout,
        startTime,
      });

      try {
        // Execute action based on type and platform
        this.executeRobotActionInternal(action, actionId);
      } catch (error) {
        clearTimeout(timeout);
        this.pendingActions.delete(actionId);
        reject(error);
      }
    });
  }

  /**
   * Check if service is connected
   */
  getConnectionStatus(): boolean {
    return this.isConnected;
  }

  // Private methods

  private executeRobotActionInternal(
    action: RobotAction,
    actionId: string,
  ): void {
    const { implementation, parameters } = action;

    // Build ROS message from template
    const message = this.buildRosMessage(
      implementation.messageTemplate,
      parameters,
    );

    // Publish to ROS topic
    this.publishToTopic(
      implementation.topic,
      implementation.messageType,
      message,
    );

    // For actions that complete immediately (like movement commands),
    // we simulate completion after a short delay
    setTimeout(() => {
      this.completeAction(actionId, {
        success: true,
        duration:
          Date.now() -
          (this.pendingActions.get(actionId)?.startTime || Date.now()),
        data: {
          topic: implementation.topic,
          messageType: implementation.messageType,
          message,
        },
      });
    }, 100);
  }

  private buildRosMessage(
    template: Record<string, unknown>,
    parameters: Record<string, unknown>,
  ): Record<string, unknown> {
    const message: Record<string, unknown> = {};

    for (const [key, value] of Object.entries(template)) {
      if (typeof value === "string" && value.includes("{{")) {
        // Template substitution
        let substituted = value;

        // Replace template variables
        for (const [paramKey, paramValue] of Object.entries(parameters)) {
          const placeholder = `{{${paramKey}}}`;
          if (substituted.includes(placeholder)) {
            substituted = substituted.replace(
              new RegExp(
                placeholder.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"),
                "g",
              ),
              String(paramValue ?? ""),
            );
          }
        }

        // Handle conditional templates
        if (
          substituted.includes("{{") &&
          substituted.includes("?") &&
          substituted.includes(":")
        ) {
          // Simple conditional: {{condition ? valueTrue : valueFalse}}
          const match = substituted.match(
            /\{\{(.+?)\s*\?\s*(.+?)\s*:\s*(.+?)\}\}/,
          );
          if (match && match.length >= 4) {
            const condition = match[1];
            const trueValue = match[2];
            const falseValue = match[3];
            // Evaluate simple conditions
            let conditionResult = false;

            if (condition?.includes("===")) {
              const parts = condition
                .split("===")
                .map((s) => s.trim().replace(/['"]/g, ""));
              if (parts.length >= 2) {
                const left = parts[0];
                const right = parts[1];
                conditionResult = parameters[left || ""] === right;
              }
            }

            substituted = substituted.replace(
              match[0],
              conditionResult ? (trueValue ?? "") : (falseValue ?? ""),
            );
          }
        }

        // Try to parse as number if it looks like one
        if (!isNaN(Number(substituted))) {
          message[key] = Number(substituted);
        } else {
          message[key] = substituted;
        }
      } else if (Array.isArray(value)) {
        // Handle array templates
        message[key] = value.map((item) =>
          typeof item === "string" && item.includes("{{")
            ? this.substituteTemplateString(item, parameters)
            : item,
        );
      } else if (typeof value === "object" && value !== null) {
        // Recursively handle nested objects
        message[key] = this.buildRosMessage(
          value as Record<string, unknown>,
          parameters,
        );
      } else {
        message[key] = value;
      }
    }

    return message;
  }

  private substituteTemplateString(
    template: string,
    parameters: Record<string, unknown>,
  ): unknown {
    let result = template;

    for (const [key, value] of Object.entries(parameters)) {
      const placeholder = `{{${key}}}`;
      if (result.includes(placeholder)) {
        result = result.replace(
          new RegExp(placeholder.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "g"),
          String(value ?? ""),
        );
      }
    }

    // Try to parse as number if it looks like one
    if (!isNaN(Number(result))) {
      return Number(result);
    }

    return result;
  }

  private publishToTopic(
    topic: string,
    messageType: string,
    message: Record<string, unknown>,
  ): void {
    if (!this.ws) return;

    const rosMessage = {
      op: "publish",
      topic,
      type: messageType,
      msg: message,
    };

    console.log(`[RobotComm] Publishing to ${topic}:`, message);
    this.ws.send(JSON.stringify(rosMessage));
  }

  private handleMessage(message: any): void {
    // Handle different types of ROS bridge messages
    switch (message.op) {
      case "publish":
        this.emit("topic_message", message.topic, message.msg);
        break;

      case "service_response":
        this.handleServiceResponse(message);
        break;

      case "status":
        console.log("[RobotComm] Status:", message);
        break;

      default:
        console.log("[RobotComm] Unhandled message:", message);
    }
  }

  private handleServiceResponse(message: any): void {
    // Handle service call responses if needed
    console.log("[RobotComm] Service response:", message);
  }

  private completeAction(actionId: string, result: RobotActionResult): void {
    const pending = this.pendingActions.get(actionId);
    if (pending) {
      clearTimeout(pending.timeout);
      this.pendingActions.delete(actionId);
      pending.resolve(result);
    }
  }

  private rejectAllPendingActions(error: Error): void {
    for (const [actionId, pending] of this.pendingActions.entries()) {
      clearTimeout(pending.timeout);
      pending.reject(error);
    }
    this.pendingActions.clear();
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;

    this.reconnectAttempts++;
    console.log(
      `[RobotComm] Scheduling reconnect attempt ${this.reconnectAttempts}/${this.config.maxReconnectAttempts} in ${this.config.reconnectInterval}ms`,
    );

    this.reconnectTimer = setTimeout(async () => {
      this.reconnectTimer = null;

      try {
        await this.connect();
      } catch (error) {
        console.error("[RobotComm] Reconnect failed:", error);

        if (this.reconnectAttempts < this.config.maxReconnectAttempts) {
          this.scheduleReconnect();
        } else {
          console.error("[RobotComm] Max reconnect attempts reached");
          this.emit("max_reconnects_reached");
        }
      }
    }, this.config.reconnectInterval);
  }

  private cleanup(): void {
    if (this.ws) {
      this.ws.removeAllListeners();
      this.ws = null;
    }
    this.isConnected = false;
  }
}

// Global service instance
let robotCommService: RobotCommunicationService | null = null;

/**
 * Get or create the global robot communication service
 */
export function getRobotCommunicationService(): RobotCommunicationService {
  if (!robotCommService) {
    robotCommService = new RobotCommunicationService();
  }
  return robotCommService;
}

/**
 * Initialize robot communication service with connection
 */
export async function initRobotCommunicationService(): Promise<RobotCommunicationService> {
  const service = getRobotCommunicationService();

  if (!service.getConnectionStatus()) {
    await service.connect();
  }

  return service;
}
