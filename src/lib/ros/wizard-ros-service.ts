"use client";

import { EventEmitter } from "events";

export interface RosMessage {
  op: string;
  topic?: string;
  type?: string;
  msg?: Record<string, unknown>;
  service?: string;
  args?: Record<string, unknown>;
  id?: string;
  result?: boolean;
  values?: Record<string, unknown>;
}

export interface ServiceRequest {
  service: string;
  args?: Record<string, unknown>;
}

export interface ServiceResponse {
  result: boolean;
  values?: Record<string, unknown>;
  error?: string;
}

export interface RobotStatus {
  connected: boolean;
  battery: number;
  position: { x: number; y: number; theta: number };
  joints: Record<string, number>;
  sensors: Record<string, unknown>;
  lastUpdate: Date;
}

export interface RobotActionExecution {
  id: string;
  actionId: string;
  pluginName: string;
  parameters: Record<string, unknown>;
  status: "pending" | "executing" | "completed" | "failed";
  startTime: Date;
  endTime?: Date;
  error?: string;
}

/**
 * Unified ROS WebSocket service for wizard interface
 * Manages connection to rosbridge and handles robot action execution
 */
export class WizardRosService extends EventEmitter {
  private ws: WebSocket | null = null;
  private url: string;
  private reconnectInterval = 3000;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private messageId = 0;
  private isConnected = false;
  private connectionAttempts = 0;
  private maxReconnectAttempts = 5;
  private isConnecting = false;

  // Robot state
  private robotStatus: RobotStatus = {
    connected: false,
    battery: 0,
    position: { x: 0, y: 0, theta: 0 },
    joints: {},
    sensors: {},
    lastUpdate: new Date(),
  };

  // Active action tracking
  private activeActions: Map<string, RobotActionExecution> = new Map();

  constructor(url: string = "ws://localhost:9090") {
    super();
    this.url = url;
  }

  /**
   * Connect to ROS bridge WebSocket
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (
        this.isConnected ||
        this.ws?.readyState === WebSocket.OPEN ||
        this.isConnecting
      ) {
        if (this.isConnected) resolve();
        return;
      }

      this.isConnecting = true;
      console.log(`[WizardROS] Connecting to ${this.url}`);
      this.ws = new WebSocket(this.url);

      const connectionTimeout = setTimeout(() => {
        if (this.ws?.readyState !== WebSocket.OPEN) {
          this.ws?.close();
          reject(new Error("Connection timeout"));
        }
      }, 10000);

      this.ws.onopen = () => {
        clearTimeout(connectionTimeout);
        console.log("[WizardROS] Connected successfully");
        this.isConnected = true;
        this.isConnecting = false;
        this.connectionAttempts = 0;
        this.clearReconnectTimer();

        // Subscribe to robot topics
        this.subscribeToRobotTopics();

        this.emit("connected");
        resolve();
      };

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data) as RosMessage;
          this.handleMessage(message);
        } catch (error) {
          console.error("[WizardROS] Failed to parse message:", error);
        }
      };

      this.ws.onclose = (event) => {
        console.log(
          `[WizardROS] Connection closed: ${event.code} - ${event.reason}`,
        );
        this.isConnected = false;
        this.isConnecting = false;
        this.emit("disconnected");

        // Schedule reconnect if not manually closed
        if (
          event.code !== 1000 &&
          this.connectionAttempts < this.maxReconnectAttempts
        ) {
          this.scheduleReconnect();
        }
      };

      this.ws.onerror = (error) => {
        console.warn("[WizardROS] WebSocket error (connection may be retried):", error);
        clearTimeout(connectionTimeout);
        this.isConnecting = false;

        // Prevent unhandled error event if no listeners
        if (this.listenerCount("error") > 0) {
          this.emit("error", error);
        }
        reject(error);
      };
    });
  }

  /**
   * Disconnect from ROS bridge
   */
  disconnect(): void {
    this.clearReconnectTimer();

    if (this.ws) {
      this.ws.close(1000, "Manual disconnect");
      this.ws = null;
    }

    this.isConnected = false;
    this.isConnecting = false;
    this.robotStatus.connected = false;
    this.emit("disconnected");
  }

  /**
   * Check if connected to ROS bridge
   */
  getConnectionStatus(): boolean {
    return this.isConnected && this.ws?.readyState === WebSocket.OPEN;
  }

  /**
   * Get current robot status
   */
  getRobotStatus(): RobotStatus {
    return { ...this.robotStatus };
  }

  /**
   * Execute robot action using plugin configuration
   */
  async executeRobotAction(
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
  ): Promise<RobotActionExecution> {
    if (!this.isConnected) {
      throw new Error("Not connected to ROS bridge");
    }

    const executionId = `${pluginName}_${actionId}_${Date.now()}`;
    const execution: RobotActionExecution = {
      id: executionId,
      actionId,
      pluginName,
      parameters,
      status: "pending",
      startTime: new Date(),
    };

    this.activeActions.set(executionId, execution);
    this.emit("action_started", execution);

    try {
      execution.status = "executing";
      this.activeActions.set(executionId, execution);

      // Execute based on action configuration or built-in mappings
      if (actionConfig) {
        await this.executeWithConfig(actionConfig, parameters);
      } else {
        await this.executeBuiltinAction(actionId, parameters);
      }

      execution.status = "completed";
      execution.endTime = new Date();
      this.emit("action_completed", execution);
    } catch (error) {
      execution.status = "failed";
      execution.error = error instanceof Error ? error.message : String(error);
      execution.endTime = new Date();
      this.emit("action_failed", execution);
    }

    this.activeActions.set(executionId, execution);
    return execution;
  }

  /**
   * Get list of active actions
   */
  getActiveActions(): RobotActionExecution[] {
    return Array.from(this.activeActions.values());
  }

  /**
   * Subscribe to robot sensor topics
   */
  private subscribeToRobotTopics(): void {
    const topics = [
      { topic: "/joint_states", type: "sensor_msgs/JointState" },
      // Battery topic removed - BatteryState message type doesn't exist in naoqi_bridge_msgs
      // Battery info can be obtained through diagnostics or other means if needed
      { topic: "/naoqi_driver/bumper", type: "naoqi_bridge_msgs/Bumper" },
      {
        topic: "/naoqi_driver/hand_touch",
        type: "naoqi_bridge_msgs/HandTouch",
      },
      {
        topic: "/naoqi_driver/head_touch",
        type: "naoqi_bridge_msgs/HeadTouch",
      },
      { topic: "/naoqi_driver/sonar/left", type: "sensor_msgs/Range" },
      { topic: "/naoqi_driver/sonar/right", type: "sensor_msgs/Range" },
    ];

    topics.forEach(({ topic, type }) => {
      this.subscribe(topic, type);
    });
  }

  /**
   * Subscribe to a ROS topic
   */
  private subscribe(topic: string, messageType: string): void {
    const message: RosMessage = {
      op: "subscribe",
      topic,
      type: messageType,
      id: `sub_${this.messageId++}`,
    };

    this.send(message);
  }

  /**
   * Publish message to ROS topic
   */
  private publish(
    topic: string,
    messageType: string,
    msg: Record<string, unknown>,
  ): void {
    const message: RosMessage = {
      op: "publish",
      topic,
      type: messageType,
      msg,
    };

    this.send(message);
  }

  /**
   * Send WebSocket message
   */
  private send(message: RosMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.warn("[WizardROS] Cannot send message - not connected");
    }
  }

  /**
   * Handle incoming ROS messages
   */
  private handleMessage(message: RosMessage): void {
    if (message.topic) {
      this.handleTopicMessage(message);
    }

    this.emit("message", message);
  }

  /**
   * Handle topic-specific messages
   */
  private handleTopicMessage(message: RosMessage): void {
    if (!message.topic || !message.msg) return;

    switch (message.topic) {
      case "/joint_states":
        this.updateJointStates(message.msg);
        break;
      case "/naoqi_driver/battery":
        this.updateBatteryStatus(message.msg);
        break;
      case "/naoqi_driver/bumper":
      case "/naoqi_driver/hand_touch":
      case "/naoqi_driver/head_touch":
      case "/naoqi_driver/sonar/left":
      case "/naoqi_driver/sonar/right":
        this.updateSensorData(message.topic, message.msg);
        break;
    }

    this.robotStatus.lastUpdate = new Date();
    this.emit("robot_status_updated", this.robotStatus);
  }

  /**
   * Update joint states from ROS message
   */
  private updateJointStates(msg: Record<string, unknown>): void {
    if (
      msg.name &&
      msg.position &&
      Array.isArray(msg.name) &&
      Array.isArray(msg.position)
    ) {
      const joints: Record<string, number> = {};

      for (let i = 0; i < msg.name.length; i++) {
        const jointName = msg.name[i] as string;
        const position = msg.position[i] as number;
        if (jointName && typeof position === "number") {
          joints[jointName] = position;
        }
      }

      this.robotStatus.joints = joints;
      this.robotStatus.connected = true;
    }
  }

  /**
   * Update battery status from ROS message
   */
  private updateBatteryStatus(msg: Record<string, unknown>): void {
    if (typeof msg.percentage === "number") {
      this.robotStatus.battery = msg.percentage;
    }
  }

  /**
   * Update sensor data from ROS message
   */
  private updateSensorData(topic: string, msg: Record<string, unknown>): void {
    this.robotStatus.sensors[topic] = msg;
  }

  /**
   * Execute action with plugin configuration
   */
  private async executeWithConfig(
    config: {
      topic: string;
      messageType: string;
      payloadMapping: {
        type: string;
        payload?: Record<string, unknown>;
        transformFn?: string;
      };
    },
    parameters: Record<string, unknown>,
  ): Promise<void> {
    let msg: Record<string, unknown>;

    if (
      (config.payloadMapping.type === "template" ||
        config.payloadMapping.type === "static") &&
      config.payloadMapping.payload
    ) {
      // Template-based payload construction
      msg = this.buildTemplatePayload(
        config.payloadMapping.payload,
        parameters,
      );
    } else if (config.payloadMapping.transformFn) {
      // Custom transform function
      msg = this.applyTransformFunction(
        config.payloadMapping.transformFn,
        parameters,
      );
    } else {
      // Direct parameter mapping
      msg = parameters;
    }

    this.publish(config.topic, config.messageType, msg);

    // Wait for action completion (simple delay for now)
    await new Promise((resolve) => setTimeout(resolve, 100));
  }

  /**
   * Execute built-in robot actions
   */
  private async executeBuiltinAction(
    actionId: string,
    parameters: Record<string, unknown>,
  ): Promise<void> {
    switch (actionId) {
      case "say_text":
        this.publish("/speech", "std_msgs/String", {
          data: parameters.text || "Hello",
        });
        break;

      case "walk_forward":
      case "walk_backward":
      case "turn_left":
      case "turn_right":
        this.executeMovementAction(actionId, parameters);
        break;

      case "move_head":
      case "turn_head":
        this.executeTurnHead(parameters);
        break;

      case "move_arm":
        this.executeMoveArm(parameters);
        break;

      case "emergency_stop":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        break;

      default:
        throw new Error(`Unknown action: ${actionId}`);
    }

    // Wait for action completion
    await new Promise((resolve) => setTimeout(resolve, 100));
  }

  /**
   * Execute movement actions
   */
  private executeMovementAction(
    actionId: string,
    parameters: Record<string, unknown>,
  ): void {
    let linear = { x: 0, y: 0, z: 0 };
    let angular = { x: 0, y: 0, z: 0 };

    const speed = Number(parameters.speed) || 0.1;

    switch (actionId) {
      case "walk_forward":
        linear.x = speed;
        break;
      case "walk_backward":
        linear.x = -speed;
        break;
      case "turn_left":
        angular.z = speed;
        break;
      case "turn_right":
        angular.z = -speed;
        break;
    }

    this.publish("/naoqi_driver/cmd_vel", "geometry_msgs/Twist", { linear, angular });
  }

  /**
   * Execute head turn action
   */
  private executeTurnHead(parameters: Record<string, unknown>): void {
    const yaw = Number(parameters.yaw) || 0;
    const pitch = Number(parameters.pitch) || 0;
    const speed = Number(parameters.speed) || 0.3;

    this.publish("/joint_angles", "naoqi_bridge_msgs/JointAnglesWithSpeed", {
      joint_names: ["HeadYaw", "HeadPitch"],
      joint_angles: [yaw, pitch],
      speed: speed,
    });
  }

  /**
   * Execute arm movement
   */
  private executeMoveArm(parameters: Record<string, unknown>): void {
    const arm = String(parameters.arm || "Right");
    const roll = Number(parameters.roll) || 0;
    const pitch = Number(parameters.pitch) || 0;
    const speed = Number(parameters.speed) || 0.2;

    const prefix = arm === "Left" ? "L" : "R";
    const jointNames = [`${prefix}ShoulderPitch`, `${prefix}ShoulderRoll`];
    const jointAngles = [pitch, roll];

    this.publish("/naoqi_driver/joint_angles", "naoqi_bridge_msgs/JointAnglesWithSpeed", {
      joint_names: jointNames,
      joint_angles: jointAngles,
      speed: speed,
    });
  }

  /**
   * Call a ROS service
   */
  async callService(
    service: string,
    args: Record<string, unknown> = {},
  ): Promise<ServiceResponse> {
    if (!this.isConnected) {
      throw new Error("Not connected to ROS bridge");
    }

    const id = `call_${this.messageId++}`;

    return new Promise((resolve, reject) => {
      const handleResponse = (message: RosMessage) => {
        if (message.op === "service_response" && message.id === id) {
          this.off("message", handleResponse);
          if (message.result === false) {
            resolve({ result: false, error: String(message.values || "Service call failed") });
          } else {
            resolve({ result: true, values: message.values });
          }
        }
      };

      this.on("message", handleResponse);

      this.send({
        op: "call_service",
        service,
        args,
        id,
      });

      setTimeout(() => {
        this.off("message", handleResponse);
        reject(new Error("Service call timed out"));
      }, 5000);
    });
  }

  /**
   * Set Autonomous Life state with fallbacks
   */
  async setAutonomousLife(enabled: boolean): Promise<boolean> {
    const desiredState = enabled ? "interactive" : "disabled";

    // List of services to try in order
    const attempts = [
      // Standard NaoQi Bridge pattern
      {
        service: "/naoqi_driver/ALAutonomousLife/setState",
        args: { state: desiredState }
      },
      {
        service: "/naoqi_driver/ALAutonomousLife/set_state",
        args: { state: desiredState }
      },
      // Direct module mapping
      {
        service: "/ALAutonomousLife/setState",
        args: { state: desiredState }
      },
      // Shortcuts/Aliases
      {
        service: "/naoqi_driver/set_autonomous_life",
        args: { state: desiredState }
      },
      {
        service: "/autonomous_life/set_state",
        args: { state: desiredState }
      },
      // Fallback: Enable/Disable topics/services
      {
        service: enabled ? "/life/enable" : "/life/disable",
        args: {}
      },
      // Last resort: Generic proxy call (if available)
      {
        service: "/naoqi_driver/function_call",
        args: {
          service: "ALAutonomousLife",
          function: "setState",
          args: [desiredState]
        }
      }
    ];

    console.log(`[WizardROS] Setting Autonomous Life to: ${desiredState}`);

    for (const attempt of attempts) {
      try {
        console.log(`[WizardROS] Trying service: ${attempt.service}`);
        const response = await this.callService(attempt.service, attempt.args);

        // If the service call didn't timeout (it resolved), check result
        if (response.result) {
          console.log(`[WizardROS] Success via ${attempt.service}`);
          return true;
        } else {
          // Resolved but failed? (e.g. internal error)
          console.warn(`[WizardROS] Service ${attempt.service} returned false result:`, response.error);
        }
      } catch (error) {
        // Service call failed or timed out
        console.warn(`[WizardROS] Service ${attempt.service} failed/timeout:`, error);
      }
    }

    console.error("[WizardROS] All Autonomous Life service attempts failed.");
    return false;
  }

  /**
   * Build template-based payload
   */
  private buildTemplatePayload(
    template: Record<string, unknown>,
    parameters: Record<string, unknown>,
  ): Record<string, unknown> {
    const result: Record<string, unknown> = {};

    for (const [key, value] of Object.entries(template)) {
      if (typeof value === "string" && value.includes("{{")) {
        // Template substitution
        let substituted = value;
        for (const [paramKey, paramValue] of Object.entries(parameters)) {
          const placeholder = `{{${paramKey}}}`;
          substituted = substituted.replace(
            new RegExp(placeholder.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "g"),
            String(paramValue ?? ""),
          );
        }
        result[key] = isNaN(Number(substituted))
          ? substituted
          : Number(substituted);
      } else if (typeof value === "object" && value !== null) {
        result[key] = this.buildTemplatePayload(
          value as Record<string, unknown>,
          parameters,
        );
      } else {
        result[key] = value;
      }
    }

    return result;
  }

  /**
   * Apply transform function for NAO6 actions
   */
  private applyTransformFunction(
    transformFn: string,
    parameters: Record<string, unknown>,
  ): Record<string, unknown> {
    switch (transformFn) {
      case "naoVelocityTransform":
        return {
          linear: {
            x: Number(parameters.linear) || 0,
            y: 0,
            z: 0,
          },
          angular: {
            x: 0,
            y: 0,
            z: Number(parameters.angular) || 0,
          },
        };

      case "naoSpeechTransform":
      case "transformToStringMessage":
        return {
          data: String(parameters.text || "Hello"),
        };

      case "naoHeadTransform":
      case "transformToHeadMovement":
        return {
          joint_names: ["HeadYaw", "HeadPitch"],
          joint_angles: [
            Number(parameters.yaw) || 0,
            Number(parameters.pitch) || 0,
          ],
          speed: Number(parameters.speed) || 0.3,
        };

      case "transformToJointAngles":
        return {
          joint_names: [String(parameters.joint_name || "HeadYaw")],
          joint_angles: [Number(parameters.angle) || 0],
          speed: Number(parameters.speed) || 0.2,
        };

      default:
        console.warn(`Unknown transform function: ${transformFn}`);
        return parameters;
    }
  }

  /**
   * Schedule reconnection attempt
   */
  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;

    this.connectionAttempts++;
    console.log(
      `[WizardROS] Scheduling reconnect attempt ${this.connectionAttempts}/${this.maxReconnectAttempts}`,
    );

    this.reconnectTimer = setTimeout(async () => {
      this.reconnectTimer = null;
      try {
        await this.connect();
      } catch (error) {
        console.warn("[WizardROS] Reconnect failed:", error);
        if (this.connectionAttempts < this.maxReconnectAttempts) {
          this.scheduleReconnect();
        } else {
          this.emit("max_reconnects_reached");
        }
      }
    }, this.reconnectInterval);
  }

  /**
   * Clear reconnect timer
   */
  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }
}

// Global service instance
let wizardRosService: WizardRosService | null = null;
let isCreatingInstance = false;

/**
 * Get or create the global wizard ROS service (true singleton)
 */
export function getWizardRosService(): WizardRosService {
  // Prevent multiple instances during creation
  if (isCreatingInstance && !wizardRosService) {
    throw new Error("WizardRosService is being initialized, please wait");
  }

  if (!wizardRosService) {
    isCreatingInstance = true;
    try {
      wizardRosService = new WizardRosService();
    } finally {
      isCreatingInstance = false;
    }
  }
  return wizardRosService;
}

/**
 * Initialize wizard ROS service with connection
 */
export async function initWizardRosService(): Promise<WizardRosService> {
  const service = getWizardRosService();

  if (!service.getConnectionStatus()) {
    await service.connect();
  }

  return service;
}
