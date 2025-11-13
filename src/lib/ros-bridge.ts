"use client";

/* eslint-disable @typescript-eslint/no-inferrable-types */
/* eslint-disable @typescript-eslint/consistent-generic-constructors */
/* eslint-disable @typescript-eslint/no-unsafe-function-type */
/* eslint-disable @typescript-eslint/no-unsafe-argument */
/* eslint-disable @typescript-eslint/prefer-promise-reject-errors */
/* eslint-disable @typescript-eslint/prefer-nullish-coalescing */
/* eslint-disable @typescript-eslint/no-unsafe-call */

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

export interface RobotStatus {
  connected: boolean;
  battery: number;
  position: { x: number; y: number; theta: number };
  joints: Record<string, number>;
  sensors: Record<string, unknown>;
  lastUpdate: Date;
}

export interface RobotAction {
  id: string;
  type: string;
  parameters: Record<string, unknown>;
  status: "pending" | "executing" | "completed" | "failed";
  startTime?: Date;
  endTime?: Date;
  error?: string;
}

/**
 * ROS WebSocket Bridge for connecting to rosbridge_server
 *
 * This service provides a high-level interface for communicating with ROS robots
 * through the rosbridge WebSocket protocol. It handles connection management,
 * message publishing/subscribing, service calls, and action execution.
 */
export class RosBridge extends EventEmitter {
  private ws: WebSocket | null = null;
  private url: string;
  private reconnectInterval: number = 3000;
  private reconnectTimer: NodeJS.Timeout | null = null;
  private messageId: number = 0;
  private pendingServices: Map<
    string,
    { resolve: Function; reject: Function }
  > = new Map();
  private subscriptions: Map<string, string> = new Map(); // topic -> subscription id
  private robotStatus: RobotStatus = {
    connected: false,
    battery: 0,
    position: { x: 0, y: 0, theta: 0 },
    joints: {},
    sensors: {},
    lastUpdate: new Date(),
  };
  private activeActions: Map<string, RobotAction> = new Map();

  constructor(url: string = "ws://localhost:9090") {
    super();
    this.url = url;
  }

  /**
   * Connect to the ROS bridge WebSocket server
   */
  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(this.url);

        this.ws.onopen = () => {
          console.log("[RosBridge] Connected to ROS bridge");
          this.robotStatus.connected = true;
          this.clearReconnectTimer();
          this.setupSubscriptions();
          this.emit("connected");
          resolve();
        };

        this.ws.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data) as RosMessage;
            this.handleMessage(message);
          } catch (error) {
            console.error("[RosBridge] Failed to parse message:", error);
          }
        };

        this.ws.onclose = (event) => {
          console.log(
            "[RosBridge] Connection closed:",
            event.code,
            event.reason,
          );
          this.robotStatus.connected = false;
          this.emit("disconnected");

          if (event.code !== 1000) {
            // Not a normal closure
            this.scheduleReconnect();
          }
        };

        this.ws.onerror = (error) => {
          console.error("[RosBridge] WebSocket error:", error);
          this.robotStatus.connected = false;
          this.emit("error", error);
          reject(error);
        };

        // Connection timeout
        setTimeout(() => {
          if (this.ws?.readyState !== WebSocket.OPEN) {
            reject(new Error("Connection timeout"));
          }
        }, 5000);
      } catch (error) {
        reject(error);
      }
    });
  }

  /**
   * Disconnect from the ROS bridge
   */
  disconnect(): void {
    this.clearReconnectTimer();

    if (this.ws) {
      this.ws.close(1000, "Manual disconnect");
      this.ws = null;
    }

    this.robotStatus.connected = false;
    this.emit("disconnected");
  }

  /**
   * Check if connected to ROS bridge
   */
  isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  /**
   * Get current robot status
   */
  getRobotStatus(): RobotStatus {
    return { ...this.robotStatus };
  }

  /**
   * Subscribe to a ROS topic
   */
  subscribe(topic: string, messageType: string): string {
    const id = `sub_${this.messageId++}`;

    const message: RosMessage = {
      op: "subscribe",
      topic,
      type: messageType,
      id,
    };

    this.send(message);
    this.subscriptions.set(topic, id);

    return id;
  }

  /**
   * Unsubscribe from a ROS topic
   */
  unsubscribe(topic: string): void {
    const id = this.subscriptions.get(topic);
    if (id) {
      const message: RosMessage = {
        op: "unsubscribe",
        id,
      };

      this.send(message);
      this.subscriptions.delete(topic);
    }
  }

  /**
   * Publish a message to a ROS topic
   */
  publish(
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
   * Call a ROS service
   */
  async callService(
    service: string,
    serviceType: string,
    args: Record<string, unknown> = {},
  ): Promise<Record<string, unknown>> {
    return new Promise((resolve, reject) => {
      const id = `srv_${this.messageId++}`;

      const message: RosMessage = {
        op: "call_service",
        service,
        type: serviceType,
        args,
        id,
      };

      this.pendingServices.set(id, { resolve, reject });
      this.send(message);

      // Service call timeout
      setTimeout(() => {
        if (this.pendingServices.has(id)) {
          this.pendingServices.delete(id);
          reject(new Error(`Service call timeout: ${service}`));
        }
      }, 10000);
    });
  }

  /**
   * Execute a robot action (high-level NAO action)
   */
  async executeAction(
    actionType: string,
    parameters: Record<string, unknown>,
  ): Promise<RobotAction> {
    const action: RobotAction = {
      id: `action_${this.messageId++}`,
      type: actionType,
      parameters,
      status: "pending",
      startTime: new Date(),
    };

    this.activeActions.set(action.id, action);
    this.emit("action_started", action);

    try {
      // Map action to ROS service calls based on NAO plugin configuration
      switch (actionType) {
        case "say_text":
          await this.naoSayText(parameters.text as string, parameters);
          break;

        case "walk_to_position":
          await this.naoWalkTo(
            parameters.x as number,
            parameters.y as number,
            parameters.theta as number,
          );
          break;

        case "play_animation":
          await this.naoPlayAnimation(parameters.animation as string);
          break;

        case "set_led_color":
          await this.naoSetLedColor(
            parameters.color as string,
            parameters.intensity as number,
          );
          break;

        case "sit_down":
          await this.naoSitDown();
          break;

        case "stand_up":
          await this.naoStandUp();
          break;

        case "turn_head":
          await this.naoTurnHead(
            parameters.yaw as number,
            parameters.pitch as number,
            parameters.speed as number,
          );
          break;

        default:
          throw new Error(`Unknown action type: ${actionType}`);
      }

      action.status = "completed";
      action.endTime = new Date();
      this.emit("action_completed", action);
    } catch (error) {
      action.status = "failed";
      action.error = error instanceof Error ? error.message : String(error);
      action.endTime = new Date();
      this.emit("action_failed", action);
    }

    this.activeActions.set(action.id, action);
    return action;
  }

  /**
   * Get list of active actions
   */
  getActiveActions(): RobotAction[] {
    return Array.from(this.activeActions.values());
  }

  // Private methods

  private send(message: RosMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.warn("[RosBridge] Cannot send message - not connected");
    }
  }

  private handleMessage(message: RosMessage): void {
    switch (message.op) {
      case "publish":
        this.handleTopicMessage(message);
        break;

      case "service_response":
        this.handleServiceResponse(message);
        break;

      case "status":
        // Handle status messages from rosbridge
        break;

      default:
        console.log("[RosBridge] Unhandled message:", message);
    }
  }

  private handleTopicMessage(message: RosMessage): void {
    if (!message.topic || !message.msg) return;

    // Update robot status based on subscribed topics
    switch (message.topic) {
      case "/battery_state":
        if (typeof message.msg.percentage === "number") {
          this.robotStatus.battery = message.msg.percentage;
        }
        break;

      case "/joint_states":
        if (message.msg.name && message.msg.position) {
          const names = message.msg.name as string[];
          const positions = message.msg.position as number[];

          for (let i = 0; i < names.length; i++) {
            const jointName = names[i];
            const jointPosition = positions[i];
            if (jointName && jointPosition !== undefined) {
              this.robotStatus.joints[jointName] = jointPosition;
            }
          }
        }
        break;

      case "/robot_pose":
        if (message.msg.position) {
          const pos = message.msg.position as Record<string, number>;
          this.robotStatus.position = {
            x: pos.x || 0,
            y: pos.y || 0,
            theta: pos.theta || 0,
          };
        }
        break;
    }

    this.robotStatus.lastUpdate = new Date();
    this.emit("topic_message", message.topic, message.msg);
    this.emit("status_update", this.robotStatus);
  }

  private handleServiceResponse(message: RosMessage): void {
    if (!message.id) return;

    const pending = this.pendingServices.get(message.id);
    if (pending) {
      this.pendingServices.delete(message.id);

      if (message.result) {
        pending.resolve(message.values || {});
      } else {
        pending.reject(new Error(`Service call failed: ${message.id}`));
      }
    }
  }

  private setupSubscriptions(): void {
    // Subscribe to common robot topics
    this.subscribe("/battery_state", "sensor_msgs/BatteryState");
    this.subscribe("/joint_states", "sensor_msgs/JointState");
    this.subscribe("/robot_pose", "geometry_msgs/PoseStamped");
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;

    console.log(
      `[RosBridge] Scheduling reconnect in ${this.reconnectInterval}ms`,
    );
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect().catch((error) => {
        console.error("[RosBridge] Reconnect failed:", error);
        this.scheduleReconnect();
      });
    }, this.reconnectInterval);
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  // NAO-specific action implementations

  private async naoSayText(
    text: string,
    params: Record<string, unknown>,
  ): Promise<void> {
    await this.callService("/say_text", "nao_msgs/SayText", {
      text,
      volume: params.volume || 0.7,
      speed: params.speed || 100,
    });
  }

  private async naoWalkTo(x: number, y: number, theta: number): Promise<void> {
    await this.callService("/walk_to", "nao_msgs/WalkTo", { x, y, theta });
  }

  private async naoPlayAnimation(animation: string): Promise<void> {
    await this.callService("/play_animation", "nao_msgs/PlayAnimation", {
      animation: `animations/Stand/Gestures/${animation}`,
    });
  }

  private async naoSetLedColor(
    color: string,
    intensity: number = 1.0,
  ): Promise<void> {
    const colorMap: Record<string, [number, number, number]> = {
      red: [1, 0, 0],
      green: [0, 1, 0],
      blue: [0, 0, 1],
      yellow: [1, 1, 0],
      magenta: [1, 0, 1],
      cyan: [0, 1, 1],
      white: [1, 1, 1],
      orange: [1, 0.5, 0],
      pink: [1, 0.7, 0.7],
    };

    const rgb = colorMap[color] ?? [0, 0, 1];
    await this.callService("/set_led_color", "nao_msgs/SetLedColor", {
      name: "FaceLeds",
      r: rgb[0] * intensity,
      g: rgb[1] * intensity,
      b: rgb[2] * intensity,
      duration: 1.0,
    });
  }

  private async naoSitDown(): Promise<void> {
    await this.callService("/sit_down", "std_srvs/Empty", {});
  }

  private async naoStandUp(): Promise<void> {
    await this.callService("/stand_up", "std_srvs/Empty", {});
  }

  private async naoTurnHead(
    yaw: number,
    pitch: number,
    speed: number = 0.3,
  ): Promise<void> {
    await this.callService("/move_head", "nao_msgs/MoveHead", {
      yaw,
      pitch,
      speed,
    });
  }
}

// Global ROS bridge instance
let rosBridgeInstance: RosBridge | null = null;

/**
 * Get or create the global ROS bridge instance
 */
export function getRosBridge(url?: string): RosBridge {
  if (!rosBridgeInstance) {
    rosBridgeInstance = new RosBridge(url);
  }
  return rosBridgeInstance;
}

/**
 * Initialize ROS bridge with connection
 */
export async function initRosBridge(url?: string): Promise<RosBridge> {
  const bridge = getRosBridge(url);

  if (!bridge.isConnected()) {
    await bridge.connect();
  }

  return bridge;
}
