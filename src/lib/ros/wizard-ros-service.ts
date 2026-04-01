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

  // Simulation mode
  private simulationMode: boolean;
  private simulationInterval: NodeJS.Timeout | null = null;

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

  constructor(url: string = "ws://localhost:9090", simulationMode: boolean = false) {
    super();
    this.url = url;
    this.simulationMode = simulationMode || 
      (typeof window !== "undefined" && process.env.NEXT_PUBLIC_SIMULATION_MODE === "true");
  }

  /**
   * Check if running in simulation mode
   */
  isSimulationMode(): boolean {
    return this.simulationMode;
  }

  /**
   * Enable or disable simulation mode
   */
  setSimulationMode(enabled: boolean): void {
    this.simulationMode = enabled;
    if (!enabled && this.simulationInterval) {
      clearInterval(this.simulationInterval);
      this.simulationInterval = null;
    }
  }

  /**
   * Connect to ROS bridge WebSocket
   */
  async connect(): Promise<void> {
    // Simulation mode - fake connection
    if (this.simulationMode) {
      return this.connectSimulation();
    }

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
        console.warn(
          "[WizardROS] WebSocket error (connection may be retried):",
          error,
        );
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

    if (this.simulationMode) {
      this.disconnectSimulation();
      return;
    }

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
   * Simulation mode connection - simulates robot responses
   */
  private async connectSimulation(): Promise<void> {
    console.log(`[WizardROS] SIMULATION MODE - Connecting to mock robot`);
    this.isConnected = true;
    this.isConnecting = false;
    this.connectionAttempts = 0;

    // Initialize mock robot state
    const mockStates = this.getMockJointStates();
    this.robotStatus = {
      connected: true,
      battery: 85,
      position: { x: 0, y: 0, theta: 0 },
      joints: mockStates.names.reduce((acc, name, i) => {
        acc[name] = mockStates.positions[i] ?? 0;
        return acc;
      }, {} as Record<string, number>),
      sensors: {},
      lastUpdate: new Date(),
    };

    // Start publishing simulated sensor data
    this.simulationInterval = setInterval(() => {
      this.publishSimulationData();
    }, 100);

    this.emit("connected");
    console.log(`[WizardROS] SIMULATION MODE - Connected to mock robot`);
  }

  /**
   * Simulation mode disconnection
   */
  private disconnectSimulation(): void {
    console.log(`[WizardROS] SIMULATION MODE - Disconnecting`);
    if (this.simulationInterval) {
      clearInterval(this.simulationInterval);
      this.simulationInterval = null;
    }
    this.isConnected = false;
    this.robotStatus.connected = false;
    this.emit("disconnected");
  }

  /**
   * Publish simulated sensor data
   */
  private publishSimulationData(): void {
    if (!this.simulationMode || !this.isConnected) return;

    const mockData = this.getMockJointStates();
    this.updateJointStates(mockData);
    this.robotStatus.battery = 85 + Math.random() * 2 - 1; // Slight variation
    this.robotStatus.sensors = {
      "/bumper": { left: false, right: false },
      "/hand_touch": { leftHand: false, rightHand: false },
      "/head_touch": { front: false, middle: false, rear: false },
      "/sonar/left": { range: 0.5 + Math.random() * 0.5 },
      "/sonar/right": { range: 0.5 + Math.random() * 0.5 },
    };
    this.robotStatus.lastUpdate = new Date();
    this.emit("robot_status_updated", this.robotStatus);
  }

  /**
   * Get mock joint states for simulation
   */
  private getMockJointStates(): { names: string[]; positions: number[] } {
    const names = [
      "HeadYaw", "HeadPitch",
      "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
      "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand",
      "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
      "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    ];
    const positions = names.map(() => (Math.random() - 0.5) * 0.1);
    return { names, positions };
  }

  /**
   * Execute action in simulation mode
   */
  private async executeSimulationAction(
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

      console.log(`[WizardROS] SIMULATION MODE - Executing ${actionId}:`, parameters);

      // Simulate action execution based on action type
      let duration = 500;

      if (actionId === "say_text" || actionId === "say_with_emotion" || actionConfig?.topic === "/speech") {
        const text = String(parameters.text || parameters.data || "Hello");
        const wordCount = text.split(/\s+/).filter(Boolean).length;
        duration = 1500 + Math.max(1000, wordCount * 300);
      } else if (actionId.includes("walk") || actionId.includes("turn") || actionConfig?.topic === "/cmd_vel") {
        duration = 500;
        // Simulate position change
        const speed = Number(parameters.speed) || 0.1;
        if (actionId === "walk_forward") {
          this.robotStatus.position.x += speed * 0.5;
        } else if (actionId === "walk_backward") {
          this.robotStatus.position.x -= speed * 0.5;
        } else if (actionId === "turn_left") {
          this.robotStatus.position.theta -= 0.5;
        } else if (actionId === "turn_right") {
          this.robotStatus.position.theta += 0.5;
        }
      } else if (actionId.includes("head") || actionId.includes("move") || actionConfig?.topic === "/joint_angles") {
        duration = 1000;
      }

      // Simulate async execution
      await new Promise((resolve) => setTimeout(resolve, duration));

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
   * Check if connected to ROS bridge
   */
  getConnectionStatus(): boolean {
    if (this.simulationMode) {
      return this.isConnected;
    }
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

    // Simulation mode - simulate action execution
    if (this.simulationMode) {
      return this.executeSimulationAction(pluginName, actionId, parameters, actionConfig);
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
    console.log("[WizardROS] Setting up robot topics...");
    const topics = [
      { topic: "/joint_states", type: "sensor_msgs/JointState" },
      { topic: "/bumper", type: "naoqi_bridge_msgs/Bumper" },
      { topic: "/hand_touch", type: "naoqi_bridge_msgs/HandTouch" },
      { topic: "/head_touch", type: "naoqi_bridge_msgs/HeadTouch" },
      { topic: "/sonar/left", type: "sensor_msgs/Range" },
      { topic: "/sonar/right", type: "sensor_msgs/Range" },
    ];

    topics.forEach(({ topic, type }) => {
      this.subscribe(topic, type);
    });

    this.advertise("/speech", "std_msgs/String");
    this.advertise("/cmd_vel", "geometry_msgs/Twist");
    this.advertise("/robot_pose", "geometry_msgs/Pose");
    this.advertise("/animation", "std_msgs/String");
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
   * Advertise a ROS topic (declare the type before publishing)
   */
  private advertise(topic: string, messageType: string): void {
    console.log(`[WizardROS] Advertising topic ${topic} as ${messageType}`);
    const message: RosMessage = {
      op: "advertise",
      topic,
      type: messageType,
      id: `adv_${this.messageId++}`,
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
    console.log(`[WizardROS] Publishing to ${topic}:`, msg);
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
      case "/bumper":
      case "/hand_touch":
      case "/head_touch":
      case "/sonar/left":
      case "/sonar/right":
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

    // Wait for action completion based on topic type
    if (config.topic === "/speech") {
      // Estimate speech duration based on text content
      const text =
        typeof msg === "object" && msg !== null && "data" in msg
          ? String((msg as any).data || "")
          : JSON.stringify(msg);
      const wordCount = text.split(/\s+/).filter(Boolean).length;
      // Emotion markup adds overhead: ~200ms per word base + emotion animation time
      const emotionOverhead = 1500; // Animation prep time
      const duration = emotionOverhead + Math.max(1000, wordCount * 300);
      console.log(
        `[WizardROS] Speech action estimated duration: ${duration}ms (${wordCount} words)`,
      );
      await new Promise((resolve) => setTimeout(resolve, duration));
    } else {
      // Short delay for non-speech actions
      await new Promise((resolve) => setTimeout(resolve, 500));
    }
  }

  /**
   * Execute built-in robot actions (robot-agnostic defaults)
   * These are generic actions that work with standard ROS topics
   */
  private async executeBuiltinAction(
    actionId: string,
    parameters: Record<string, unknown>,
  ): Promise<void> {
    switch (actionId) {
      case "say_text":
      case "say_with_emotion":
        const text = String(parameters.text || "Hello");
        this.publish("/speech", "std_msgs/String", { data: text });
        const wordCount = text.split(/\s+/).filter(Boolean).length;
        const emotion = String(parameters.emotion || "neutral");
        const emotionOverhead = 1500;
        const duration = emotionOverhead + Math.max(1000, wordCount * 300);
        console.log(
          `[WizardROS] Speech action (${actionId}) estimated: ${duration}ms`,
        );
        await new Promise((resolve) => setTimeout(resolve, duration));
        break;

      case "wave_goodbye":
        const waveText = String(parameters.text || "Goodbye");
        this.publish("/speech", "std_msgs/String", { data: waveText });
        await new Promise((resolve) => setTimeout(resolve, 3000));
        break;

      case "walk_forward":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: Number(parameters.speed) || 0.1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        await new Promise((resolve) => setTimeout(resolve, 500));
        break;

      case "walk_backward":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: -(Number(parameters.speed) || 0.1), y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        await new Promise((resolve) => setTimeout(resolve, 500));
        break;

      case "turn_left":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: -(Number(parameters.speed) || 0.3) },
        });
        await new Promise((resolve) => setTimeout(resolve, 500));
        break;

      case "turn_right":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: Number(parameters.speed) || 0.3 },
        });
        await new Promise((resolve) => setTimeout(resolve, 500));
        break;

      case "move_head":
      case "turn_head":
        this.publish(
          "/joint_angles",
          "naoqi_bridge_msgs/JointAnglesWithSpeed",
          {
            joint_names: ["HeadYaw", "HeadPitch"],
            joint_angles: [
              Number(parameters.yaw) || 0,
              Number(parameters.pitch) || 0,
            ],
            speed: Number(parameters.speed) || 0.3,
          },
        );
        await new Promise((resolve) => setTimeout(resolve, 1000));
        break;

      case "move_arm":
        const arm = String(parameters.arm || "right");
        const prefix = arm.toLowerCase() === "left" ? "L" : "R";
        this.publish(
          "/joint_angles",
          "naoqi_bridge_msgs/JointAnglesWithSpeed",
          {
            joint_names: [
              `${prefix}ShoulderPitch`,
              `${prefix}ShoulderRoll`,
              `${prefix}ElbowYaw`,
              `${prefix}ElbowRoll`,
            ],
            joint_angles: [
              Number(parameters.shoulder_pitch) || 0,
              Number(parameters.shoulder_roll) || 0,
              Number(parameters.elbow_yaw) || 0,
              Number(parameters.elbow_roll) || 0,
            ],
            speed: Number(parameters.speed) || 0.3,
          },
        );
        await new Promise((resolve) => setTimeout(resolve, 1000));
        break;

      case "emergency_stop":
        this.publish("/cmd_vel", "geometry_msgs/Twist", {
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        });
        break;

      default:
        throw new Error(
          `Unknown action: ${actionId}. Define this action in your robot plugin.`,
        );
    }
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

    // Simulation mode - return mock responses
    if (this.simulationMode) {
      console.log(`[WizardROS] SIMULATION MODE - Service call: ${service}`, args);

      const mockResponses: Record<string, ServiceResponse> = {
        "/naoqi_driver/get_robot_info": {
          result: true,
          values: {
            robotName: "MOCK-NAO6",
            robotVersion: "6.0",
            bodyType: "nao",
          },
        },
        "/naoqi_driver/get_joint_names": {
          result: true,
          values: {
            joint_names: [
              "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll",
              "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
              "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
              "RWristYaw", "RHand", "LHipYawPitch", "LHipRoll",
              "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
              "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
              "RAnklePitch", "RAnkleRoll",
            ],
          },
        },
        "/naoqi_driver/get_position": {
          result: true,
          values: this.robotStatus.position,
        },
      };

      return mockResponses[service] || { result: true };
    }

    const id = `call_${this.messageId++}`;

    return new Promise((resolve, reject) => {
      const handleResponse = (message: RosMessage) => {
        if (message.op === "service_response" && message.id === id) {
          this.off("message", handleResponse);
          if (message.result === false) {
            resolve({
              result: false,
              error: String(message.values || "Service call failed"),
            });
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
        args: { state: desiredState },
      },
      {
        service: "/naoqi_driver/ALAutonomousLife/set_state",
        args: { state: desiredState },
      },
      // Direct module mapping
      {
        service: "/ALAutonomousLife/setState",
        args: { state: desiredState },
      },
      // Shortcuts/Aliases
      {
        service: "/naoqi_driver/set_autonomous_life",
        args: { state: desiredState },
      },
      {
        service: "/autonomous_life/set_state",
        args: { state: desiredState },
      },
      // Fallback: Enable/Disable topics/services
      {
        service: enabled ? "/life/enable" : "/life/disable",
        args: {},
      },
      // Last resort: Generic proxy call (if available)
      {
        service: "/naoqi_driver/function_call",
        args: {
          service: "ALAutonomousLife",
          function: "setState",
          args: [desiredState],
        },
      },
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
          console.warn(
            `[WizardROS] Service ${attempt.service} returned false result:`,
            response.error,
          );
        }
      } catch (error) {
        // Service call failed or timed out
        console.warn(
          `[WizardROS] Service ${attempt.service} failed/timeout:`,
          error,
        );
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

      case "transformToEmotionalSpeech":
        return this.transformToEmotionalSpeech(parameters);

      case "transformToWaveGoodbye":
        return this.transformToWaveGoodbye(parameters);

      case "transformToAnimation":
        return this.transformToAnimation(parameters);

      default:
        console.warn(`Unknown transform function: ${transformFn}`);
        return parameters;
    }
  }

  /**
   * Transform parameters for emotional speech
   * NAOqi markup: \rspd=<speed>\<text>
   * Using pure speech modifiers without animations to avoid sound effects
   */
  private transformToEmotionalSpeech(parameters: Record<string, unknown>): {
    data: string;
  } {
    const text = String(parameters.text || "Hello");
    const emotion = String(parameters.emotion || "neutral");
    const speed = Number(parameters.speed || 1.0);
    const speedPercent = Math.round(speed * 100);

    let markedText = text;

    switch (emotion) {
      case "happy":
        markedText = `\\\\rspd=120\\\\vct=100\\\\ ${text}`;
        break;
      case "excited":
        markedText = `\\\\rspd=140\\\\vct=110\\\\ ${text}`;
        break;
      case "sad":
        markedText = `\\\\rspd=80\\\\vct=80\\\\ ${text}`;
        break;
      case "calm":
        markedText = `\\\\rspd=90\\\\vct=90\\\\ ${text}`;
        break;
      case "neutral":
      default:
        markedText = `\\\\rspd=${speedPercent}\\\\vct=100\\\\ ${text}`;
        break;
    }

    return { data: markedText };
  }

  /**
   * Transform for wave goodbye - speech without animation sound
   */
  private transformToWaveGoodbye(parameters: Record<string, unknown>): {
    data: string;
  } {
    const text = String(parameters.text || "Goodbye!");
    const markedText = `\\\\rspd=110\\\\ ${text}`;
    return { data: markedText };
  }

  /**
   * Transform for playing animations
   */
  private transformToAnimation(parameters: Record<string, unknown>): {
    data: string;
  } {
    const animation = String(parameters.animation || "Hey_1");
    const markedText = `^start(animations/Stand/Gestures/${animation})`;
    return { data: markedText };
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
export function getWizardRosService(simulationMode?: boolean): WizardRosService {
  // Prevent multiple instances during creation
  if (isCreatingInstance && !wizardRosService) {
    throw new Error("WizardRosService is being initialized, please wait");
  }

  if (!wizardRosService) {
    isCreatingInstance = true;
    try {
      const url = typeof window !== "undefined" 
        ? (process.env.NEXT_PUBLIC_ROS_BRIDGE_URL || "ws://localhost:9090")
        : "ws://localhost:9090";
      wizardRosService = new WizardRosService(url, simulationMode);
    } finally {
      isCreatingInstance = false;
    }
  }
  return wizardRosService;
}

/**
 * Initialize wizard ROS service with connection
 */
export async function initWizardRosService(simulationMode?: boolean): Promise<WizardRosService> {
  const service = getWizardRosService(simulationMode);

  if (simulationMode !== undefined) {
    service.setSimulationMode(simulationMode);
  }

  if (!service.getConnectionStatus()) {
    await service.connect();
  }

  return service;
}

/**
 * Reset the global wizard ROS service (useful for testing or reinitializing)
 */
export function resetWizardRosService(): void {
  if (wizardRosService) {
    wizardRosService.disconnect();
    wizardRosService = null;
  }
}
