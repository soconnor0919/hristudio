import { WebSocketServer, WebSocket } from "ws";

interface RosMessage {
  op: string;
  topic?: string;
  type?: string;
  id?: string;
  msg?: Record<string, unknown>;
  service?: string;
  args?: Record<string, unknown>;
}

interface Subscriber {
  id: string;
  topic: string;
  type: string;
  ws: WebSocket;
}

const PORT = parseInt(process.env.MOCK_ROBOT_PORT || "9090", 10);
const PUBLISH_INTERVAL = parseInt(process.env.MOCK_PUBLISH_INTERVAL || "100", 10);

const subscribers: Map<string, Subscriber> = new Map();
let subscriberIdCounter = 0;

const mockRobotState = {
  battery: 85,
  position: { x: 0, y: 0, theta: 0 },
  joints: [
    "HeadYaw",
    "HeadPitch",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "LHand",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw",
    "RHand",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
  ],
  jointPositions: new Array(26).fill(0).map(() => (Math.random() - 0.5) * 0.1),
  bumperLeft: false,
  bumperRight: false,
  handTouchLeft: false,
  handTouchRight: false,
  headTouchFront: false,
  headTouchMiddle: false,
  headTouchRear: false,
  sonarLeft: 0.5 + Math.random() * 0.5,
  sonarRight: 0.5 + Math.random() * 0.5,
  lastSpeechText: "",
};

function broadcastToSubscribers(topic: string, msg: Record<string, unknown>, type: string): void {
  const message = JSON.stringify({
    op: "publish",
    topic,
    type,
    msg,
  });

  subscribers.forEach((sub) => {
    if (sub.topic === topic && sub.ws.readyState === WebSocket.OPEN) {
      try {
        sub.ws.send(message);
      } catch (e) {
        console.error(`Failed to send to subscriber ${sub.id}:`, e);
      }
    }
  });
}

function publishRobotState(): void {
  broadcastToSubscribers(
    "/joint_states",
    {
      header: { stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 }, frame_id: "" },
      name: mockRobotState.joints,
      position: mockRobotState.jointPositions,
      velocity: new Array(26).fill(0),
      effort: new Array(26).fill(0),
    },
    "sensor_msgs/JointState"
  );

  broadcastToSubscribers(
    "/naoqi_driver/battery",
    { header: {}, percentage: mockRobotState.battery, charging: false, plug: false },
    "naoqi_bridge_msgs/Bumper"
  );

  broadcastToSubscribers(
    "/bumper",
    { left: mockRobotState.bumperLeft, right: mockRobotState.bumperRight },
    "naoqi_bridge_msgs/Bumper"
  );

  broadcastToSubscribers(
    "/hand_touch",
    {
      leftHand: mockRobotState.handTouchLeft,
      rightHand: mockRobotState.handTouchRight,
    },
    "naoqi_bridge_msgs/HandTouch"
  );

  broadcastToSubscribers(
    "/head_touch",
    {
      front: mockRobotState.headTouchFront,
      middle: mockRobotState.headTouchMiddle,
      rear: mockRobotState.headTouchRear,
    },
    "naoqi_bridge_msgs/HeadTouch"
  );

  broadcastToSubscribers(
    "/sonar/left",
    { header: {}, radiation_type: 1, field_of_view: 0.5, min_range: 0.1, max_range: 5.0, range: mockRobotState.sonarLeft },
    "sensor_msgs/Range"
  );

  broadcastToSubscribers(
    "/sonar/right",
    { header: {}, radiation_type: 1, field_of_view: 0.5, min_range: 0.1, max_range: 5.0, range: mockRobotState.sonarRight },
    "sensor_msgs/Range"
  );
}

function handleMessage(ws: WebSocket, data: string): void {
  try {
    const message: RosMessage = JSON.parse(data);
    console.log(`[MockRobot] Received: ${message.op} ${message.topic || message.service || ""}`);

    switch (message.op) {
      case "subscribe":
        handleSubscribe(ws, message);
        break;

      case "unsubscribe":
        handleUnsubscribe(message);
        break;

      case "publish":
        handlePublish(message);
        break;

      case "call_service":
        handleServiceCall(ws, message);
        break;

      case "advertise":
        console.log(`[MockRobot] Client advertising: ${message.topic}`);
        break;

      case "unadvertise":
        console.log(`[MockRobot] Client unadvertising: ${message.topic}`);
        break;

      case "auth":
        ws.send(JSON.stringify({ op: "auth_result", result: true }));
        break;

      default:
        console.log(`[MockRobot] Unknown operation: ${message.op}`);
    }
  } catch (e) {
    console.error("[MockRobot] Failed to parse message:", e);
  }
}

function handleSubscribe(ws: WebSocket, message: RosMessage): void {
  if (!message.topic) return;

  const id = `sub_${subscriberIdCounter++}`;
  const subscriber: Subscriber = {
    id,
    topic: message.topic,
    type: message.type || "unknown",
    ws,
  };

  subscribers.set(id, subscriber);
  console.log(`[MockRobot] Subscribed to ${message.topic} (${id})`);

  if (message.id) {
    ws.send(JSON.stringify({ op: "subscribe", id: message.id, values: true }));
  }
}

function handleUnsubscribe(message: RosMessage): void {
  if (!message.id) return;

  const subscriber = subscribers.get(message.id);
  if (subscriber) {
    console.log(`[MockRobot] Unsubscribed from ${subscriber.topic}`);
    subscribers.delete(message.id);
  }
}

function handlePublish(message: RosMessage): void {
  if (!message.topic || !message.msg) return;

  console.log(`[MockRobot] Publish to ${message.topic}:`, JSON.stringify(message.msg).slice(0, 200));

  if (message.topic === "/cmd_vel") {
    handleCmdVel(message.msg);
  } else if (message.topic === "/speech") {
    handleSpeech(message.msg);
  } else if (message.topic === "/joint_angles") {
    handleJointAngles(message.msg);
  } else if (message.topic === "/autonomous_life/control") {
    handleAutonomousLife(message.msg);
  } else if (message.topic === "/leds") {
    handleLEDs(message.msg);
  }
}

function handleCmdVel(msg: Record<string, unknown>): void {
  const twist = msg as { linear?: { x?: number; y?: number; z?: number }; angular?: { x?: number; y?: number; z?: number } };
  const linear = twist.linear || {};
  const angular = twist.angular || {};

  if (angular.z !== undefined && angular.z !== 0) {
    mockRobotState.position.theta += angular.z * (PUBLISH_INTERVAL / 1000);
    console.log(`[MockRobot] Turning: angular.z=${angular.z}, new theta=${mockRobotState.position.theta.toFixed(2)}`);
  }

  if (linear.x !== undefined && linear.x !== 0) {
    const dx = linear.x * Math.cos(mockRobotState.position.theta) * (PUBLISH_INTERVAL / 1000);
    const dy = linear.x * Math.sin(mockRobotState.position.theta) * (PUBLISH_INTERVAL / 1000);
    mockRobotState.position.x += dx;
    mockRobotState.position.y += dy;
    console.log(`[MockRobot] Walking: linear.x=${linear.x}, pos=(${mockRobotState.position.x.toFixed(2)}, ${mockRobotState.position.y.toFixed(2)})`);
  }
}

function handleSpeech(msg: Record<string, unknown>): void {
  const text = (msg as { data?: string }).data || "";
  mockRobotState.lastSpeechText = text;
  console.log(`[MockRobot] Speaking: "${text}"`);

  setTimeout(() => {
    broadcastToSubscribers(
      "/speech/status",
      { state: "done", text },
      "std_msgs/String"
    );
    console.log(`[MockRobot] Speech complete: "${text}"`);
  }, Math.max(500, text.split(/\s+/).length * 300 + 1500));
}

function handleJointAngles(msg: Record<string, unknown>): void {
  const data = msg as {
    joint_names?: string[];
    joint_angles?: number[];
    speed?: number;
  };

  if (data.joint_names && data.joint_angles && Array.isArray(data.joint_angles)) {
    const jointAngles = data.joint_angles;
    data.joint_names.forEach((name, i) => {
      const idx = mockRobotState.joints.indexOf(name);
      const angle = jointAngles[i];
      if (idx >= 0 && angle !== undefined) {
        mockRobotState.jointPositions[idx] = angle;
      }
    });
    console.log(`[MockRobot] Joint angles updated: ${data.joint_names.join(", ")}`);
  }
}

function handleAutonomousLife(msg: Record<string, unknown>): void {
  const state = (msg as { data?: string }).data || "disabled";
  console.log(`[MockRobot] Autonomous life: ${state}`);
}

function handleLEDs(msg: Record<string, unknown>): void {
  const ledName = (msg as { name?: string }).name || "unknown";
  const color = (msg as { color?: string }).color || "unknown";
  console.log(`[MockRobot] LED ${ledName} set to ${color}`);
}

function handleServiceCall(ws: WebSocket, message: RosMessage): void {
  const service = message.service || "";
  const id = message.id || `svc_${Date.now()}`;
  const args = message.args || {};

  console.log(`[MockRobot] Service call: ${service}`, args);

  let response: Record<string, unknown> = {};

  switch (service) {
    case "/rosapi/get_param":
      response = { value: args.param || "" };
      break;

    case "/rosapi/topics_for_type":
      response = { topics: [] };
      break;

    case "/rosapi/get_topic_type":
      response = { type: "" };
      break;

    case "/rosapi/get_node_details":
      response = { node_api: "", publications: [], subscriptions: [], services: [] };
      break;

    case "/naoqi_driver/get_robot_info":
      response = {
        robotName: "MOCK-NAO6",
        robotVersion: "6.0",
        bodyType: "nao",
        headTiltAngle: 0,
        time: Math.floor(Date.now() / 1000),
      };
      break;

    case "/naoqi_driver/get_joint_names":
      response = { joint_names: mockRobotState.joints };
      break;

    case "/naoqi_driver/get_position":
      response = {
        x: mockRobotState.position.x,
        y: mockRobotState.position.y,
        theta: mockRobotState.position.theta,
      };
      break;

    case "/naoqi_driver/is_waking_up":
      response = { success: true, is_waking_up: false, is_webots: false };
      break;

    case "/naoqi_driver/robot_supports":
      response = { supports_service: true };
      break;

    case "/naoqi_driver/set_autonomous_state":
      response = { success: true };
      break;

    case "/naoqi_driver/toggle_autonomous":
      response = { success: true };
      break;

    case "/naoqi_driver/call_button_action":
      response = { success: true, button_id: (args as { button_id?: string }).button_id };
      break;

    case "/naoqi_driver/robot_batch_request":
      response = { success: true };
      break;

    default:
      console.log(`[MockRobot] Unknown service: ${service}`);
      response = { success: true };
  }

  ws.send(JSON.stringify({
    op: "service_response",
    id,
    service,
    result: true,
    values: response,
  }));
}

const wss = new WebSocketServer({ port: PORT });

console.log(`[MockRobot] Mock Robot Server starting on ws://localhost:${PORT}`);
console.log(`[MockRobot] Publish interval: ${PUBLISH_INTERVAL}ms`);
console.log("[MockRobot] Simulating NAO6 robot with rosbridge protocol\n");

wss.on("connection", (ws: WebSocket) => {
  console.log("[MockRobot] Client connected");

  ws.on("message", (data: Buffer) => {
    handleMessage(ws, data.toString());
  });

  ws.on("close", () => {
    console.log("[MockRobot] Client disconnected");
  });

  ws.on("error", (error) => {
    console.error("[MockRobot] WebSocket error:", error);
  });

  ws.send(JSON.stringify({ op: "connected", id: "mock_robot_server" }));
});

setInterval(publishRobotState, PUBLISH_INTERVAL);

console.log(`[MockRobot] Server ready. Connect via WebSocket to ws://localhost:${PORT}`);
