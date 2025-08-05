import { eq } from "drizzle-orm";
import { type NextRequest } from "next/server";
import { type WebSocketServer } from "ws";
import { auth } from "~/server/auth";
import { db } from "~/server/db";
import { trialEvents, trials } from "~/server/db/schema";

// Store active WebSocket connections
const connections = new Map<string, Set<any>>();
const userConnections = new Map<
  string,
  { userId: string; trialId: string; role: string }
>();

// Create WebSocket server instance
const wss: WebSocketServer | null = null;

export const runtime = "nodejs";

export async function GET(request: NextRequest) {
  const url = new URL(request.url);
  const trialId = url.searchParams.get("trialId");
  const token = url.searchParams.get("token");

  if (!trialId) {
    return new Response("Missing trialId parameter", { status: 400 });
  }

  if (!token) {
    return new Response("Missing authentication token", { status: 401 });
  }

  // For WebSocket upgrade, we need to handle this differently in Next.js
  // This is a simplified version - in production you'd use a separate WebSocket server

  return new Response(
    JSON.stringify({
      message: "WebSocket endpoint available",
      trialId,
      endpoint: `/api/websocket?trialId=${trialId}&token=${token}`,
      instructions: "Use WebSocket client to connect to this endpoint",
    }),
    {
      status: 200,
      headers: {
        "Content-Type": "application/json",
      },
    },
  );
}

// WebSocket connection handler (for external WebSocket server)
export async function handleWebSocketConnection(ws: any, request: any) {
  try {
    const url = new URL(request.url, `http://${request.headers.host}`);
    const trialId = url.searchParams.get("trialId");
    const token = url.searchParams.get("token");

    if (!trialId || !token) {
      ws.close(1008, "Missing required parameters");
      return;
    }

    // Verify authentication
    const session = await auth();
    if (!session?.user) {
      ws.close(1008, "Unauthorized");
      return;
    }

    // Verify trial access
    const trial = await db
      .select()
      .from(trials)
      .where(eq(trials.id, trialId))
      .limit(1);

    if (!trial.length) {
      ws.close(1008, "Trial not found");
      return;
    }

    const userRole = session.user.roles?.[0]?.role;
    if (
      !userRole ||
      !["administrator", "researcher", "wizard", "observer"].includes(userRole)
    ) {
      ws.close(1008, "Insufficient permissions");
      return;
    }

    const connectionId = crypto.randomUUID();
    const userId = session.user.id;

    // Store connection info
    userConnections.set(connectionId, {
      userId,
      trialId,
      role: userRole,
    });

    // Add to trial connections
    if (!connections.has(trialId)) {
      connections.set(trialId, new Set());
    }
    connections.get(trialId)!.add(ws);

    // Send initial connection confirmation
    ws.send(
      JSON.stringify({
        type: "connection_established",
        data: {
          connectionId,
          trialId,
          timestamp: new Date().toISOString(),
        },
      }),
    );

    // Send current trial status
    await sendTrialStatus(ws, trialId);

    ws.on("message", async (data: Buffer) => {
      try {
        const message = JSON.parse(data.toString());
        await handleWebSocketMessage(ws, connectionId, message);
      } catch (error) {
        console.error("Error handling WebSocket message:", error);
        ws.send(
          JSON.stringify({
            type: "error",
            data: {
              message: "Invalid message format",
              timestamp: new Date().toISOString(),
            },
          }),
        );
      }
    });

    ws.on("close", () => {
      console.log(`WebSocket disconnected: ${connectionId}`);

      // Clean up connections
      const connectionInfo = userConnections.get(connectionId);
      if (connectionInfo) {
        const trialConnections = connections.get(connectionInfo.trialId);
        if (trialConnections) {
          trialConnections.delete(ws);
          if (trialConnections.size === 0) {
            connections.delete(connectionInfo.trialId);
          }
        }
        userConnections.delete(connectionId);
      }
    });

    ws.on("error", (error: Error) => {
      console.error(`WebSocket error for ${connectionId}:`, error);
    });

    console.log(`WebSocket connected: ${connectionId} for trial ${trialId}`);
  } catch (error) {
    console.error("WebSocket setup error:", error);
    ws.close(1011, "Internal server error");
  }
}

async function handleWebSocketMessage(
  ws: any,
  connectionId: string,
  message: any,
) {
  const connectionInfo = userConnections.get(connectionId);
  if (!connectionInfo) {
    return;
  }

  const { userId, trialId, role } = connectionInfo;

  switch (message.type) {
    case "trial_action":
      if (["wizard", "researcher", "administrator"].includes(role)) {
        await handleTrialAction(trialId, userId, message.data);
        broadcastToTrial(trialId, {
          type: "trial_action_executed",
          data: {
            action: message.data,
            executedBy: userId,
            timestamp: new Date().toISOString(),
          },
        });
      }
      break;

    case "step_transition":
      if (["wizard", "researcher", "administrator"].includes(role)) {
        await handleStepTransition(trialId, userId, message.data);
        broadcastToTrial(trialId, {
          type: "step_changed",
          data: {
            ...message.data,
            changedBy: userId,
            timestamp: new Date().toISOString(),
          },
        });
      }
      break;

    case "wizard_intervention":
      if (["wizard", "researcher", "administrator"].includes(role)) {
        await logTrialEvent(
          trialId,
          "wizard_intervention",
          message.data,
          userId,
        );
        broadcastToTrial(trialId, {
          type: "intervention_logged",
          data: {
            ...message.data,
            interventionBy: userId,
            timestamp: new Date().toISOString(),
          },
        });
      }
      break;

    case "request_trial_status":
      await sendTrialStatus(ws, trialId);
      break;

    case "heartbeat":
      ws.send(
        JSON.stringify({
          type: "heartbeat_response",
          data: {
            timestamp: new Date().toISOString(),
          },
        }),
      );
      break;

    default:
      ws.send(
        JSON.stringify({
          type: "error",
          data: {
            message: `Unknown message type: ${message.type}`,
            timestamp: new Date().toISOString(),
          },
        }),
      );
  }
}

async function handleTrialAction(
  trialId: string,
  userId: string,
  actionData: any,
) {
  try {
    // Log the action as a trial event
    await logTrialEvent(trialId, "wizard_action", actionData, userId);

    // Update trial status if needed
    if (actionData.actionType === "start_trial") {
      await db
        .update(trials)
        .set({
          status: "in_progress",
          startedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(trials.id, trialId));
    } else if (actionData.actionType === "complete_trial") {
      await db
        .update(trials)
        .set({
          status: "completed",
          completedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(trials.id, trialId));
    } else if (actionData.actionType === "abort_trial") {
      await db
        .update(trials)
        .set({
          status: "aborted",
          completedAt: new Date(),
          updatedAt: new Date(),
        })
        .where(eq(trials.id, trialId));
    }
  } catch (error) {
    console.error("Error handling trial action:", error);
    throw error;
  }
}

async function handleStepTransition(
  trialId: string,
  userId: string,
  stepData: any,
) {
  try {
    await logTrialEvent(trialId, "step_transition", stepData, userId);
  } catch (error) {
    console.error("Error handling step transition:", error);
    throw error;
  }
}

async function logTrialEvent(
  trialId: string,
  eventType: string,
  data: any,
  userId: string,
) {
  try {
    await db.insert(trialEvents).values({
      trialId,
      eventType: eventType as "trial_start" | "trial_end" | "step_start" | "step_end" | "wizard_intervention" | "error" | "custom",
      data,
      createdBy: userId,
      timestamp: new Date(),
    });
  } catch (error) {
    console.error("Error logging trial event:", error);
    throw error;
  }
}

async function sendTrialStatus(ws: any, trialId: string) {
  try {
    const trial = await db
      .select()
      .from(trials)
      .where(eq(trials.id, trialId))
      .limit(1);

    if (trial.length > 0) {
      ws.send(
        JSON.stringify({
          type: "trial_status",
          data: {
            trial: trial[0],
            timestamp: new Date().toISOString(),
          },
        }),
      );
    }
  } catch (error) {
    console.error("Error sending trial status:", error);
  }
}

function broadcastToTrial(trialId: string, message: any) {
  const trialConnections = connections.get(trialId);
  if (trialConnections) {
    const messageStr = JSON.stringify(message);
    for (const ws of trialConnections) {
      if (ws.readyState === 1) {
        // WebSocket.OPEN
        ws.send(messageStr);
      }
    }
  }
}

// Utility function to broadcast trial updates
export function broadcastTrialUpdate(
  trialId: string,
  updateType: string,
  data: any,
) {
  broadcastToTrial(trialId, {
    type: updateType,
    data: {
      ...data,
      timestamp: new Date().toISOString(),
    },
  });
}

// Cleanup orphaned connections
setInterval(() => {
  for (const [connectionId, info] of userConnections.entries()) {
    const trialConnections = connections.get(info.trialId);
    if (!trialConnections || trialConnections.size === 0) {
      userConnections.delete(connectionId);
    }
  }
}, 30000);
