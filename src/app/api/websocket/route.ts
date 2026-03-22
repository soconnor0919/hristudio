import { NextRequest } from "next/server";
import { headers } from "next/headers";
import { wsManager } from "~/server/services/websocket-manager";
import { auth } from "~/lib/auth";

const clientConnections = new Map<
  string,
  { socket: WebSocket; clientId: string }
>();

function generateClientId(): string {
  return `ws_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
}

export const runtime = "edge";
export const dynamic = "force-dynamic";

export async function GET(request: NextRequest) {
  const url = new URL(request.url);
  const trialId = url.searchParams.get("trialId");
  const token = url.searchParams.get("token");

  if (!trialId) {
    return new Response("Missing trialId parameter", { status: 400 });
  }

  let userId: string | null = null;

  try {
    const session = await auth.api.getSession({
      headers: await headers(),
    });
    if (session?.user?.id) {
      userId = session.user.id;
    }
  } catch {
    if (!token) {
      return new Response("Authentication required", { status: 401 });
    }

    try {
      const tokenData = JSON.parse(atob(token));
      userId = tokenData.userId;
    } catch {
      return new Response("Invalid token", { status: 401 });
    }
  }

  const pair = new WebSocketPair();
  const clientId = generateClientId();
  const serverWebSocket = Object.values(pair)[0] as WebSocket;

  clientConnections.set(clientId, { socket: serverWebSocket, clientId });

  await wsManager.subscribe(clientId, serverWebSocket, trialId, userId);

  serverWebSocket.accept();

  serverWebSocket.addEventListener("message", async (event) => {
    try {
      const message = JSON.parse(event.data as string);

      switch (message.type) {
        case "heartbeat":
          wsManager.sendToClient(clientId, {
            type: "heartbeat_response",
            data: { timestamp: Date.now() },
          });
          break;

        case "request_trial_status": {
          const status = await wsManager.getTrialStatus(trialId);
          wsManager.sendToClient(clientId, {
            type: "trial_status",
            data: {
              trial: status?.trial ?? null,
              current_step_index: status?.currentStepIndex ?? 0,
              timestamp: Date.now(),
            },
          });
          break;
        }

        case "request_trial_events": {
          const events = await wsManager.getTrialEvents(
            trialId,
            message.data?.limit ?? 100,
          );
          wsManager.sendToClient(clientId, {
            type: "trial_events_snapshot",
            data: { events, timestamp: Date.now() },
          });
          break;
        }

        case "ping":
          wsManager.sendToClient(clientId, {
            type: "pong",
            data: { timestamp: Date.now() },
          });
          break;

        default:
          console.log(
            `[WS] Unknown message type from client ${clientId}:`,
            message.type,
          );
      }
    } catch (error) {
      console.error(`[WS] Error processing message from ${clientId}:`, error);
    }
  });

  serverWebSocket.addEventListener("close", () => {
    wsManager.unsubscribe(clientId);
    clientConnections.delete(clientId);
  });

  serverWebSocket.addEventListener("error", (error) => {
    console.error(`[WS] Error for client ${clientId}:`, error);
    wsManager.unsubscribe(clientId);
    clientConnections.delete(clientId);
  });

  return new Response(null, {
    status: 101,
    webSocket: serverWebSocket,
  } as ResponseInit);
}

declare global {
  interface WebSocket {
    accept(): void;
  }
}
