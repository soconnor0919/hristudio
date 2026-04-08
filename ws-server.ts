import { serve, type ServerWebSocket } from "bun";
import { wsManager } from "./src/server/services/websocket-manager";
import { db } from "./src/server/db";
import { wsConnections } from "./src/server/db/schema";
import { eq } from "drizzle-orm";

const port = parseInt(process.env.WS_PORT || "3001", 10);

interface WSData {
  clientId: string;
  trialId: string;
  userId: string | null;
}

function generateClientId(): string {
  return `ws_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
}

async function recordConnection(
  clientId: string,
  trialId: string,
  userId: string | null,
): Promise<void> {
  try {
    await db.insert(wsConnections).values({
      clientId,
      trialId,
      userId,
    });
    console.log(`[DB] Recorded connection for trial ${trialId}`);
  } catch (error) {
    console.error(`[DB] Failed to record connection:`, error);
  }
}

async function removeConnection(clientId: string): Promise<void> {
  try {
    await db.delete(wsConnections).where(eq(wsConnections.clientId, clientId));
    console.log(`[DB] Removed connection ${clientId}`);
  } catch (error) {
    console.error(`[DB] Failed to remove connection:`, error);
  }
}

console.log(`Starting WebSocket server on port ${port}...`);

serve<WSData>({
  port,
  async fetch(req, server) {
    const url = new URL(req.url);

    // Internal broadcast endpoint — called by Next.js tRPC router
    if (url.pathname === "/internal/broadcast") {
      if (req.method !== "POST") {
        return new Response("Method not allowed", { status: 405 });
      }
      const { trialId, message } = (await req.json()) as {
        trialId: string;
        message: { type: string; data: Record<string, unknown> };
      };
      await wsManager.broadcast(trialId, message);
      return new Response("OK", { status: 200 });
    }

    if (url.pathname === "/api/websocket") {
      if (req.headers.get("upgrade") !== "websocket") {
        return new Response("WebSocket upgrade required", { status: 426 });
      }

      const trialId = url.searchParams.get("trialId");
      const token = url.searchParams.get("token");

      if (!trialId) {
        return new Response("Missing trialId parameter", { status: 400 });
      }

      let userId: string | null = null;
      if (token) {
        try {
          const tokenData = JSON.parse(atob(token));
          userId = tokenData.userId;
        } catch {
          return new Response("Invalid token", { status: 401 });
        }
      }

      const clientId = generateClientId();
      const wsData: WSData = { clientId, trialId, userId };

      const upgraded = server.upgrade(req, { data: wsData });

      if (!upgraded) {
        return new Response("WebSocket upgrade failed", { status: 500 });
      }

      return;
    }

    return new Response("Not found", { status: 404 });
  },
  websocket: {
    async open(ws: ServerWebSocket<WSData>) {
      const { clientId, trialId, userId } = ws.data;

      wsManager.addClient(clientId, {
        socket: ws as unknown as WebSocket,
        trialId,
        userId,
        connectedAt: Date.now(),
      });

      await recordConnection(clientId, trialId, userId);

      console.log(
        `[WS] Client ${clientId} connected to trial ${trialId}. Total: ${wsManager.getConnectionCount()}`,
      );

      ws.send(
        JSON.stringify({
          type: "connection_established",
          data: {
            trialId,
            userId,
            role: "connected",
            connectedAt: Date.now(),
          },
        }),
      );
    },
    async message(ws: ServerWebSocket<WSData>, message) {
      const { clientId, trialId } = ws.data;

      try {
        const msg = JSON.parse(message.toString());

        switch (msg.type) {
          case "heartbeat":
            ws.send(
              JSON.stringify({
                type: "heartbeat_response",
                data: { timestamp: Date.now() },
              }),
            );
            break;

          case "request_trial_status": {
            const status = await wsManager.getTrialStatus(trialId);
            ws.send(
              JSON.stringify({
                type: "trial_status",
                data: {
                  trial: status?.trial ?? null,
                  current_step_index: status?.currentStepIndex ?? 0,
                  timestamp: Date.now(),
                },
              }),
            );
            break;
          }

          case "request_trial_events": {
            const events = await wsManager.getTrialEvents(
              trialId,
              msg.data?.limit ?? 100,
            );
            ws.send(
              JSON.stringify({
                type: "trial_events_snapshot",
                data: { events, timestamp: Date.now() },
              }),
            );
            break;
          }

          case "ping":
            ws.send(
              JSON.stringify({
                type: "pong",
                data: { timestamp: Date.now() },
              }),
            );
            break;

          default:
            console.log(
              `[WS] Unknown message type from ${clientId}:`,
              msg.type,
            );
        }
      } catch (error) {
        console.error(`[WS] Error processing message from ${clientId}:`, error);
      }
    },
    async close(ws: ServerWebSocket<WSData>) {
      const { clientId } = ws.data;
      console.log(`[WS] Client ${clientId} disconnected`);
      wsManager.removeClient(clientId);
      await removeConnection(clientId);
    },
  },
});

console.log(
  `> WebSocket server running on ws://localhost:${port}/api/websocket`,
);
