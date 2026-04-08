import { db } from "~/server/db";
import {
  trials,
  trialEvents,
  wsConnections,
  experiments,
} from "~/server/db/schema";
import { eq, sql } from "drizzle-orm";

interface ClientConnection {
  socket: WebSocket;
  trialId: string;
  userId: string | null;
  connectedAt: number;
}

type OutgoingMessage = {
  type: string;
  data: Record<string, unknown>;
};

class WebSocketManager {
  private clients: Map<string, ClientConnection> = new Map();
  private heartbeatIntervals: Map<string, ReturnType<typeof setInterval>> =
    new Map();

  private getTrialRoomClients(trialId: string): ClientConnection[] {
    const clients: ClientConnection[] = [];
    for (const [, client] of this.clients) {
      if (client.trialId === trialId) {
        clients.push(client);
      }
    }
    return clients;
  }

  addClient(clientId: string, connection: ClientConnection): void {
    this.clients.set(clientId, connection);
    console.log(
      `[WS] Client ${clientId} added for trial ${connection.trialId}. Total: ${this.clients.size}`,
    );
  }

  removeClient(clientId: string): void {
    const client = this.clients.get(clientId);
    if (client) {
      console.log(
        `[WS] Client ${clientId} removed from trial ${client.trialId}`,
      );
    }

    const heartbeatInterval = this.heartbeatIntervals.get(clientId);
    if (heartbeatInterval) {
      clearInterval(heartbeatInterval);
      this.heartbeatIntervals.delete(clientId);
    }

    this.clients.delete(clientId);
  }

  async subscribe(
    clientId: string,
    socket: WebSocket,
    trialId: string,
    userId: string | null,
  ): Promise<void> {
    const client: ClientConnection = {
      socket,
      trialId,
      userId,
      connectedAt: Date.now(),
    };

    this.clients.set(clientId, client);

    const heartbeatInterval = setInterval(() => {
      this.sendToClient(clientId, { type: "heartbeat", data: {} });
    }, 30000);

    this.heartbeatIntervals.set(clientId, heartbeatInterval);

    console.log(
      `[WS] Client ${clientId} subscribed to trial ${trialId}. Total clients: ${this.clients.size}`,
    );
  }

  unsubscribe(clientId: string): void {
    const client = this.clients.get(clientId);
    if (client) {
      console.log(
        `[WS] Client ${clientId} unsubscribed from trial ${client.trialId}`,
      );
    }

    const heartbeatInterval = this.heartbeatIntervals.get(clientId);
    if (heartbeatInterval) {
      clearInterval(heartbeatInterval);
      this.heartbeatIntervals.delete(clientId);
    }

    this.clients.delete(clientId);
  }

  sendToClient(clientId: string, message: OutgoingMessage): void {
    const client = this.clients.get(clientId);
    if (client?.socket.readyState === 1) {
      try {
        client.socket.send(JSON.stringify(message));
      } catch (error) {
        console.error(`[WS] Error sending to client ${clientId}:`, error);
        this.unsubscribe(clientId);
      }
    }
  }

  async broadcast(trialId: string, message: OutgoingMessage): Promise<void> {
    const clients = this.getTrialRoomClients(trialId);

    if (clients.length === 0) {
      return;
    }

    const messageStr = JSON.stringify(message);
    const disconnectedClients: string[] = [];

    for (const [clientId, client] of this.clients) {
      if (client.trialId === trialId && client.socket.readyState === 1) {
        try {
          client.socket.send(messageStr);
        } catch (error) {
          console.error(
            `[WS] Error broadcasting to client ${clientId}:`,
            error,
          );
          disconnectedClients.push(clientId);
        }
      }
    }

    for (const clientId of disconnectedClients) {
      this.unsubscribe(clientId);
    }

    console.log(
      `[WS] Broadcast to ${clients.length} clients for trial ${trialId}: ${message.type}`,
    );
  }

  // Called from Next.js tRPC router — POSTs to the Bun ws-server process
  // which holds the actual client connections.
  async broadcastExternal(
    trialId: string,
    message: OutgoingMessage,
  ): Promise<void> {
    const wsPort = process.env.WS_PORT ?? "3001";
    try {
      await fetch(`http://localhost:${wsPort}/internal/broadcast`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ trialId, message }),
      });
    } catch (error) {
      console.error(`[WS] Failed to broadcast externally for trial ${trialId}:`, error);
    }
  }

  async broadcastToAll(message: OutgoingMessage): Promise<void> {
    const messageStr = JSON.stringify(message);
    const disconnectedClients: string[] = [];

    for (const [clientId, client] of this.clients) {
      if (client.socket.readyState === 1) {
        try {
          client.socket.send(messageStr);
        } catch (error) {
          console.error(
            `[WS] Error broadcasting to client ${clientId}:`,
            error,
          );
          disconnectedClients.push(clientId);
        }
      }
    }

    for (const clientId of disconnectedClients) {
      this.unsubscribe(clientId);
    }
  }

  async getTrialStatus(trialId: string): Promise<{
    trial: {
      id: string;
      status: string;
      startedAt: Date | null;
      completedAt: Date | null;
    };
    currentStepIndex: number;
  } | null> {
    const [trial] = await db
      .select({
        id: trials.id,
        status: trials.status,
        startedAt: trials.startedAt,
        completedAt: trials.completedAt,
      })
      .from(trials)
      .where(eq(trials.id, trialId))
      .limit(1);

    if (!trial) {
      return null;
    }

    return {
      trial: {
        id: trial.id,
        status: trial.status,
        startedAt: trial.startedAt,
        completedAt: trial.completedAt,
      },
      currentStepIndex: 0,
    };
  }

  async getTrialEvents(
    trialId: string,
    limit: number = 100,
  ): Promise<unknown[]> {
    const events = await db
      .select()
      .from(trialEvents)
      .where(eq(trialEvents.trialId, trialId))
      .orderBy(trialEvents.timestamp)
      .limit(limit);

    return events;
  }

  getTrialStatusSync(trialId: string): {
    trial: {
      id: string;
      status: string;
      startedAt: Date | null;
      completedAt: Date | null;
    };
    currentStepIndex: number;
  } | null {
    return null;
  }

  getTrialEventsSync(trialId: string, limit: number = 100): unknown[] {
    return [];
  }

  getConnectionCount(trialId?: string): number {
    if (trialId) {
      return this.getTrialRoomClients(trialId).length;
    }
    return this.clients.size;
  }

  getConnectedTrialIds(): string[] {
    const trialIds = new Set<string>();
    for (const [, client] of this.clients) {
      trialIds.add(client.trialId);
    }
    return Array.from(trialIds);
  }

  async getTrialsWithActiveConnections(studyIds?: string[]): Promise<string[]> {
    const conditions =
      studyIds && studyIds.length > 0
        ? sql`${wsConnections.trialId} IN (
          SELECT ${trials.id} FROM ${trials} 
          WHERE ${trials.experimentId} IN (
            SELECT ${experiments.id} FROM ${experiments} 
            WHERE ${experiments.studyId} IN (${sql.raw(studyIds.map((id) => `'${id}'`).join(","))})
          )
        )`
        : undefined;

    const connections = await db
      .selectDistinct({ trialId: wsConnections.trialId })
      .from(wsConnections);

    return connections.map((c) => c.trialId);
  }
}

export const wsManager = new WebSocketManager();
