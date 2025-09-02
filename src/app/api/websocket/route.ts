export const runtime = "edge";

declare global {
  var WebSocketPair: new () => { 0: WebSocket; 1: WebSocket };

  interface WebSocket {
    accept(): void;
  }

  interface ResponseInit {
    webSocket?: WebSocket;
  }
}

type Json = Record<string, unknown>;

interface ClientInfo {
  userId: string | null;
  role: "wizard" | "researcher" | "administrator" | "observer" | "unknown";
  connectedAt: number;
}

interface TrialState {
  trial: {
    id: string;
    status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
    startedAt: string | null;
    completedAt: string | null;
  };
  currentStepIndex: number;
  updatedAt: number;
}

declare global {
  // Per-trial subscriber sets
  // Using globalThis for ephemeral in-memory broadcast in the current Edge isolate
  // (not shared globally across regions/instances)
  var __trialRooms: Map<string, Set<WebSocket>> | undefined;
  var __trialState: Map<string, TrialState> | undefined;
}

const rooms = (globalThis.__trialRooms ??= new Map<string, Set<WebSocket>>());
const states = (globalThis.__trialState ??= new Map<string, TrialState>());

function safeJSON<T>(v: T): string {
  try {
    return JSON.stringify(v);
  } catch {
    return '{"type":"error","data":{"message":"serialization_error"}}';
  }
}

function send(ws: WebSocket, message: { type: string; data?: Json }) {
  try {
    ws.send(safeJSON(message));
  } catch {
    // swallow send errors
  }
}

function broadcast(trialId: string, message: { type: string; data?: Json }) {
  const room = rooms.get(trialId);
  if (!room) return;
  const payload = safeJSON(message);
  for (const client of room) {
    try {
      client.send(payload);
    } catch {
      // ignore individual client send failure
    }
  }
}

function ensureTrialState(trialId: string): TrialState {
  let state = states.get(trialId);
  if (!state) {
    state = {
      trial: {
        id: trialId,
        status: "scheduled",
        startedAt: null,
        completedAt: null,
      },
      currentStepIndex: 0,
      updatedAt: Date.now(),
    };
    states.set(trialId, state);
  }
  return state;
}

function updateTrialStatus(
  trialId: string,
  patch: Partial<TrialState["trial"]> &
    Partial<Pick<TrialState, "currentStepIndex">>,
) {
  const state = ensureTrialState(trialId);
  if (typeof patch.currentStepIndex === "number") {
    state.currentStepIndex = patch.currentStepIndex;
  }
  state.trial = {
    ...state.trial,
    ...(patch.status !== undefined ? { status: patch.status } : {}),
    ...(patch.startedAt !== undefined
      ? { startedAt: patch.startedAt ?? null }
      : {}),
    ...(patch.completedAt !== undefined
      ? { completedAt: patch.completedAt ?? null }
      : {}),
  };
  state.updatedAt = Date.now();
  states.set(trialId, state);
  return state;
}

// Very lightweight token parse (base64-encoded JSON per client hook)
// In production, replace with properly signed JWT verification.
function parseToken(token: string | null): ClientInfo {
  if (!token) {
    return { userId: null, role: "unknown", connectedAt: Date.now() };
  }
  try {
    const decodedUnknown = JSON.parse(atob(token)) as unknown;
    const userId =
      typeof decodedUnknown === "object" &&
      decodedUnknown !== null &&
      "userId" in decodedUnknown &&
      typeof (decodedUnknown as Record<string, unknown>).userId === "string"
        ? ((decodedUnknown as Record<string, unknown>).userId as string)
        : null;

    const connectedAt = Date.now();
    const role: ClientInfo["role"] = "wizard"; // default role for live trial control context

    return { userId, role, connectedAt };
  } catch {
    return { userId: null, role: "unknown", connectedAt: Date.now() };
  }
}

export async function GET(req: Request): Promise<Response> {
  const { searchParams } = new URL(req.url);
  const trialId = searchParams.get("trialId");
  const token = searchParams.get("token");

  if (!trialId) {
    return new Response("Missing trialId parameter", { status: 400 });
  }

  // If this isn't a WebSocket upgrade, return a small JSON descriptor
  const upgrade = req.headers.get("upgrade") ?? "";
  if (upgrade.toLowerCase() !== "websocket") {
    return new Response(
      safeJSON({
        message: "WebSocket endpoint",
        trialId,
        info: "Open a WebSocket connection to this URL to receive live trial updates.",
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  }

  // Create WebSocket pair (typed) and destructure endpoints
  const pair = new WebSocketPair();
  const client = pair[0];
  const server = pair[1];

  // Register server-side handlers
  server.accept();

  const clientInfo = parseToken(token);

  // Join room
  const room = rooms.get(trialId) ?? new Set<WebSocket>();
  room.add(server);
  rooms.set(trialId, room);

  // Immediately acknowledge connection and provide current trial status snapshot
  const state = ensureTrialState(trialId);

  send(server, {
    type: "connection_established",
    data: {
      trialId,
      userId: clientInfo.userId,
      role: clientInfo.role,
      connectedAt: clientInfo.connectedAt,
    },
  });

  send(server, {
    type: "trial_status",
    data: {
      trial: state.trial,
      current_step_index: state.currentStepIndex,
      timestamp: Date.now(),
    },
  });

  server.addEventListener("message", (ev: MessageEvent<string>) => {
    let parsed: unknown;
    try {
      parsed = JSON.parse(typeof ev.data === "string" ? ev.data : "{}");
    } catch {
      send(server, {
        type: "error",
        data: { message: "invalid_json" },
      });
      return;
    }

    const maybeObj =
      typeof parsed === "object" && parsed !== null
        ? (parsed as Record<string, unknown>)
        : {};
    const type = typeof maybeObj.type === "string" ? maybeObj.type : "";
    const data: Json =
      maybeObj.data &&
      typeof maybeObj.data === "object" &&
      maybeObj.data !== null
        ? (maybeObj.data as Record<string, unknown>)
        : {};
    const now = Date.now();

    const getString = (key: string, fallback = ""): string => {
      const v = (data as Record<string, unknown>)[key];
      return typeof v === "string" ? v : fallback;
    };
    const getNumber = (key: string): number | undefined => {
      const v = (data as Record<string, unknown>)[key];
      return typeof v === "number" ? v : undefined;
    };

    switch (type) {
      case "heartbeat": {
        send(server, { type: "heartbeat_response", data: { timestamp: now } });
        break;
      }

      case "request_trial_status": {
        const s = ensureTrialState(trialId);
        send(server, {
          type: "trial_status",
          data: {
            trial: s.trial,
            current_step_index: s.currentStepIndex,
            timestamp: now,
          },
        });
        break;
      }

      case "trial_action": {
        // Supports: start_trial, complete_trial, abort_trial, and generic actions
        const actionType = getString("actionType", "unknown");
        let updated: TrialState | null = null;

        if (actionType === "start_trial") {
          const stepIdx = getNumber("step_index") ?? 0;
          updated = updateTrialStatus(trialId, {
            status: "in_progress",
            startedAt: new Date().toISOString(),
            currentStepIndex: stepIdx,
          });
        } else if (actionType === "complete_trial") {
          updated = updateTrialStatus(trialId, {
            status: "completed",
            completedAt: new Date().toISOString(),
          });
        } else if (actionType === "abort_trial") {
          updated = updateTrialStatus(trialId, {
            status: "aborted",
            completedAt: new Date().toISOString(),
          });
        }

        // Broadcast the action execution event
        broadcast(trialId, {
          type: "trial_action_executed",
          data: {
            action_type: actionType,
            timestamp: now,
            userId: clientInfo.userId,
            ...data,
          },
        });

        // If trial state changed, broadcast status
        if (updated) {
          broadcast(trialId, {
            type: "trial_status",
            data: {
              trial: updated.trial,
              current_step_index: updated.currentStepIndex,
              timestamp: now,
            },
          });
        }
        break;
      }

      case "wizard_intervention": {
        // Log/broadcast a wizard intervention (note, correction, manual control)
        broadcast(trialId, {
          type: "intervention_logged",
          data: {
            timestamp: now,
            userId: clientInfo.userId,
            ...data,
          },
        });
        break;
      }

      case "step_transition": {
        // Update step index and broadcast
        const from = getNumber("from_step");
        const to = getNumber("to_step");

        if (typeof to !== "number" || !Number.isFinite(to)) {
          send(server, {
            type: "error",
            data: { message: "invalid_step_transition" },
          });
          return;
        }

        const updated = updateTrialStatus(trialId, {
          currentStepIndex: to,
        });

        broadcast(trialId, {
          type: "step_changed",
          data: {
            timestamp: now,
            userId: clientInfo.userId,
            from_step:
              typeof from === "number" ? from : updated.currentStepIndex,
            to_step: updated.currentStepIndex,
            ...data,
          },
        });
        break;
      }

      default: {
        // Relay unknown/custom messages to participants in the same trial room
        broadcast(trialId, {
          type: type !== "" ? type : "message",
          data: {
            timestamp: now,
            userId: clientInfo.userId,
            ...data,
          },
        });
        break;
      }
    }
  });

  server.addEventListener("close", () => {
    const room = rooms.get(trialId);
    if (room) {
      room.delete(server);
      if (room.size === 0) {
        rooms.delete(trialId);
      }
    }
  });

  server.addEventListener("error", () => {
    try {
      server.close();
    } catch {
      // ignore
    }
    const room = rooms.get(trialId);
    if (room) {
      room.delete(server);
      if (room.size === 0) {
        rooms.delete(trialId);
      }
    }
  });

  // Hand over the client end of the socket to the response
  return new Response(null, {
    status: 101,
    webSocket: client,
  });
}
