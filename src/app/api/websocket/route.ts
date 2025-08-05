import { type NextRequest } from "next/server";

// Store active WebSocket connections (for external WebSocket server)
// These would be used by a separate WebSocket implementation
// const connections = new Map<string, Set<WebSocket>>();
// const userConnections = new Map<
//   string,
//   { userId: string; trialId: string; role: string }
// >();

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
