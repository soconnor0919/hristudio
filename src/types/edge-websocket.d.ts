/**
 * Edge WebSocket TypeScript declarations for Next.js Edge runtime.
 *
 * Purpose:
 * - Provide typings for the non-standard `WebSocketPair` constructor available in Edge runtimes.
 * - Augment the DOM `WebSocket` interface with the `accept()` method (server-side socket).
 * - Augment `ResponseInit` to allow `{ webSocket: WebSocket }` when returning a 101 Switching Protocols response.
 *
 * This file is safe to include in strict mode projects.
 */

declare global {
  /**
   * Edge runtime-specific constructor that yields a pair of WebSockets:
   * index 0 is the client end, index 1 is the server end.
   *
   * Usage:
   *   const pair = new WebSocketPair();
   *   const [client, server] = Object.values(pair) as [WebSocket, WebSocket];
   */
  // Edge WebSocketPair declaration
  var WebSocketPair: {
    new (): { 0: WebSocket; 1: WebSocket };
    prototype: object;
  };

  /**
   * The server-side WebSocket in Edge runtimes exposes `accept()` to finalize the upgrade.
   * This augments the standard DOM WebSocket interface.
   */
  interface WebSocket {
    /**
     * Accept the server-side WebSocket before sending/receiving messages.
     * No-op on client-side sockets.
     */
    accept(): void;
  }

  /**
   * Next.js Edge runtime allows `webSocket` in ResponseInit when returning a 101 response.
   * This augments the standard DOM ResponseInit interface.
   */
  interface ResponseInit {
    webSocket?: WebSocket;
  }
}

export {};
