"use client";

import { signOut } from "next-auth/react";
import { toast } from "sonner";
import { TRPCClientError } from "@trpc/client";

/**
 * Auth error codes that should trigger automatic logout
 */
const AUTH_ERROR_CODES = [
  "UNAUTHORIZED",
  "FORBIDDEN",
  "UNAUTHENTICATED",
] as const;

/**
 * Auth error messages that should trigger automatic logout
 */
const AUTH_ERROR_MESSAGES = [
  "unauthorized",
  "unauthenticated",
  "forbidden",
  "invalid token",
  "token expired",
  "session expired",
  "authentication failed",
  "access denied",
] as const;

/**
 * Checks if an error is an authentication/authorization error that should trigger logout
 */
export function isAuthError(error: unknown): boolean {
  if (!error) return false;

  // Check TRPC errors
  if (error instanceof TRPCClientError) {
    // Check error code
    const trpcErrorData = error.data as
      | { code?: string; httpStatus?: number }
      | undefined;
    const errorCode = trpcErrorData?.code;
    if (
      errorCode &&
      AUTH_ERROR_CODES.includes(errorCode as (typeof AUTH_ERROR_CODES)[number])
    ) {
      return true;
    }

    // Check HTTP status codes
    const httpStatus = trpcErrorData?.httpStatus;
    if (httpStatus === 401 || httpStatus === 403) {
      return true;
    }

    // Check error message
    const message = error.message?.toLowerCase() ?? "";
    return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
  }

  // Check generic errors
  if (error instanceof Error) {
    const message = error.message?.toLowerCase() || "";
    return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
  }

  // Check error objects with message property
  if (typeof error === "object" && error !== null) {
    if ("message" in error) {
      const errorObj = error as { message: unknown };
      const message = String(errorObj.message).toLowerCase();
      return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
    }

    // Check for status codes in error objects
    if ("status" in error) {
      const statusObj = error as { status: unknown };
      const status = statusObj.status as number;
      return status === 401 || status === 403;
    }
  }

  return false;
}

/**
 * Handles authentication errors by logging out the user
 */
export async function handleAuthError(
  error: unknown,
  customMessage?: string,
): Promise<void> {
  if (!isAuthError(error)) {
    return;
  }

  console.warn("Authentication error detected, logging out user:", error);

  // Show user-friendly message
  const message = customMessage ?? "Session expired. Please log in again.";
  toast.error(message);

  // Small delay to let the toast show
  setTimeout(() => {
    void (async () => {
      try {
        await signOut({
          callbackUrl: "/",
          redirect: true,
        });
      } catch (signOutError) {
        console.error("Error during sign out:", signOutError);
        // Force redirect if signOut fails
        window.location.href = "/";
      }
    })();
  }, 1000);
}

/**
 * React Query error handler that automatically handles auth errors
 */
export function createAuthErrorHandler(customMessage?: string) {
  return (error: unknown) => {
    void handleAuthError(error, customMessage);
  };
}

/**
 * tRPC error handler that automatically handles auth errors
 */
export function handleTRPCError(error: unknown, customMessage?: string): void {
  void handleAuthError(error, customMessage);
}

/**
 * Generic error handler for any error type
 */
export function handleGenericError(
  error: unknown,
  customMessage?: string,
): void {
  void handleAuthError(error, customMessage);
}

/**
 * Hook-style error handler for use in React components
 */
export function useAuthErrorHandler() {
  return {
    handleAuthError: (error: unknown, customMessage?: string) => {
      void handleAuthError(error, customMessage);
    },
    isAuthError,
    createErrorHandler: createAuthErrorHandler,
  };
}

/**
 * Higher-order function to wrap API calls with automatic auth error handling
 */
export function withAuthErrorHandling<
  T extends (...args: unknown[]) => Promise<unknown>,
>(fn: T, customMessage?: string): T {
  return (async (...args: Parameters<T>): Promise<ReturnType<T>> => {
    try {
      return (await fn(...args)) as ReturnType<T>;
    } catch (error) {
      await handleAuthError(error, customMessage);
      throw error; // Re-throw so calling code can handle it too
    }
  }) as T;
}

/**
 * Utility to check if current error should show a generic error message
 * (i.e., it's not an auth error that will auto-logout)
 */
export function shouldShowGenericError(error: unknown): boolean {
  return !isAuthError(error);
}
