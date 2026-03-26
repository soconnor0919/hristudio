"use client";

import { signOut } from "~/lib/auth-client";
import { toast } from "sonner";
import { TRPCClientError } from "@trpc/client";

const AUTH_ERROR_CODES = [
  "UNAUTHORIZED",
  "FORBIDDEN",
  "UNAUTHENTICATED",
] as const;

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

export function isAuthError(error: unknown): boolean {
  if (!error) return false;

  if (error instanceof TRPCClientError) {
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

    const httpStatus = trpcErrorData?.httpStatus;
    if (httpStatus === 401 || httpStatus === 403) {
      return true;
    }

    const message = error.message?.toLowerCase() ?? "";
    return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
  }

  if (error instanceof Error) {
    const message = error.message?.toLowerCase() || "";
    return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
  }

  if (typeof error === "object" && error !== null) {
    if ("message" in error) {
      const errorObj = error as { message: unknown };
      const message = String(errorObj.message).toLowerCase();
      return AUTH_ERROR_MESSAGES.some((authMsg) => message.includes(authMsg));
    }

    if ("status" in error) {
      const statusObj = error as { status: unknown };
      const status = statusObj.status as number;
      return status === 401 || status === 403;
    }
  }

  return false;
}

export async function handleAuthError(
  error: unknown,
  customMessage?: string,
): Promise<void> {
  if (!isAuthError(error)) {
    return;
  }

  console.warn("Authentication error detected, logging out user:", error);

  const message = customMessage ?? "Session expired. Please log in again.";
  toast.error(message);

  setTimeout(() => {
    void (async () => {
      try {
        await signOut();
        window.location.href = "/";
      } catch (signOutError) {
        console.error("Error during sign out:", signOutError);
        window.location.href = "/";
      }
    })();
  }, 1000);
}

export function useAuthErrorHandler() {
  return {
    handleAuthError,
    isAuthError,
  };
}

export function shouldShowGenericError(error: unknown): boolean {
  return !isAuthError(error);
}
