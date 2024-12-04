import { NextResponse } from "next/server";
import { ENVIRONMENT } from "~/db/schema";

export type ApiResponse<T> = {
  data?: T;
  error?: string;
};

export function getEnvironment(): typeof ENVIRONMENT[keyof typeof ENVIRONMENT] {
  return process.env.NODE_ENV === 'production' 
    ? ENVIRONMENT.PRODUCTION 
    : ENVIRONMENT.DEVELOPMENT;
}

export function createApiResponse<T>(
  data?: T,
  error?: string,
  status: number = error ? 400 : 200
): NextResponse<ApiResponse<T>> {
  return NextResponse.json(
    { data, error },
    { status }
  );
}

export const ApiError = {
  Unauthorized: () => createApiResponse(undefined, "Unauthorized", 401),
  Forbidden: () => createApiResponse(undefined, "Forbidden", 403),
  NotFound: (resource: string) => createApiResponse(undefined, `${resource} not found`, 404),
  BadRequest: (message: string) => createApiResponse(undefined, message, 400),
  ServerError: (error: unknown) => {
    console.error("Server error:", error);
    return createApiResponse(
      undefined,
      "Internal server error",
      500
    );
  }
}; 