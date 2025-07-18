"use client";

import { useEffect, useState } from "react";
import { signOut, useSession } from "next-auth/react";
import { useRouter } from "next/navigation";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

export default function SignOutPage() {
  const { data: session, status } = useSession();
  const router = useRouter();
  const [isSigningOut, setIsSigningOut] = useState(false);

  useEffect(() => {
    // If user is not logged in, redirect to home
    if (status === "loading") return; // Still loading
    if (!session) {
      router.push("/");
      return;
    }
  }, [session, status, router]);

  const handleSignOut = async () => {
    setIsSigningOut(true);
    try {
      await signOut({
        callbackUrl: "/",
        redirect: true,
      });
    } catch (error) {
      console.error("Error signing out:", error);
      setIsSigningOut(false);
    }
  };

  if (status === "loading") {
    return (
      <div className="flex min-h-screen items-center justify-center bg-gradient-to-br from-slate-50 to-slate-100">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-blue-600 mx-auto mb-4"></div>
          <p className="text-slate-600">Loading...</p>
        </div>
      </div>
    );
  }

  if (!session) {
    return null; // Will redirect via useEffect
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-gradient-to-br from-slate-50 to-slate-100 px-4">
      <div className="w-full max-w-md">
        {/* Header */}
        <div className="mb-8 text-center">
          <Link href="/" className="inline-block">
            <h1 className="text-3xl font-bold text-slate-900">HRIStudio</h1>
          </Link>
          <p className="mt-2 text-slate-600">
            Sign out of your research account
          </p>
        </div>

        {/* Sign Out Card */}
        <Card>
          <CardHeader>
            <CardTitle>Sign Out</CardTitle>
            <CardDescription>
              Are you sure you want to sign out of your account?
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="rounded-md bg-blue-50 p-3 text-sm text-blue-700">
              <p className="font-medium">
                Currently signed in as: {session.user.name ?? session.user.email}
              </p>
            </div>

            <div className="flex gap-3">
              <Button
                onClick={handleSignOut}
                className="flex-1"
                disabled={isSigningOut}
                variant="destructive"
              >
                {isSigningOut ? "Signing out..." : "Sign Out"}
              </Button>
              <Button asChild variant="outline" className="flex-1">
                <Link href="/">Cancel</Link>
              </Button>
            </div>
          </CardContent>
        </Card>

        {/* Footer */}
        <div className="mt-8 text-center text-xs text-slate-500">
          <p>
            © 2024 HRIStudio. A platform for Human-Robot Interaction research.
          </p>
        </div>
      </div>
    </div>
  );
}
