'use client';

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import { useUser } from "@clerk/nextjs";
import { useRouter } from "next/navigation";
import { useState } from "react";
import { Loader2 } from "lucide-react";
import { Logo } from "~/components/logo";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "~/components/ui/alert-dialog";

interface InvitationAcceptContentProps {
  token: string;
}

export function InvitationAcceptContent({ token }: InvitationAcceptContentProps) {
  const { isLoaded, isSignedIn } = useUser();
  const router = useRouter();
  const [isAccepting, setIsAccepting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleAcceptInvitation = async () => {
    setIsAccepting(true);
    setError(null);
    
    try {
      const response = await fetch(`/api/invitations/accept/${token}`, {
        method: "POST",
      });

      if (!response.ok) {
        const errorText = await response.text();
        throw new Error(errorText || "Failed to accept invitation");
      }

      router.push("/dashboard");
    } catch (error) {
      setError(error instanceof Error ? error.message : "Failed to accept invitation");
      setIsAccepting(false);
    }
  };

  if (!isLoaded) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <Loader2 className="h-8 w-8 animate-spin" />
      </div>
    );
  }

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-50/50">
      <div className="w-full max-w-md px-4 py-8">
        <div className="flex flex-col items-center mb-8">
          <div className="mb-6">
            <Logo className="h-10" />
          </div>
          <p className="text-gray-500 text-center">
            A platform for managing human-robot interaction studies
          </p>
        </div>

        <Card className="shadow-lg">
          <CardHeader>
            <CardTitle>Research Study Invitation</CardTitle>
            <CardDescription>
              You've been invited to collaborate on a research study. {!isSignedIn && " Please sign in or create an account to continue."}
            </CardDescription>
          </CardHeader>
          <CardContent>
            {error && (
              <div className="mb-4 p-4 text-sm text-red-800 bg-red-100 rounded-lg">
                {error}
              </div>
            )}
            
            {isSignedIn ? (
              <AlertDialog>
                <AlertDialogTrigger asChild>
                  <Button className="w-full" disabled={isAccepting}>
                    Accept Invitation
                  </Button>
                </AlertDialogTrigger>
                <AlertDialogContent>
                  <AlertDialogHeader>
                    <AlertDialogTitle>Accept Research Study Invitation</AlertDialogTitle>
                    <AlertDialogDescription>
                      Are you sure you want to accept this invitation? You will be added as a collaborator to the research study.
                    </AlertDialogDescription>
                  </AlertDialogHeader>
                  <AlertDialogFooter>
                    <AlertDialogCancel>Cancel</AlertDialogCancel>
                    <AlertDialogAction
                      onClick={handleAcceptInvitation}
                      disabled={isAccepting}
                    >
                      {isAccepting ? (
                        <>
                          <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                          Accepting...
                        </>
                      ) : (
                        "Accept"
                      )}
                    </AlertDialogAction>
                  </AlertDialogFooter>
                </AlertDialogContent>
              </AlertDialog>
            ) : (
              <div className="flex flex-col gap-3">
                <Button
                  variant="default"
                  className="w-full"
                  onClick={() => router.push(`/sign-in?redirect_url=${encodeURIComponent(`/invite/accept/${token}`)}`)}
                >
                  Sign In
                </Button>
                <Button
                  variant="outline"
                  className="w-full"
                  onClick={() => router.push(`/sign-up?redirect_url=${encodeURIComponent(`/invite/accept/${token}`)}`)}
                >
                  Create Account
                </Button>
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
} 