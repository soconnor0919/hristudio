"use client";

import { useRouter, useSearchParams } from "next/navigation";
import { useSession, signOut } from "next-auth/react";
import { api } from "~/trpc/react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { useToast } from "~/hooks/use-toast";
import Link from "next/link";
import { format } from "date-fns";
import { Logo } from "~/components/logo";

export default function InvitePage() {
  const router = useRouter();
  const searchParams = useSearchParams();
  const { data: session, status } = useSession();
  const { toast } = useToast();

  const token = searchParams.get("token");

  // Don't fetch invitation data until we're authenticated
  const { data: invitation, isLoading: isLoadingInvitation } = api.study.getInvitation.useQuery(
    { token: token! },
    {
      enabled: !!token && status === "authenticated",
      retry: false,
      onError: (error) => {
        toast({
          title: "Error",
          description: error.message,
          variant: "destructive",
        });
      },
    }
  );

  const { mutate: acceptInvitation, isLoading: isAccepting } = api.study.acceptInvitation.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "You have successfully joined the study.",
      });
      router.push(`/dashboard/studies/${invitation?.studyId}`);
      router.refresh();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  // Show loading state for missing token
  if (!token) {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Invalid Invitation
                  </CardTitle>
                  <CardDescription className="text-base">
                    No invitation token provided. Please check your invitation link.
                  </CardDescription>
                </div>
                <Button asChild className="w-full">
                  <Link href="/dashboard">Return to Dashboard</Link>
                </Button>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  // Show authentication required state
  if (status === "unauthenticated") {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Study Invitation
                  </CardTitle>
                  <CardDescription className="text-base">
                    Sign in or create an account to view and accept this invitation.
                  </CardDescription>
                </div>
                <div className="space-y-4">
                  <Button asChild variant="default" className="w-full">
                    <Link href={`/auth/signin?callbackUrl=${encodeURIComponent('/invite?token=' + token)}`}>
                      Sign In
                    </Link>
                  </Button>
                  <div className="relative">
                    <div className="absolute inset-0 flex items-center">
                      <span className="w-full border-t" />
                    </div>
                    <div className="relative flex justify-center text-xs uppercase">
                      <span className="bg-background px-2 text-muted-foreground">
                        or
                      </span>
                    </div>
                  </div>
                  <Button asChild variant="outline" className="w-full">
                    <Link href={`/auth/signup?callbackUrl=${encodeURIComponent('/invite?token=' + token)}`}>
                      Create Account
                    </Link>
                  </Button>
                </div>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  // Show loading state while checking authentication
  if (status === "loading") {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Loading...
                  </CardTitle>
                  <CardDescription className="text-base">
                    Please wait while we load your invitation.
                  </CardDescription>
                </div>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  // Show error state for invalid invitation
  if (!invitation) {
    return (
      <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
        <Logo 
          className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
          iconClassName="text-primary"
        />
        <div className="w-full max-w-[800px] px-4 py-8">
          <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
            <CardContent className="grid p-0 md:grid-cols-2">
              <div className="p-6 md:p-8">
                <div className="mb-6 space-y-2">
                  <CardTitle className="text-2xl font-bold tracking-tight">
                    Invalid Invitation
                  </CardTitle>
                  <CardDescription className="text-base">
                    This invitation link appears to be invalid or has expired. Please request a new invitation.
                  </CardDescription>
                </div>
                <Button asChild className="w-full">
                  <Link href="/dashboard">Return to Dashboard</Link>
                </Button>
              </div>
              <div className="relative hidden h-full md:block">
                <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <Logo 
                    className="pointer-events-none"
                    iconClassName="h-32 w-32 mr-0 text-primary/40"
                    textClassName="sr-only"
                  />
                </div>
              </div>
            </CardContent>
          </Card>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-gradient relative flex min-h-screen items-center justify-center px-4">
      <Logo 
        className="absolute left-4 top-4 text-lg transition-colors hover:text-primary md:left-8 md:top-8"
        iconClassName="text-primary"
      />
      <div className="w-full max-w-[800px] px-4 py-8">
        <Card className="auth-card shadow-xl transition-shadow hover:shadow-lg">
          <CardContent className="grid p-0 md:grid-cols-2">
            <div className="p-6 md:p-8">
              <div className="mb-6 space-y-2">
                <CardTitle className="text-2xl font-bold tracking-tight">
                  Study Invitation
                </CardTitle>
                <CardDescription className="text-base">
                  You've been invited to join {invitation.study.title} as a {invitation.role}.
                </CardDescription>
              </div>
              <div className="space-y-4">
                <div className="rounded-lg bg-muted p-4 space-y-2">
                  <div>
                    <span className="text-sm font-medium text-muted-foreground">Study: </span>
                    <span className="text-sm">{invitation.study.title}</span>
                  </div>
                  {invitation.study.description && (
                    <div>
                      <span className="text-sm font-medium text-muted-foreground">Description: </span>
                      <span className="text-sm">{invitation.study.description}</span>
                    </div>
                  )}
                  <div>
                    <span className="text-sm font-medium text-muted-foreground">Role: </span>
                    <span className="text-sm capitalize">{invitation.role}</span>
                  </div>
                  <div>
                    <span className="text-sm font-medium text-muted-foreground">Invited by: </span>
                    <span className="text-sm">
                      {invitation.creator.firstName} {invitation.creator.lastName}
                    </span>
                  </div>
                  <div>
                    <span className="text-sm font-medium text-muted-foreground">Expires: </span>
                    <span className="text-sm">{format(new Date(invitation.expiresAt), "PPp")}</span>
                  </div>
                </div>

                {session.user.email === invitation.email ? (
                  <Button
                    className="w-full"
                    onClick={() => acceptInvitation({ token })}
                    disabled={isAccepting}
                  >
                    {isAccepting ? "Accepting..." : "Accept Invitation"}
                  </Button>
                ) : (
                  <div className="space-y-4">
                    <p className="text-sm text-muted-foreground">
                      This invitation was sent to {invitation.email}, but you're signed in with a different
                      email address ({session.user.email}).
                    </p>
                    <div className="flex flex-col gap-2">
                      <Button asChild variant="default" className="w-full">
                        <Link href={`/auth/signin?callbackUrl=${encodeURIComponent('/invite?token=' + token)}`}>
                          Sign in with a different account
                        </Link>
                      </Button>
                      <Button 
                        variant="outline" 
                        className="w-full"
                        onClick={() => signOut({ callbackUrl: `/invite?token=${token}` })}
                      >
                        Sign out
                      </Button>
                    </div>
                  </div>
                )}
              </div>
            </div>
            <div className="relative hidden h-full md:block">
              <div className="absolute inset-0 bg-gradient-to-br from-primary/30 via-primary/20 to-secondary/10 rounded-r-lg" />
              <div className="absolute inset-0 flex items-center justify-center">
                <Logo 
                  className="pointer-events-none"
                  iconClassName="h-32 w-32 mr-0 text-primary/40"
                  textClassName="sr-only"
                />
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
} 