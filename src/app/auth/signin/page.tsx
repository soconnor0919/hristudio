import type { Metadata } from "next";
import Link from "next/link";
import { redirect } from "next/navigation";
import { getServerAuthSession } from "~/server/auth";

import {
  Card,
  CardContent,
  CardDescription,
  CardTitle,
} from "~/components/ui/card";
import { SignInForm } from "~/components/auth/sign-in-form";
import { Logo } from "~/components/logo";

export const metadata: Metadata = {
  title: "Sign In | HRIStudio",
  description: "Sign in to your account",
};

export default async function SignInPage({
  searchParams,
}: {
  searchParams: Promise<Record<string, string | string[] | undefined>>
}) {
  const session = await getServerAuthSession();
  if (session) {
    redirect("/dashboard");
  }

  const params = await searchParams;
  const error = params?.error ? String(params.error) : null;
  const showError = error === "CredentialsSignin";

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
                  Welcome back
                </CardTitle>
                <CardDescription className="text-base">
                  Sign in to your account to continue
                </CardDescription>
              </div>
              <SignInForm error={showError} />
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
        <p className="mt-6 text-center text-sm text-muted-foreground">
          Don't have an account?{" "}
          <Link 
            href={`/auth/signup${params?.callbackUrl ? `?callbackUrl=${params.callbackUrl}` : ''}`}
            className="underline underline-offset-4 hover:text-primary"
          >
            Sign up
          </Link>
        </p>
      </div>
    </div>
  );
} 