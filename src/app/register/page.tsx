import type { Metadata } from "next";
import Link from "next/link";
import { redirect } from "next/navigation";
import { getServerAuthSession } from "~/server/auth";

import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardTitle
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Logo } from "~/components/logo";

export const metadata: Metadata = {
  title: "Register | HRIStudio",
  description: "Create a new account",
};

export default async function RegisterPage() {
  const session = await getServerAuthSession();

  if (session) {
    redirect("/dashboard");
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
                  Create an account
                </CardTitle>
                <CardDescription className="text-base">
                  Enter your details to get started
                </CardDescription>
              </div>
              <form action="/api/auth/register" method="POST" className="animate-in fade-in-50 slide-in-from-bottom-4">
                <div className="grid gap-4">
                  <div className="grid gap-4 sm:grid-cols-2">
                    <div className="grid gap-2">
                      <Label htmlFor="firstName">First Name</Label>
                      <Input
                        id="firstName"
                        name="firstName"
                        placeholder="John"
                        autoComplete="given-name"
                        className="auth-input"
                        required
                      />
                    </div>
                    <div className="grid gap-2">
                      <Label htmlFor="lastName">Last Name</Label>
                      <Input
                        id="lastName"
                        name="lastName"
                        placeholder="Doe"
                        autoComplete="family-name"
                        className="auth-input"
                        required
                      />
                    </div>
                  </div>
                  <div className="grid gap-2">
                    <Label htmlFor="email">Email</Label>
                    <Input
                      id="email"
                      name="email"
                      placeholder="name@example.com"
                      type="email"
                      autoCapitalize="none"
                      autoComplete="email"
                      autoCorrect="off"
                      className="auth-input"
                      required
                    />
                  </div>
                  <div className="grid gap-2">
                    <Label htmlFor="password">Password</Label>
                    <Input
                      id="password"
                      name="password"
                      type="password"
                      autoComplete="new-password"
                      className="auth-input"
                      required
                    />
                    <p className="text-sm text-muted-foreground">
                      Must be at least 8 characters long
                    </p>
                  </div>
                  <Button className="w-full" type="submit">
                    Create account
                  </Button>
                </div>
                <div className="mt-4 text-center text-sm text-muted-foreground">
                  Already have an account?{" "}
                  <Link
                    href="/login"
                    className="font-medium text-primary underline-offset-4 transition-colors hover:underline"
                  >
                    Sign in
                  </Link>
                </div>
              </form>
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
          By creating an account, you agree to our{" "}
          <Link href="/terms" className="underline underline-offset-4 hover:text-primary">
            Terms of Service
          </Link>{" "}
          and{" "}
          <Link href="/privacy" className="underline underline-offset-4 hover:text-primary">
            Privacy Policy
          </Link>
          .
        </p>
      </div>
    </div>
  );
} 