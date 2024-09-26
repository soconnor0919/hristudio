"use client"

import { useState } from "react"
import { useSignIn, useSignUp } from "@clerk/nextjs"
import { useRouter } from "next/navigation"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "~/components/ui/card"
import { Input } from "~/components/ui/input"
import { Button } from "~/components/ui/button"
import { Separator } from "~/components/ui/separator"
import Link from "next/link"
import { FcGoogle } from "react-icons/fc"
import { FaApple } from "react-icons/fa"

export default function SignInPage() {
  const { isLoaded, signIn, setActive } = useSignIn();
  const { signUp } = useSignUp();
  const [emailAddress, setEmailAddress] = useState("");
  const [password, setPassword] = useState("");
  const router = useRouter();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!isLoaded) return;

    try {
      const result = await signIn.create({
        identifier: emailAddress,
        password,
      });

      if (result.status === "complete") {
        await setActive({ session: result.createdSessionId });
        router.push("/dash");
      }
    } catch (err) {
      const error = err as { errors?: { message: string }[] };
      console.error("Error:", error.errors?.[0]?.message ?? "Unknown error");

      // If the error indicates the user does not exist, trigger sign-up
      if (error.errors?.[0]?.message.includes("not found")) {
        if (!signUp) {
          console.error("Sign-up functionality is not available.");
          return;
        }

        try {
          const signUpResult = await signUp.create({
            emailAddress,
            password,
          });

          if (signUpResult.status === "complete") {
            await setActive({ session: signUpResult.createdSessionId });

            // Create a user entry in the database
            const response = await fetch('/api/users', {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ email: emailAddress }),
            });

            if (!response.ok) {
              const errorData = await response.json();
              console.error("Error creating user in database:", errorData.error);
              return; // Optionally handle the error (e.g., show a message to the user)
            }

            router.push("/dash");
          }
        } catch (signUpErr) {
          const signUpError = signUpErr as { errors?: { message: string }[] };
          console.error("Sign-up Error:", signUpError.errors?.[0]?.message ?? "Unknown error");
        }
      }
    }
  }

  const signInWith = (strategy: "oauth_google" | "oauth_apple") => {
    if (!isLoaded) return
    signIn.authenticateWithRedirect({
      strategy,
      redirectUrl: "/sso-callback",
      redirectUrlComplete: "/dash",
    }).catch((error) => {
      console.error("Authentication error:", error); // Handle any potential errors
    })
  }

  return (
    <div className="min-h-screen bg-gradient-to-b from-blue-100 to-white flex items-center justify-center">
      <Card className="w-[350px]">
        <CardHeader>
          <CardTitle>Sign in to HRIStudio</CardTitle>
          <CardDescription>Enter your email and password to sign in</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid w-full items-center gap-4">
            <Button variant="outline" onClick={() => signInWith("oauth_google")}>
              <FcGoogle className="mr-2 h-4 w-4" />
              Sign in with Google
            </Button>
            <Button variant="outline" onClick={() => signInWith("oauth_apple")}>
              <FaApple className="mr-2 h-4 w-4" />
              Sign in with Apple
            </Button>
          </div>
          <Separator className="my-4" />
          <form onSubmit={handleSubmit}>
            <div className="grid w-full items-center gap-4">
              <div className="flex flex-col space-y-1.5">
                <Input
                  id="email"
                  placeholder="Email"
                  type="email"
                  value={emailAddress}
                  onChange={(e) => setEmailAddress(e.target.value)}
                />
              </div>
              <div className="flex flex-col space-y-1.5">
                <Input
                  id="password"
                  placeholder="Password"
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                />
              </div>
              <Button className="w-full" type="submit">Sign In</Button>
            </div>
          </form>
        </CardContent>
        <CardFooter className="flex flex-col">
          <p className="mt-4 text-sm text-center">
            Don&apos;t have an account?{" "}
            <Link href="/sign-up" className="text-blue-600 hover:underline">
              Sign up
            </Link>
          </p>
        </CardFooter>
      </Card>
    </div>
  )
}