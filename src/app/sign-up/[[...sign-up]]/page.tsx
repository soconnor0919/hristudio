"use client"

import { useState } from "react"
import { useSignUp } from "@clerk/nextjs"
import { useRouter } from "next/navigation"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "~/components/ui/card"
import { Input } from "~/components/ui/input"
import { Button } from "~/components/ui/button"
import { Separator } from "~/components/ui/separator"
import Link from "next/link"
import { FcGoogle } from "react-icons/fc"
import { FaApple } from "react-icons/fa"

export default function SignUpPage() {
  const { isLoaded, signUp, setActive } = useSignUp()
  const [emailAddress, setEmailAddress] = useState("")
  const [password, setPassword] = useState("")
  const router = useRouter()

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    if (!isLoaded) return

    try {
      const result = await signUp.create({
        emailAddress,
        password,
      })

      if (result.status === "complete") {
        await setActive({ session: result.createdSessionId })

        // Create a user entry in the database
        const response = await fetch('/api/users', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ email: emailAddress }),
        })

        if (!response.ok) {
          const errorData = await response.json()
          console.error("Error creating user in database:", errorData.error)
          return // Optionally handle the error (e.g., show a message to the user)
        }

        router.push("/dash")
      }
    } catch (err) {
      const error = err as { errors?: { message: string }[] }; // Specify type
      console.error("Error:", error.errors?.[0]?.message ?? "Unknown error") // Use optional chaining
    }
  }

  const signUpWith = (strategy: "oauth_google" | "oauth_apple") => {
    if (!isLoaded) return
    signUp.authenticateWithRedirect({
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
          <CardTitle>Sign up for HRIStudio</CardTitle>
          <CardDescription>Create an account to get started</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid w-full items-center gap-4">
            <Button variant="outline" onClick={() => signUpWith("oauth_google")}>
              <FcGoogle className="mr-2 h-4 w-4" />
              Sign up with Google
            </Button>
            <Button variant="outline" onClick={() => signUpWith("oauth_apple")}>
              <FaApple className="mr-2 h-4 w-4" />
              Sign up with Apple
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
              <Button className="w-full" type="submit">Sign Up</Button>
            </div>
          </form>
        </CardContent>
        <CardFooter className="flex flex-col">
          <p className="mt-4 text-sm text-center">
            Already have an account?{" "}
            <Link href="/sign-in" className="text-blue-600 hover:underline">
              Sign in
            </Link>
          </p>
        </CardFooter>
      </Card>
    </div>
  )
}