'use client';

import { SignedIn, SignedOut, SignInButton, SignOutButton, UserButton, useUser } from "@clerk/nextjs";
import { Button } from "~/components/ui/button";
import { useEffect, useState } from "react";

export default function Home() {
  const { user, isLoaded } = useUser(); // Get user information and loading state
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (isLoaded) {
      setLoading(false);
    }
  }, [isLoaded]);

  if (loading) {
    return <div>Loading...</div>; // Show a loading state while fetching user data
  }

  return (
    <div className="flex flex-col items-center justify-center min-h-screen p-8">
      <h1 className="text-3xl font-bold mb-4">Welcome to HRIStudio</h1>
      <SignedOut>
        <SignInButton>
          <Button>Sign In</Button>
        </SignInButton>
      </SignedOut>
      <SignedIn>
        <UserButton />
        <p className="mt-4">Signed in as: {user?.emailAddresses[0].emailAddress}</p> {/* Display user's email */}
        <SignOutButton>
          <Button>Sign Out</Button>
        </SignOutButton>
      </SignedIn>
    </div>
  );
}
