'use client';

import React from "react";
import { SignIn } from "@clerk/nextjs";

export default function SignInPage() {
  return (
    <div className="container flex items-center justify-center min-h-screen py-10">
      <SignIn />
    </div>
  );
} 