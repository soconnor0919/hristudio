'use client';

import React from "react";
import { SignUp } from "@clerk/nextjs";

export default function SignUpPage() {
  return (
    <div className="container flex items-center justify-center min-h-screen py-10">
      <SignUp />
    </div>
  );
} 