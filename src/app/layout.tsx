import "~/styles/globals.css";

import { type Metadata } from "next";
import { Inter } from "next/font/google";

import { SessionProvider } from "next-auth/react";
import { TRPCReactProvider } from "~/trpc/react";

export const metadata: Metadata = {
  title: "HRIStudio",
  description:
    "Web-based platform for standardizing Wizard of Oz studies in Human-Robot Interaction research",
  icons: [{ rel: "icon", url: "/favicon.ico" }],
};

const inter = Inter({
  subsets: ["latin"],
  variable: "--font-sans",
});

export default function RootLayout({
  children,
}: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en" className={`${inter.variable}`}>
      <body>
        <SessionProvider>
          <TRPCReactProvider>{children}</TRPCReactProvider>
        </SessionProvider>
      </body>
    </html>
  );
}
