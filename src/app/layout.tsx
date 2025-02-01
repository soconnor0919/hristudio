import "~/styles/globals.css";

import { Inter } from "next/font/google";
import { GeistSans } from 'geist/font/sans';
import { headers } from "next/headers";

import { TRPCReactProvider } from "~/trpc/react";
import { cn } from "~/lib/utils";
import { Providers } from "~/components/providers";
import DatabaseCheck from "~/components/db-check";

const inter = Inter({
  subsets: ["latin"],
  variable: "--font-sans",
});


export const metadata = {
  title: "HRIStudio",
  description: "A platform for managing human research studies and participant interactions.",
  icons: [{ rel: "icon", url: "/favicon.ico" }],
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en" className="h-full">
      <body className={cn(
        "min-h-screen bg-background font-sans antialiased",
        inter.variable
      )}>
        <TRPCReactProvider {...{ headers: headers() }}>
          <Providers>
            <DatabaseCheck>
              <div className="relative h-full">
                {children}
              </div>
            </DatabaseCheck>
          </Providers>
        </TRPCReactProvider>
      </body>
    </html>
  );
}
