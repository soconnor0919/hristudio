import { ClerkProvider } from "@clerk/nextjs";
import { Inter } from 'next/font/google';
import { Toaster } from "~/components/ui/toaster";
import "~/app/globals.css";

const inter = Inter({ subsets: ['latin'] });

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <ClerkProvider>
      <html lang="en">
        <body className={inter.className}>
          {children}
          <Toaster />
        </body>
      </html>
    </ClerkProvider>
  );
}