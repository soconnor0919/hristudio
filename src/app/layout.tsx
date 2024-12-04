import {
  ClerkProvider
} from '@clerk/nextjs';
import { Analytics } from "@vercel/analytics/react"
import { Inter } from 'next/font/google';
import './globals.css';
import { Metadata } from 'next';

const inter = Inter({ subsets: ['latin'] });

export const metadata: Metadata = {
  title: 'HRIStudio',
  description: 'A platform for managing human-robot interaction studies',
  icons: {
    icon: [
      { url: '/icon', type: 'image/svg+xml' },
    ],
  },
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <ClerkProvider>
      <Analytics />
      <html lang="en">
        <body className={inter.className}>
          {children}
        </body>
      </html>
    </ClerkProvider>
  )
}