import { ClerkProvider } from '@clerk/nextjs'
import { Inter } from "next/font/google"
import { StudyProvider } from '~/context/StudyContext'

import "~/styles/globals.css"

const inter = Inter({
  subsets: ["latin"],
  display: "swap",
  variable: "--font-sans",
})

export const metadata = {
  title: "HRIStudio",
  description: "Created with create-t3-app",
  icons: [{ rel: "icon", url: "/favicon.ico" }],
}

export default function RootLayout({ children }: React.PropsWithChildren) {
  return (
    <ClerkProvider>
      <StudyProvider>
        <html lang="en" className={inter.variable}>
          <body className="font-sans">
            {children}
          </body>
        </html>
      </StudyProvider>
    </ClerkProvider>
  )
}