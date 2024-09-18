import { ClerkProvider } from '@clerk/nextjs'
import { Inter } from "next/font/google"

import "~/styles/globals.css"

const inter = Inter({
  subsets: ["latin"],
  display: "swap",
  variable: "--font-sans",
})

export const metadata = {
  title: "T3 App",
  description: "Created with create-t3-app",
  icons: [{ rel: "icon", url: "/favicon.ico" }],
}

export default function RootLayout({ children }: React.PropsWithChildren) {
  return (
    <ClerkProvider>
      {/* <ThemeProvider attribute="class" defaultTheme="system" enableSystem> */}
        <html lang="en" className={inter.variable}>
          <body className="font-sans">
            {children}
          </body>
        </html>
      {/* </ThemeProvider> */}
    </ClerkProvider>
  )
}