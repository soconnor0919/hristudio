import { type PropsWithChildren } from "react"
import { Sidebar } from "~/components/sidebar"
import { inter } from "../layout"

import "~/styles/globals.css"

export default function RootLayout({ children }: PropsWithChildren) {
  return (
    <html lang="en">
      <body className={`font-sans ${inter.variable}`}>
        <div className="flex h-screen">
          <Sidebar />
          <main className="flex-1 overflow-y-auto">
            {children}
          </main>
        </div>
      </body>
    </html>
  )
}