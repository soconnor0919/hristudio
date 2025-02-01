'use server'

import { db } from "~/server/db"
import { users } from "~/server/db/schema"
import { Card } from "~/components/ui/card"
import { DatabaseIcon } from "lucide-react"
import { Logo } from "~/components/logo"

async function checkDatabase() {
  try {
    // Try a simple query to check database connection
    await db.select().from(users).limit(1)
    return true
  } catch (error) {
    console.error("Database connection error:", error)
    return false
  }
}

export default async function DatabaseCheck({
  children,
}: {
  children: React.ReactNode
}): Promise<JSX.Element> {
  const isConnected = await checkDatabase()

  if (isConnected) {
    return <>{children}</>
  }

  return (
    <div className="grid min-h-screen place-items-center bg-background font-sans antialiased">
      <div className="flex flex-col items-center gap-8">
        <Logo 
          className="text-2xl"
          iconClassName="h-8 w-8"
        />
        <Card className="w-[448px] border-destructive p-6">
          <div className="flex flex-col items-center gap-4 text-center">
            <DatabaseIcon className="h-12 w-12 text-destructive" />
            <div className="space-y-2">
              <h1 className="text-2xl font-bold tracking-tight">
                Database Connection Error
              </h1>
              <p className="text-muted-foreground">
                Could not connect to the database. Please make sure the database is running and try again.
              </p>
            </div>
            <div className="w-full rounded-lg bg-muted p-4">
              <p className="mb-2 font-mono text-sm">Start the database with:</p>
              <code className="block rounded bg-background p-2 text-sm">
                docker-compose up -d
              </code>
            </div>
          </div>
        </Card>
      </div>
    </div>
  )
} 
