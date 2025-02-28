"use client"

import { useEffect } from "react"
import { useRouter } from "next/navigation"
import { useSession } from "next-auth/react"
import { api } from "~/trpc/react"

import { AppSidebar } from "~/components/navigation/app-sidebar"
import { Header } from "~/components/navigation/header"
import { SidebarProvider } from "~/components/ui/sidebar"
import { StudyProvider } from "~/components/providers/study-provider"
import { PluginStoreProvider } from "~/components/providers/plugin-store-provider"
import { PageTransition } from "~/components/layout/page-transition"

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode
}) {
  const { data: session, status } = useSession()
  const router = useRouter()

  // Get user's studies
  const { data: studies, isLoading: isLoadingStudies } = api.study.getMyStudies.useQuery(
    undefined,
    {
      enabled: status === "authenticated",
    }
  );

  useEffect(() => {
    if (status === "unauthenticated") {
      router.replace("/auth/signin")
    }
  }, [status, router])

  useEffect(() => {
    // Only redirect if we've loaded studies and user has none, and we're not already on onboarding
    if (!isLoadingStudies && studies && studies.length === 0 && !window.location.pathname.includes("/onboarding")) {
      router.replace("/onboarding")
    }
  }, [studies, isLoadingStudies, router])

  // Show nothing while loading
  if (status === "loading" || isLoadingStudies) {
    return null
  }

  // Show nothing if not authenticated (will redirect)
  if (!session) {
    return null
  }

  // Show nothing if no studies (will redirect to onboarding)
  if (studies && studies.length === 0) {
    return null
  }

  return (
    <SidebarProvider>
      <PluginStoreProvider>
        <StudyProvider>
          <div className="flex h-full min-h-screen w-full bg-muted/40 dark:bg-background relative">
            {/* Background Elements */}
            <div className="pointer-events-none fixed inset-0 z-0">
              {/* Base Gradient */}
              <div className="absolute inset-0 bg-gradient-to-b from-background via-primary/10 to-background" />
              
              {/* Gradient Orb */}
              <div className="absolute inset-0 flex items-center justify-center">
                <div className="absolute h-[1200px] w-[1200px] left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2">
                  <div className="absolute inset-0 rounded-full bg-gradient-to-r from-primary/30 via-secondary/30 to-background opacity-60 blur-3xl dark:opacity-40 animate-gradient" />
                </div>
              </div>

              {/* Dotted Pattern */}
              <div 
                className="absolute inset-0 opacity-[0.35] dark:opacity-[0.15]"
                style={{
                  backgroundImage: `
                    radial-gradient(circle at 1px 1px, hsl(var(--primary)/0.5) 1px, transparent 0)
                  `,
                  backgroundSize: '32px 32px',
                  maskImage: 'linear-gradient(to bottom, transparent, black 10%, black 90%, transparent)',
                }}
              />
            </div>

            {/* Content */}
            <AppSidebar className="z-20" />
            <div className="flex w-0 flex-1 flex-col z-10">
              <Header/>
              <main className="flex-1 overflow-auto p-4">
                <PageTransition>
                  {children}
                </PageTransition>
              </main>
            </div>
          </div>
        </StudyProvider>
      </PluginStoreProvider>
    </SidebarProvider>
  )
} 