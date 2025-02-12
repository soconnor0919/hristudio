"use client"

import { useEffect } from "react"
import { useRouter } from "next/navigation"
import { useSession } from "next-auth/react"
import { api } from "~/trpc/react"

import { AppSidebar } from "~/components/navigation/app-sidebar"
import { Header } from "~/components/navigation/header"
import { SidebarProvider } from "~/components/ui/sidebar"
import { StudyProvider } from "~/components/providers/study-provider"
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
    // Only redirect if we've loaded studies and user has none
    if (!isLoadingStudies && studies && studies.length === 0) {
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
      <StudyProvider>
        <div className="flex h-full min-h-screen w-full">
          <AppSidebar />
          <div className="flex w-0 flex-1 flex-col">
            <Header />
            <main className="flex-1 overflow-auto p-4">
              <PageTransition>
                {children}
              </PageTransition>
            </main>
          </div>
        </div>
      </StudyProvider>
    </SidebarProvider>
  )
} 