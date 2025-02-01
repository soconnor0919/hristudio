"use client"

import * as React from "react"
import { useRouter } from "next/navigation"
import { Notebook, ChevronsUpDown, Plus } from "lucide-react"
import { useSession } from "next-auth/react"

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu"
import {
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
  useSidebar,
} from "~/components/ui/sidebar"
import { useStudy } from "~/components/providers/study-provider"
import { Button } from "~/components/ui/button"
import { cn } from "~/lib/utils"

export function StudySwitcher() {
  const { status } = useSession()
  
  // Show nothing while loading to prevent flash
  if (status === "loading") {
    return null
  }
  
  return <StudySwitcherContent />
}

function StudySwitcherContent() {
  const { isMobile } = useSidebar()
  const router = useRouter()
  const { studies, activeStudy, setActiveStudy, isLoading } = useStudy()

  const handleCreateStudy = () => {
    router.push("/dashboard/studies/new")
  }

  if (isLoading) {
    return (
      <SidebarMenu>
        <SidebarMenuItem>
          <SidebarMenuButton
            size="lg"
            className="animate-pulse"
          >
            <div className="flex aspect-square size-8 items-center justify-center rounded-lg bg-sidebar-accent/10">
              <Notebook className="size-4 text-muted-foreground/50" />
            </div>
            <div className="grid flex-1 gap-1">
              <div className="h-4 w-24 rounded bg-sidebar-accent/10" />
              <div className="h-3 w-16 rounded bg-sidebar-accent/10" />
            </div>
          </SidebarMenuButton>
        </SidebarMenuItem>
      </SidebarMenu>
    )
  }

  if (!studies || studies.length === 0) {
    return (
      <SidebarMenu>
        <SidebarMenuItem>
          <SidebarMenuButton
            size="lg"
            onClick={handleCreateStudy}
            className="data-[state=open]:bg-sidebar-accent data-[state=open]:text-sidebar-accent-foreground"
          >
            <div className="flex aspect-square size-8 items-center justify-center rounded-lg bg-sidebar-primary text-sidebar-primary-foreground">
              <Plus className="size-4" />
            </div>
            <div className="grid flex-1 text-left text-sm leading-tight">
              <span className="truncate font-semibold">Create Study</span>
              <span className="truncate text-xs">Get started</span>
            </div>
          </SidebarMenuButton>
        </SidebarMenuItem>
      </SidebarMenu>
    )
  }

  return (
    <SidebarMenu>
      <SidebarMenuItem>
        <DropdownMenu>
          <DropdownMenuTrigger asChild>
            <SidebarMenuButton
              size="lg"
              className="data-[state=open]:bg-sidebar-accent data-[state=open]:text-sidebar-accent-foreground"
            >
              <div className="flex aspect-square size-8 items-center justify-center rounded-lg bg-sidebar-primary text-sidebar-primary-foreground">
                <Notebook className="size-4" />
              </div>
              <div className="grid flex-1 text-left text-sm leading-tight">
                <span className="truncate font-semibold">
                  {activeStudy?.title ?? "Select Study"}
                </span>
                <span className="truncate text-xs">{activeStudy?.role ?? ""}</span>
              </div>
              <ChevronsUpDown className="ml-auto size-4" />
            </SidebarMenuButton>
          </DropdownMenuTrigger>
          <DropdownMenuContent
            className="w-[--radix-dropdown-menu-trigger-width] min-w-56 rounded-lg"
            align="start"
            side={isMobile ? "bottom" : "right"}
            sideOffset={4}
          >
            <DropdownMenuLabel className="text-xs text-muted-foreground">
              Studies
            </DropdownMenuLabel>
            {studies.map((study) => (
              <DropdownMenuItem
                key={study.id}
                onClick={() => setActiveStudy(study)}
                className="gap-2 p-2"
              >
                <div className="flex size-6 items-center justify-center rounded-sm border">
                  <Notebook className="size-4 shrink-0" />
                </div>
                <div className="flex-1">
                  <p>{study.title}</p>
                  <p className="text-xs text-muted-foreground">{study.role}</p>
                </div>
              </DropdownMenuItem>
            ))}
            <DropdownMenuSeparator />
            <DropdownMenuItem
              onClick={handleCreateStudy}
              className="gap-2 p-2"
            >
              <div className="flex size-6 items-center justify-center rounded-md">
                <Plus className="size-4 text-muted-foreground" />
              </div>
              <div className="font-medium text-muted-foreground">Create new study</div>
            </DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      </SidebarMenuItem>
    </SidebarMenu>
  )
} 