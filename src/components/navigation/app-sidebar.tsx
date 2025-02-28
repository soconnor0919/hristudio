"use client"

import {
  Home,
  Settings2,
  Microscope,
  Users,
  FlaskConical,
  Bot
} from "lucide-react"
import * as React from "react"
import { useSession } from "next-auth/react"
import { useStudy } from "~/components/providers/study-provider"

import { StudySwitcher } from "~/components/auth/study-switcher"
import {
  Sidebar,
  SidebarContent,
  SidebarFooter,
  SidebarHeader,
  SidebarRail,
} from "~/components/ui/sidebar"
import { NavMain } from "~/components/navigation/nav-main"
import { NavUser } from "~/components/navigation/nav-user"

export function AppSidebar({ ...props }: React.ComponentProps<typeof Sidebar>) {
  const { data: session } = useSession()
  const { activeStudy } = useStudy()

  if (!session) return null

  // Base navigation items that are always shown
  const baseNavItems = [
    {
      title: "Overview",
      url: "/dashboard",
      icon: Home,
    },
    {
      title: "Studies",
      url: "/dashboard/studies",
      icon: Microscope,
    },
    {
      title: "Robot Store",
      url: "/dashboard/store",
      icon: Bot,
    },
  ]

  // Study-specific navigation items that are only shown when a study is active
  const studyNavItems = activeStudy
    ? [
        {
          title: "Participants",
          url: `/dashboard/studies/${activeStudy.id}/participants`,
          icon: Users,
        },
        {
          title: "Experiments",
          url: `/dashboard/studies/${activeStudy.id}/experiments`,
          icon: FlaskConical,
        },
      ]
    : []

  // Settings navigation items
  const settingsNavItems = [
    {
      title: "Settings",
      url: "/dashboard/settings",
      icon: Settings2,
    }
  ]

  const navItems = [...baseNavItems, ...studyNavItems, ...settingsNavItems]

  return (
    <Sidebar 
      collapsible="icon" 
      variant="floating"
      className="border-none"
      {...props}
    >
      <SidebarHeader>
        <StudySwitcher />
      </SidebarHeader>
      <SidebarContent>
        <NavMain items={navItems} />
      </SidebarContent>
      <SidebarFooter>
        <NavUser />
      </SidebarFooter>
      <SidebarRail />
    </Sidebar>
  )
}
