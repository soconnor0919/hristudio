"use client"

import {
  Beaker,
  Home,
  Settings2,
  User,
  Microscope,
  Users,
  Plus
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
      items: [
        {
          title: "All Studies",
          url: "/dashboard/studies",
        },
        {
          title: "Create Study",
          url: "/dashboard/studies/new",
        },
      ],
    },
  ]

  // Study-specific navigation items that are only shown when a study is active
  const studyNavItems = activeStudy
    ? [
        {
          title: "Participants",
          url: `/dashboard/studies/${activeStudy.id}/participants`,
          icon: Users,
          items: [
            {
              title: "All Participants",
              url: `/dashboard/studies/${activeStudy.id}/participants`,
            },
            {
              title: "Add Participant",
              url: `/dashboard/studies/${activeStudy.id}/participants/new`,
              // Only show if user is admin
              hidden: activeStudy.role !== "ADMIN",
            },
          ],
        },
      ]
    : []

  // Settings navigation items
  const settingsNavItems = [
    {
      title: "Settings",
      url: "/dashboard/settings",
      icon: Settings2,
      items: [
        {
          title: "Account",
          url: "/dashboard/account",
          icon: User,
        },
        {
          title: "Team",
          url: "/dashboard/settings/team",
        },
        {
          title: "Billing",
          url: "/dashboard/settings/billing",
        },
      ],
    },
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
