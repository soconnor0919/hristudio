"use client"

import {
  Beaker,
  Home,
  Settings2,
  User
} from "lucide-react"
import * as React from "react"
import { useSession } from "next-auth/react"

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

const data = {
  navMain: [
    {
      title: "Overview",
      url: "/dashboard",
      icon: Home,
      isActive: true,
    },
    {
      title: "Studies",
      url: "/dashboard/studies",
      icon: Beaker,
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
  ],
}

export function AppSidebar({ ...props }: React.ComponentProps<typeof Sidebar>) {
  const { data: session } = useSession()
  if (!session) return null

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
        <NavMain items={data.navMain} />
      </SidebarContent>
      <SidebarFooter>
        <NavUser />
      </SidebarFooter>
      <SidebarRail />
    </Sidebar>
  )
}
