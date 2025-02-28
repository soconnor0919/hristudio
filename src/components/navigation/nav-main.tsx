"use client"

import { usePathname } from "next/navigation"
import { type LucideIcon } from "lucide-react"
import Link from "next/link"

import {
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
} from "~/components/ui/sidebar"
import { cn } from "~/lib/utils"

interface NavItem {
  title: string
  url: string
  icon: LucideIcon
}

export function NavMain({ items }: { items: NavItem[] }) {
  const pathname = usePathname()

  // Find the most specific matching route
  const activeItem = items
    .filter(item => {
      if (item.url === "/dashboard") {
        return pathname === "/dashboard"
      }
      return pathname.startsWith(item.url)
    })
    .sort((a, b) => b.url.length - a.url.length)[0]

  return (
    <SidebarMenu className="pt-2">
      {items.map((item) => {
        const isActive = item.url === activeItem?.url

        return (
          <SidebarMenuItem key={item.url}>
            <SidebarMenuButton
              asChild
              isActive={isActive}
              tooltip={item.title}
              className={cn(
                "relative flex w-full items-center gap-2 rounded-md px-3 py-2 text-sm outline-none transition-colors",
                "hover:bg-sidebar-accent hover:text-sidebar-accent-foreground",
                "focus-visible:ring-2 focus-visible:ring-sidebar-ring",
                "group-data-[collapsible=icon]:px-0 group-data-[collapsible=icon]:justify-center",
                isActive && "bg-sidebar-accent font-medium text-sidebar-accent-foreground"
              )}
            >
              <Link href={item.url} className="flex items-center gap-2 w-full group-data-[collapsible=icon]:justify-center">
                <item.icon className="h-4 w-4 shrink-0" />
                <span className="truncate group-data-[collapsible=icon]:hidden">{item.title}</span>
              </Link>
            </SidebarMenuButton>
          </SidebarMenuItem>
        )
      })}
    </SidebarMenu>
  )
}