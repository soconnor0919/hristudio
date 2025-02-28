"use client"

import { Separator } from "~/components/ui/separator"
import { SidebarTrigger } from "~/components/ui/sidebar"
import { BreadcrumbNav } from "./breadcrumb-nav"
import { Logo } from "~/components/logo"

export function Header() {
  return (
    <div className="sticky top-0 z-40 w-full">
      <header
        data-nav="header"
        className="mx-2 mt-2 flex h-14 items-center justify-between rounded-lg border shadow-sm md:ml-0 px-6"
      >
        <div className="flex items-center gap-2">
          <SidebarTrigger className="-ml-2 text-[hsl(var(--sidebar-foreground))] hover:bg-[hsl(var(--sidebar-hover))]/20" />
          <Separator orientation="vertical" className="h-4 bg-[hsl(var(--sidebar-border))]" />
          <BreadcrumbNav />
        </div>
        <Logo
          href="/dashboard"
          className="text-[hsl(var(--sidebar-foreground))]"
          iconClassName="text-[hsl(var(--sidebar-muted))]"
        />
      </header>
    </div>
  )
} 