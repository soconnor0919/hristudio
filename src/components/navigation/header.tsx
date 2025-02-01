"use client"

import { Separator } from "~/components/ui/separator"
import { SidebarTrigger } from "~/components/ui/sidebar"
import { BreadcrumbNav } from "./breadcrumb-nav"
import { Logo } from "~/components/logo"

export function Header() {
  return (
    <div className="sticky top-0 z-40 w-full">
      <header className="mx-2 mt-2 flex h-14 items-center justify-between rounded-lg border bg-gradient-to-r from-[hsl(var(--sidebar-gradient-from))] to-[hsl(var(--sidebar-gradient-to))] px-6 shadow-sm md:ml-0">
        <div className="flex items-center gap-2">
          <SidebarTrigger className="-ml-2 text-[hsl(var(--sidebar-text))] hover:bg-[hsl(var(--sidebar-text))]/10" />
          <Separator orientation="vertical" className="h-4 bg-[hsl(var(--sidebar-text))]/10" />
          <BreadcrumbNav />
        </div>
        <Logo 
          href="/dashboard"
          className="text-[hsl(var(--sidebar-text))]"
          iconClassName="text-[hsl(var(--sidebar-text-muted))]"
        />
      </header>
    </div>
  )
} 