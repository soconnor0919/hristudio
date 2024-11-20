'use client';

import { UserButton, useUser } from "@clerk/nextjs"
import {
  BarChartIcon,
  UsersRoundIcon,
  LandPlotIcon,
  BotIcon,
  FolderIcon,
  FileTextIcon,
  LayoutDashboard,
  Menu,
  Settings
} from "lucide-react"
import Link from "next/link"
import { usePathname } from "next/navigation"
import { useState } from "react"
import { Button } from "~/components/ui/button"
import { Sheet, SheetContent, SheetTrigger, SheetTitle } from "~/components/ui/sheet"
import { cn } from "~/lib/utils"

const navItems = [
  { name: "Dashboard", href: "/dashboard", icon: LayoutDashboard },
  { name: "Studies", href: "/dashboard/studies", icon: FolderIcon },
  { name: "Participants", href: "/dashboard/participants", icon: UsersRoundIcon },
  { name: "Trials", href: "/dashboard/trials", icon: LandPlotIcon },
  { name: "Forms", href: "/dashboard/forms", icon: FileTextIcon },
  { name: "Data Analysis", href: "/dashboard/analysis", icon: BarChartIcon },
  { name: "Settings", href: "/dashboard/settings", icon: Settings },
];

export function Sidebar() {
  const pathname = usePathname()
  const [isOpen, setIsOpen] = useState(false)
  const { user } = useUser()

  const HRIStudioLogo = () => (
    <Link href="/dashboard" className="flex items-center font-sans text-xl text-[hsl(var(--sidebar-foreground))]">
      <BotIcon className="h-6 w-6 mr-1 text-[hsl(var(--sidebar-muted))]" />
      <span className="font-extrabold">HRI</span>
      <span className="font-normal">Studio</span>
    </Link>
  )

  const SidebarContent = () => (
    <div className="flex h-full flex-col bg-gradient-to-b from-[hsl(var(--sidebar-background-top))] to-[hsl(var(--sidebar-background-bottom))]">
      <nav className="flex-1 overflow-y-auto p-4">
        <ul className="space-y-2">
          {navItems.map((item) => {
            const IconComponent = item.icon;
            return (
              <li key={item.href}>
                <Button
                  asChild
                  variant="ghost"
                  className={cn(
                    "w-full justify-start text-[hsl(var(--sidebar-foreground))] hover:bg-[hsl(var(--sidebar-hover))]",
                    pathname === item.href && "bg-[hsl(var(--sidebar-hover))] font-semibold"
                  )}
                >
                  <Link href={item.href} onClick={() => setIsOpen(false)}>
                    <IconComponent className="h-5 w-5 mr-3" />
                    <span>{item.name}</span>
                  </Link>
                </Button>
              </li>
            );
          })}
        </ul>
      </nav>
      <div className="border-t p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <UserButton />
            <div>
              <p className="text-sm font-medium text-[hsl(var(--sidebar-foreground))]">{user?.fullName ?? 'User'}</p>
              <p className="text-xs text-[hsl(var(--sidebar-muted))]">{user?.primaryEmailAddress?.emailAddress ?? 'user@example.com'}</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  )

  return (
    <>
      <div className="lg:hidden fixed top-0 left-0 right-0 z-50">
        <div className="flex h-14 items-center justify-between border-b px-4 bg-background">
          <HRIStudioLogo />
          <Sheet open={isOpen} onOpenChange={setIsOpen}>
            <SheetTrigger asChild>
              <Button variant="ghost" className="h-14 w-14 px-0">
                <Menu className="h-6 w-6" />
              </Button>
            </SheetTrigger>
            <SheetContent side="top" className="w-full">
              <SheetTitle className="sr-only">Navigation Menu</SheetTitle>
              <SidebarContent />
            </SheetContent>
          </Sheet>
        </div>
      </div>
      <div className="hidden lg:flex lg:w-64 lg:flex-col lg:border-r lg:bg-gradient-to-b lg:from-[hsl(var(--sidebar-background-top))] lg:to-[hsl(var(--sidebar-background-bottom))]">
        <div className="flex h-14 items-center border-b px-4">
          <HRIStudioLogo />
        </div>
        <SidebarContent />
      </div>
    </>
  )
}