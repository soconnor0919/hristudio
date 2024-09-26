"use client"

import { UserButton, useUser } from "@clerk/nextjs"
import {
  BarChartIcon,
  BeakerIcon,
  BotIcon,
  FolderIcon,
  LayoutDashboard,
  Menu,
  Settings
} from "lucide-react"
import Link from "next/link"
import { usePathname } from "next/navigation"
import { useState } from "react"
import { Button } from "~/components/ui/button"
import { Sheet, SheetContent, SheetTrigger } from "~/components/ui/sheet"
import { cn } from "~/lib/utils"

const navItems = [
  { name: "Dashboard", href: "/dash", icon: LayoutDashboard },
  { name: "Studies", href: "/studies", icon: FolderIcon },
  { name: "Participants", href: "/participants", icon: BeakerIcon },
  { name: "Data Analysis", href: "/analysis", icon: BarChartIcon },
  { name: "Settings", href: "/settings", icon: Settings },
];

export function Sidebar() {
  const pathname = usePathname()
  const [isOpen, setIsOpen] = useState(false)
  const { user } = useUser()

  const SidebarContent = () => (
    <div className="flex h-full flex-col lg:bg-blue-50">
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
                    "w-full justify-start text-blue-800 hover:bg-blue-100",
                    pathname === item.href && "bg-blue-100 font-semibold"
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
      <div className="border-t border-blue-200 p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <UserButton />
            <div>
              <p className="text-sm font-medium text-blue-800">{user?.fullName ?? 'User'}</p>
              <p className="text-xs text-blue-600">{user?.primaryEmailAddress?.emailAddress ?? 'user@example.com'}</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  )

  const HRIStudioLogo = () => (
    <Link href="/dash" className="flex items-center font-sans text-xl text-blue-800">
      <BotIcon className="h-6 w-6 mr-1 text-blue-600" />
      <span className="font-extrabold">HRI</span>
      <span className="font-normal">Studio</span>
    </Link>
  )

  return (
    <>
      <div className="lg:hidden fixed top-0 left-0 right-0">
        <div className="flex h-14 items-center justify-between border-b px-4 bg-background">
          <HRIStudioLogo />
          <Sheet open={isOpen} onOpenChange={setIsOpen}>
            <SheetTrigger asChild>
              <Button variant="ghost" className="h-14 w-14 px-0">
                <Menu className="h-6 w-6" />
              </Button>
            </SheetTrigger>
            <SheetContent side="top" className="w-full">
              <SidebarContent />
            </SheetContent>
          </Sheet>
        </div>
      </div>
      <div className="hidden lg:flex lg:w-64 lg:flex-col lg:border-r lg:bg-background">
        <div className="flex h-14 items-center border-b px-4">
          <HRIStudioLogo />
        </div>
        <SidebarContent />
      </div>
    </>
  )
}