'use client';

import { UserButton, useUser } from "@clerk/nextjs"
import {
  BarChartIcon,
  UsersRoundIcon,
  LandPlotIcon,
  FileTextIcon,
  LayoutDashboard,
  Menu,
  Settings,
  ChevronDown,
  FolderIcon,
  PlusIcon
} from "lucide-react"
import Link from "next/link"
import { usePathname, useRouter } from "next/navigation"
import { useState } from "react"
import { Button } from "~/components/ui/button"
import { Sheet, SheetContent, SheetTrigger, SheetTitle } from "~/components/ui/sheet"
import { cn } from "~/lib/utils"
import { Logo } from "~/components/logo"
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select"
import { Separator } from "~/components/ui/separator"
import { useActiveStudy } from "~/context/active-study"

const getNavItems = (studyId?: number) => [
  { 
    name: "Dashboard", 
    href: studyId ? `/dashboard/studies/${studyId}` : "/dashboard", 
    icon: LayoutDashboard,
    exact: true,
    requiresStudy: false
  },
  {
    name: "Studies",
    href: "/dashboard/studies",
    icon: FolderIcon,
    exact: true,
    requiresStudy: false,
    hideWithStudy: true
  },
  {
    name: "Participants", 
    href: `/dashboard/studies/${studyId}/participants`, 
    icon: UsersRoundIcon,
    requiresStudy: true,
    baseRoute: "participants"
  },
  { 
    name: "Trials", 
    href: `/dashboard/studies/${studyId}/trials`, 
    icon: LandPlotIcon,
    requiresStudy: true,
    baseRoute: "trials"
  },
  { 
    name: "Forms", 
    href: `/dashboard/studies/${studyId}/forms`, 
    icon: FileTextIcon,
    requiresStudy: true,
    baseRoute: "forms"
  },
  { 
    name: "Data Analysis", 
    href: `/dashboard/studies/${studyId}/analysis`, 
    icon: BarChartIcon,
    requiresStudy: true,
    baseRoute: "analysis"
  },
  { 
    name: "Settings", 
    href: `/dashboard/studies/${studyId}/settings`, 
    icon: Settings,
    requiresStudy: true,
    baseRoute: "settings"
  },
];

export function Sidebar() {
  const pathname = usePathname()
  const router = useRouter()
  const [isOpen, setIsOpen] = useState(false)
  const { user } = useUser()
  const { activeStudy, setActiveStudy, studies, isLoading } = useActiveStudy()

  const navItems = getNavItems(activeStudy?.id)
  const visibleNavItems = activeStudy 
    ? navItems.filter(item => !item.hideWithStudy)
    : navItems.filter(item => !item.requiresStudy)

  const isActiveRoute = (item: { href: string, exact?: boolean, baseRoute?: string }) => {
    if (item.exact) {
      return pathname === item.href;
    }
    if (item.baseRoute && activeStudy) {
      const pattern = new RegExp(`/dashboard/studies/\\d+/${item.baseRoute}`);
      return pattern.test(pathname);
    }
    return pathname.startsWith(item.href);
  };

  const handleStudyChange = (value: string) => {
    if (value === "all") {
      setActiveStudy(null);
      router.push("/dashboard");
    } else {
      const study = studies.find(s => s.id.toString() === value);
      if (study) {
        setActiveStudy(study);
        router.push(`/dashboard/studies/${study.id}`);
      }
    }
  };

  const SidebarContent = () => (
    <div className="flex h-full flex-col">
      <div className="p-4">
        <Select
          value={activeStudy?.id?.toString() || "all"}
          onValueChange={handleStudyChange}
        >
          <SelectTrigger className="w-full sidebar-button">
            <div className="flex items-center justify-between">
              <span className="truncate">
                {activeStudy?.title || "All Studies"}
              </span>
            </div>
          </SelectTrigger>
          <SelectContent className="sidebar-dropdown-content">
            <SelectItem value="all" className="sidebar-button">
              <div className="flex items-center">
                <FolderIcon className="h-4 w-4 mr-2" />
                All Studies
              </div>
            </SelectItem>
            <Separator className="sidebar-separator" />
            {studies.map((study) => (
              <SelectItem 
                key={study.id} 
                value={study.id.toString()}
                className="sidebar-button"
              >
                {study.title}
              </SelectItem>
            ))}
            <Separator className="sidebar-separator" />
            <Button
              variant="ghost"
              className="w-full justify-start sidebar-button"
              asChild
            >
              <Link href="/dashboard/studies/new">
                <PlusIcon className="h-4 w-4 mr-2" />
                Create New Study
              </Link>
            </Button>
          </SelectContent>
        </Select>
      </div>

      <nav className="flex-1 overflow-y-auto p-4">
        <ul className="space-y-2">
          {visibleNavItems.map((item) => {
            const IconComponent = item.icon;
            const isActive = isActiveRoute(item);
            
            return (
              <li key={item.href}>
                <Button
                  asChild
                  variant="ghost"
                  className="w-full justify-start sidebar-button"
                  data-active={isActive}
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

      <div className="p-4">
        <div className="border-t border-[hsl(var(--sidebar-separator))]">
          <div className="flex items-center justify-between pt-4">
            <div className="flex items-center space-x-4">
              <div className="w-8 h-8">
                <UserButton afterSignOutUrl="/" />
              </div>
              {user && (
                <div className="min-w-0">
                  <p className="text-sm font-medium text-[hsl(var(--sidebar-foreground))] truncate">
                    {user.fullName ?? user.username ?? 'User'}
                  </p>
                  <p className="text-xs text-[hsl(var(--sidebar-muted))] truncate">
                    {user.primaryEmailAddress?.emailAddress ?? 'user@example.com'}
                  </p>
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  )

  return (
    <>
      <div className="lg:hidden fixed top-0 left-0 right-0 z-50">
        <div className="flex h-14 items-center justify-between border-b border-[hsl(var(--sidebar-border))]">
          <Logo 
            href="/dashboard"
            className="text-[hsl(var(--sidebar-foreground))]"
            iconClassName="text-[hsl(var(--sidebar-muted))]"
          />
          <Sheet open={isOpen} onOpenChange={setIsOpen}>
            <SheetTrigger asChild>
              <Button variant="ghost" className="h-14 w-14 px-0 sidebar-button">
                <Menu className="h-6 w-6" />
              </Button>
            </SheetTrigger>
            <SheetContent 
              side="left" 
              className="w-full p-0 border-[hsl(var(--sidebar-border))]"
            >
              <SheetTitle className="sr-only">Navigation Menu</SheetTitle>
              <SidebarContent />
            </SheetContent>
          </Sheet>
        </div>
      </div>
      <div className="hidden lg:flex lg:w-64 lg:flex-col lg:border-r lg:border-[hsl(var(--sidebar-border))]">
        <div className="flex h-14 items-center border-b border-[hsl(var(--sidebar-border))] px-4">
          <Logo 
            href="/dashboard"
            className="text-[hsl(var(--sidebar-foreground))]"
            iconClassName="text-[hsl(var(--sidebar-muted))]"
          />
        </div>
        <SidebarContent />
      </div>
    </>
  )
}