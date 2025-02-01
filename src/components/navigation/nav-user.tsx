"use client"

import { ChevronsUpDown, LogOut, Settings, User } from "lucide-react"
import { useSession } from "next-auth/react"
import Link from "next/link"
import Image from "next/image"

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuGroup,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu"
import {
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
} from "~/components/ui/sidebar"
import { Avatar, AvatarFallback } from "~/components/ui/avatar"

export function NavUser() {
  const { data: session, status } = useSession()

  if (status === "loading") {
    return (
      <SidebarMenu>
        <SidebarMenuItem>
          <SidebarMenuButton
            size="lg"
            className="animate-pulse"
          >
            <div className="flex aspect-square size-8 items-center justify-center rounded-lg bg-sidebar-accent/10">
              <User className="size-4 text-muted-foreground/50" />
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

  if (!session?.user) {
    return null
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
              <Avatar className="size-8 rounded-lg">
                {session.user.image ? (
                  <div className="relative size-full overflow-hidden rounded-lg">
                    <Image
                      src={session.user.image}
                      alt={session.user.name ?? "User"}
                      fill
                      sizes="32px"
                      className="object-cover"
                      onError={(e) => {
                        console.error("Error loading nav avatar:", session.user.image);
                        e.currentTarget.style.display = "none";
                      }}
                    />
                  </div>
                ) : (
                  <AvatarFallback className="rounded-lg bg-sidebar-muted">
                    <User className="size-4" />
                  </AvatarFallback>
                )}
              </Avatar>
              <div className="grid flex-1 text-left text-sm leading-tight">
                <span className="truncate font-semibold">
                  {session.user.name ?? "User"}
                </span>
                <span className="truncate text-xs text-sidebar-muted">
                  {session.user.email}
                </span>
              </div>
              <ChevronsUpDown className="ml-auto size-4" />
            </SidebarMenuButton>
          </DropdownMenuTrigger>
          <DropdownMenuContent
            className="w-[--radix-dropdown-menu-trigger-width] min-w-56 rounded-lg"
            align="end"
            sideOffset={4}
          >
            <DropdownMenuLabel className="font-normal">
              <div className="flex items-center gap-2 px-1 py-1.5">
                <Avatar className="size-8 rounded-lg">
                  {session.user.image ? (
                    <div className="relative size-full overflow-hidden rounded-lg">
                      <Image
                        src={session.user.image}
                        alt={session.user.name ?? "User"}
                        fill
                        sizes="32px"
                        className="object-cover"
                        onError={(e) => {
                          console.error("Error loading dropdown avatar:", session.user.image);
                          e.currentTarget.style.display = "none";
                        }}
                      />
                    </div>
                  ) : (
                    <AvatarFallback className="rounded-lg">
                      <User className="size-4" />
                    </AvatarFallback>
                  )}
                </Avatar>
                <div className="grid flex-1 text-left text-sm leading-tight">
                  <span className="font-semibold">
                    {session.user.name ?? "User"}
                  </span>
                  <span className="text-xs text-muted-foreground">
                    {session.user.email}
                  </span>
                </div>
              </div>
            </DropdownMenuLabel>
            <DropdownMenuSeparator />
            <DropdownMenuGroup>
              <DropdownMenuItem asChild>
                <Link href="/dashboard/account">
                  <Settings className="mr-2 size-4" />
                  Settings
                </Link>
              </DropdownMenuItem>
            </DropdownMenuGroup>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href="/api/auth/signout">
                <LogOut className="mr-2 size-4" />
                Sign out
              </Link>
            </DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      </SidebarMenuItem>
    </SidebarMenu>
  )
}
