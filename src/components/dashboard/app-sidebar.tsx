"use client";

import React, { useEffect } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { signOut, useSession } from "next-auth/react";
import {
  BarChart3,
  Building,
  ChevronDown,
  FlaskConical,
  Home,
  LogOut,
  MoreHorizontal,
  Settings,
  User,
  Users,
  UserCheck,
  TestTube,
} from "lucide-react";

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import {
  Sidebar,
  SidebarContent,
  SidebarFooter,
  SidebarGroup,
  SidebarGroupContent,
  SidebarGroupLabel,
  SidebarHeader,
  SidebarMenu,
  SidebarMenuButton,
  SidebarMenuItem,
  SidebarRail,
} from "~/components/ui/sidebar";
import { Logo } from "~/components/ui/logo";

import { useStudyManagement } from "~/hooks/useStudyManagement";

// Navigation items
const navigationItems = [
  {
    title: "Overview",
    url: "/dashboard",
    icon: Home,
  },
  {
    title: "Studies",
    url: "/studies",
    icon: Building,
  },
  {
    title: "Experiments",
    url: "/experiments",
    icon: FlaskConical,
  },
  {
    title: "Participants",
    url: "/participants",
    icon: Users,
  },
  {
    title: "Trials",
    url: "/trials",
    icon: TestTube,
  },
  {
    title: "Analytics",
    url: "/analytics",
    icon: BarChart3,
  },
];

const adminItems = [
  {
    title: "Administration",
    url: "/admin",
    icon: UserCheck,
  },
];

interface AppSidebarProps extends React.ComponentProps<typeof Sidebar> {
  userRole?: string;
}

export function AppSidebar({
  userRole = "researcher",
  ...props
}: AppSidebarProps) {
  const { data: session } = useSession();
  const pathname = usePathname();
  const isAdmin = userRole === "administrator";
  const { selectedStudyId, userStudies, selectStudy, refreshStudyData } =
    useStudyManagement();

  type Study = {
    id: string;
    name: string;
  };

  // Filter navigation items based on study selection
  const availableNavigationItems = navigationItems.filter((item) => {
    // These items are always available
    if (item.url === "/dashboard" || item.url === "/studies") {
      return true;
    }
    // These items require a selected study
    return selectedStudyId !== null;
  });

  const handleSignOut = async () => {
    await signOut({ callbackUrl: "/" });
  };

  const handleStudySelect = async (studyId: string) => {
    try {
      await selectStudy(studyId);
    } catch (error) {
      console.error("Failed to select study:", error);
      // If study selection fails (e.g., study not found), clear the selection
      await selectStudy(null);
    }
  };

  const selectedStudy = userStudies.find(
    (study: Study) => study.id === selectedStudyId,
  );

  // If we have a selectedStudyId but can't find the study, clear the selection
  React.useEffect(() => {
    if (selectedStudyId && userStudies.length > 0 && !selectedStudy) {
      console.warn(
        "Selected study not found in user studies, clearing selection",
      );
      void selectStudy(null);
    }
  }, [selectedStudyId, userStudies, selectedStudy, selectStudy]);

  // Auto-refresh studies list when component mounts to catch external changes
  useEffect(() => {
    const interval = setInterval(() => {
      void refreshStudyData();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refreshStudyData]);

  return (
    <Sidebar collapsible="icon" variant="sidebar" {...props}>
      <SidebarHeader>
        <SidebarMenu>
          <SidebarMenuItem>
            <SidebarMenuButton size="lg" asChild>
              <Link href="/dashboard">
                <Logo iconSize="md" showText={true} />
              </Link>
            </SidebarMenuButton>
          </SidebarMenuItem>
        </SidebarMenu>
      </SidebarHeader>

      <SidebarContent>
        {/* Study Selector */}
        <SidebarGroup>
          <SidebarGroupLabel>Active Study</SidebarGroupLabel>
          <SidebarGroupContent>
            <SidebarMenu>
              <SidebarMenuItem>
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <SidebarMenuButton className="w-full">
                      <Building className="h-4 w-4 flex-shrink-0" />
                      <span className="truncate">
                        {selectedStudy?.name ?? "Select Study"}
                      </span>
                      <ChevronDown className="ml-auto h-4 w-4 flex-shrink-0" />
                    </SidebarMenuButton>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent
                    className="w-[--radix-popper-anchor-width]"
                    align="start"
                  >
                    <DropdownMenuLabel>Studies</DropdownMenuLabel>
                    {userStudies.map((study: Study) => (
                      <DropdownMenuItem
                        key={study.id}
                        onClick={() => handleStudySelect(study.id)}
                        className="cursor-pointer"
                      >
                        <Building className="mr-2 h-4 w-4 flex-shrink-0" />
                        <span className="truncate" title={study.name}>
                          {study.name}
                        </span>
                      </DropdownMenuItem>
                    ))}
                    <DropdownMenuSeparator />
                    {selectedStudyId && (
                      <DropdownMenuItem
                        onClick={async () => {
                          await selectStudy(null);
                        }}
                      >
                        <Building className="mr-2 h-4 w-4 opacity-50" />
                        Clear selection
                      </DropdownMenuItem>
                    )}
                    <DropdownMenuItem asChild>
                      <Link href="/studies/new">
                        <Building className="mr-2 h-4 w-4" />
                        Create study
                      </Link>
                    </DropdownMenuItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </SidebarMenuItem>
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>

        {/* Main Navigation */}
        <SidebarGroup>
          <SidebarGroupLabel>Research</SidebarGroupLabel>
          <SidebarGroupContent>
            <SidebarMenu>
              {availableNavigationItems.map((item) => {
                const isActive =
                  pathname === item.url ||
                  (item.url !== "/dashboard" && pathname.startsWith(item.url));

                return (
                  <SidebarMenuItem key={item.title}>
                    <SidebarMenuButton asChild isActive={isActive}>
                      <Link href={item.url}>
                        <item.icon className="h-4 w-4" />
                        <span>{item.title}</span>
                      </Link>
                    </SidebarMenuButton>
                  </SidebarMenuItem>
                );
              })}
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>

        {/* Study-specific items hint */}
        {!selectedStudyId && (
          <SidebarGroup>
            <SidebarGroupContent>
              <div className="text-muted-foreground px-3 py-2 text-xs">
                Select a study to access experiments, participants, trials, and
                analytics.
              </div>
            </SidebarGroupContent>
          </SidebarGroup>
        )}

        {/* Admin Section */}
        {isAdmin && (
          <SidebarGroup>
            <SidebarGroupLabel>Administration</SidebarGroupLabel>
            <SidebarGroupContent>
              <SidebarMenu>
                {adminItems.map((item) => {
                  const isActive = pathname.startsWith(item.url);

                  return (
                    <SidebarMenuItem key={item.title}>
                      <SidebarMenuButton asChild isActive={isActive}>
                        <Link href={item.url}>
                          <item.icon className="h-4 w-4" />
                          <span>{item.title}</span>
                        </Link>
                      </SidebarMenuButton>
                    </SidebarMenuItem>
                  );
                })}
              </SidebarMenu>
            </SidebarGroupContent>
          </SidebarGroup>
        )}
      </SidebarContent>

      <SidebarFooter>
        <SidebarMenu>
          <SidebarMenuItem>
            <DropdownMenu>
              <DropdownMenuTrigger asChild>
                <SidebarMenuButton size="lg">
                  <User className="h-4 w-4" />
                  <span>{session?.user?.name ?? "User"}</span>
                  <MoreHorizontal className="ml-auto h-4 w-4" />
                </SidebarMenuButton>
              </DropdownMenuTrigger>
              <DropdownMenuContent
                className="w-[--radix-popper-anchor-width]"
                align="end"
              >
                <DropdownMenuLabel>
                  {session?.user?.name ?? "User"}
                </DropdownMenuLabel>
                <DropdownMenuSeparator />
                <DropdownMenuItem asChild>
                  <Link href="/profile">
                    <Settings className="mr-2 h-4 w-4" />
                    Profile & Settings
                  </Link>
                </DropdownMenuItem>
                <DropdownMenuSeparator />
                <DropdownMenuItem onClick={handleSignOut}>
                  <LogOut className="mr-2 h-4 w-4" />
                  Sign out
                </DropdownMenuItem>
              </DropdownMenuContent>
            </DropdownMenu>
          </SidebarMenuItem>
        </SidebarMenu>
      </SidebarFooter>

      <SidebarRail />
    </Sidebar>
  );
}
