"use client";

import React, { useEffect } from "react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { signOut, useSession } from "next-auth/react";
import { toast } from "sonner";
import {
  BarChart3,
  Building,
  ChevronDown,
  FlaskConical,
  Home,
  LogOut,
  MoreHorizontal,
  Puzzle,
  Settings,
  TestTube,
  User,
  UserCheck,
  Users,
} from "lucide-react";

import { useSidebar } from "~/components/ui/sidebar";

import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "~/components/ui/tooltip";
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
import { Avatar, AvatarImage, AvatarFallback } from "~/components/ui/avatar";
import { Logo } from "~/components/ui/logo";

import { useStudyManagement } from "~/hooks/useStudyManagement";
import { handleAuthError, isAuthError } from "~/lib/auth-error-handler";
import { api } from "~/trpc/react";

// Global items - always available
const globalItems = [
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
    title: "Profile",
    url: "/profile",
    icon: User,
  },
];

// Current Study Work section - only shown when study is selected
const studyWorkItems = [
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
    title: "Experiments",
    url: "/experiments",
    icon: FlaskConical,
  },
  {
    title: "Analytics",
    url: "/analytics",
    icon: BarChart3,
  },
  {
    title: "Plugins",
    url: "/plugins",
    icon: Puzzle,
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
  const { state: sidebarState } = useSidebar();
  const isCollapsed = sidebarState === "collapsed";
  const { selectedStudyId, userStudies, selectStudy, refreshStudyData } =
    useStudyManagement();

  // Debug API call
  const { data: debugData } = api.dashboard.debug.useQuery(undefined, {
    enabled: process.env.NODE_ENV === "development",
    staleTime: 1000 * 30, // 30 seconds
  });

  type Study = {
    id: string;
    name: string;
  };

  // Build study work items with proper URLs when study is selected
  const studyWorkItemsWithUrls = selectedStudyId
    ? studyWorkItems.map((item) => ({
      ...item,
      url: `/studies/${selectedStudyId}${item.url}`,
    }))
    : [];

  const handleSignOut = async () => {
    await signOut({ callbackUrl: "/" });
  };

  const handleStudySelect = async (studyId: string) => {
    try {
      await selectStudy(studyId);
    } catch (error) {
      console.error("Failed to select study:", error);
      // Handle auth errors first
      if (isAuthError(error)) {
        await handleAuthError(error, "Session expired while selecting study");
        return;
      }
      // If study selection fails (e.g., study not found), clear the selection
      await selectStudy(null);
    }
  };

  const handleClearStudy = async (event: React.MouseEvent) => {
    try {
      event.preventDefault();
      event.stopPropagation();
      console.log("Clearing study selection...");
      await selectStudy(null);
      console.log("Study selection cleared successfully");
      toast.success("Study selection cleared");
    } catch (error) {
      console.error("Failed to clear study:", error);
      // Handle auth errors first
      if (isAuthError(error)) {
        await handleAuthError(error, "Session expired while clearing study");
        return;
      }
      toast.error("Failed to clear study selection");
    }
  };

  const selectedStudy = userStudies.find(
    (study: Study) => study.id === selectedStudyId,
  );

  // Debug logging for study data
  React.useEffect(() => {
    console.log("Sidebar debug - User studies:", {
      count: userStudies.length,
      studies: userStudies.map((s) => ({ id: s.id, name: s.name })),
      selectedStudyId,
      selectedStudy: selectedStudy
        ? { id: selectedStudy.id, name: selectedStudy.name }
        : null,
    });
  }, [userStudies, selectedStudyId, selectedStudy]);

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
      void (async () => {
        try {
          await refreshStudyData();
        } catch (error) {
          console.error("Failed to refresh study data:", error);
          if (isAuthError(error)) {
            void handleAuthError(error, "Session expired during data refresh");
          }
        }
      })();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refreshStudyData]);

  // Show debug info in development
  const showDebug = process.env.NODE_ENV === "development";

  const [mounted, setMounted] = React.useState(false);

  React.useEffect(() => {
    setMounted(true);
  }, []);

  if (!mounted) {
    return (
      <Sidebar collapsible="icon" variant="sidebar" {...props}>
        <SidebarHeader />
        <SidebarContent />
        <SidebarFooter />
      </Sidebar>
    );
  }

  return (
    <Sidebar collapsible="icon" variant="sidebar" {...props}>
      <SidebarHeader>
        <SidebarMenu>
          <SidebarMenuItem>
            <SidebarMenuButton size="lg" asChild>
              <Link href="/dashboard">
                <Logo iconSize="md" showText={!isCollapsed} />
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
                {isCollapsed ? (
                  <TooltipProvider>
                    <Tooltip>
                      <TooltipTrigger asChild>
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
                              <DropdownMenuItem onClick={handleClearStudy}>
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
                      </TooltipTrigger>
                      <TooltipContent side="right" className="text-sm">
                        {selectedStudy?.name ?? "Select Study"}
                      </TooltipContent>
                    </Tooltip>
                  </TooltipProvider>
                ) : (
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
                        <DropdownMenuItem onClick={handleClearStudy}>
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
                )}
              </SidebarMenuItem>
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>

        {/* Main Navigation */}
        {/* Global Section */}
        <SidebarGroup>
          <SidebarGroupLabel>Platform</SidebarGroupLabel>
          <SidebarGroupContent>
            <SidebarMenu>
              {globalItems.map((item) => {
                const isActive =
                  pathname === item.url ||
                  (item.url !== "/dashboard" && pathname.startsWith(item.url));

                const menuButton = (
                  <SidebarMenuButton asChild isActive={isActive}>
                    <Link href={item.url}>
                      <item.icon className="h-4 w-4" />
                      <span>{item.title}</span>
                    </Link>
                  </SidebarMenuButton>
                );

                return (
                  <SidebarMenuItem key={item.title}>
                    {isCollapsed ? (
                      <TooltipProvider>
                        <Tooltip>
                          <TooltipTrigger asChild>{menuButton}</TooltipTrigger>
                          <TooltipContent side="right" className="text-sm">
                            {item.title}
                          </TooltipContent>
                        </Tooltip>
                      </TooltipProvider>
                    ) : (
                      menuButton
                    )}
                  </SidebarMenuItem>
                );
              })}
            </SidebarMenu>
          </SidebarGroupContent>
        </SidebarGroup>

        {/* Current Study Work Section */}
        {selectedStudyId && selectedStudy ? (
          <SidebarGroup>
            <SidebarGroupLabel>Current Study Work</SidebarGroupLabel>
            <SidebarGroupContent>
              <SidebarMenu>
                {studyWorkItemsWithUrls.map((item) => {
                  const isActive =
                    pathname === item.url ||
                    (item.url !== "/dashboard" &&
                      pathname.startsWith(item.url));

                  const menuButton = (
                    <SidebarMenuButton asChild isActive={isActive}>
                      <Link href={item.url}>
                        <item.icon className="h-4 w-4" />
                        <span>{item.title}</span>
                      </Link>
                    </SidebarMenuButton>
                  );

                  return (
                    <SidebarMenuItem key={item.title}>
                      {isCollapsed ? (
                        <TooltipProvider>
                          <Tooltip>
                            <TooltipTrigger asChild>
                              {menuButton}
                            </TooltipTrigger>
                            <TooltipContent side="right" className="text-sm">
                              {item.title}
                            </TooltipContent>
                          </Tooltip>
                        </TooltipProvider>
                      ) : (
                        menuButton
                      )}
                    </SidebarMenuItem>
                  );
                })}
              </SidebarMenu>
            </SidebarGroupContent>
          </SidebarGroup>
        ) : (
          !isCollapsed && (
            <SidebarGroup>
              <SidebarGroupLabel>Current Study Work</SidebarGroupLabel>
              <SidebarGroupContent>
                <div className="text-muted-foreground px-3 py-2 text-xs">
                  Select a study to access participants, trials, experiments,
                  and analytics.
                </div>
              </SidebarGroupContent>
            </SidebarGroup>
          )
        )}

        {/* Admin Section */}
        {isAdmin && (
          <SidebarGroup>
            <SidebarGroupLabel>Administration</SidebarGroupLabel>
            <SidebarGroupContent>
              <SidebarMenu>
                {adminItems.map((item) => {
                  const isActive = pathname.startsWith(item.url);

                  const menuButton = (
                    <SidebarMenuButton asChild isActive={isActive}>
                      <Link href={item.url}>
                        <item.icon className="h-4 w-4" />
                        <span>{item.title}</span>
                      </Link>
                    </SidebarMenuButton>
                  );

                  return (
                    <SidebarMenuItem key={item.title}>
                      {isCollapsed ? (
                        <TooltipProvider>
                          <Tooltip>
                            <TooltipTrigger asChild>
                              {menuButton}
                            </TooltipTrigger>
                            <TooltipContent side="right" className="text-sm">
                              {item.title}
                            </TooltipContent>
                          </Tooltip>
                        </TooltipProvider>
                      ) : (
                        menuButton
                      )}
                    </SidebarMenuItem>
                  );
                })}
              </SidebarMenu>
            </SidebarGroupContent>
          </SidebarGroup>
        )}
      </SidebarContent>

      {/* Debug info moved to footer tooltip button */}

      <SidebarFooter>
        <SidebarMenu>
          {showDebug && (
            <SidebarMenuItem>
              {isCollapsed ? (
                <TooltipProvider>
                  <Tooltip>
                    <TooltipTrigger asChild>
                      <button
                        type="button"
                        className="text-muted-foreground hover:bg-sidebar-accent hover:text-sidebar-accent-foreground flex h-8 w-8 items-center justify-center rounded-md border border-transparent text-xs"
                        aria-label="Debug info"
                      >
                        <BarChart3 className="h-4 w-4" />
                      </button>
                    </TooltipTrigger>
                    <TooltipContent
                      side="right"
                      className="space-y-1 p-2 text-[10px]"
                    >
                      <div>Session: {session?.user?.email ?? "No session"}</div>
                      <div>Role: {userRole ?? "No role"}</div>
                      <div>Studies: {userStudies.length}</div>
                      <div>Selected: {selectedStudy?.name ?? "None"}</div>
                      <div>Auth: {session ? "✓" : "✗"}</div>
                      {debugData && (
                        <>
                          <div>DB User: {debugData.user?.email ?? "None"}</div>
                          <div>
                            System Roles:{" "}
                            {debugData.systemRoles.join(", ") || "None"}
                          </div>
                          <div>
                            Memberships: {debugData.studyMemberships.length}
                          </div>
                          <div>All Studies: {debugData.allStudies.length}</div>
                          <div>
                            Session ID: {debugData.session.userId.slice(0, 8)}
                            ...
                          </div>
                        </>
                      )}
                    </TooltipContent>
                  </Tooltip>
                </TooltipProvider>
              ) : (
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <SidebarMenuButton className="w-full justify-start">
                      <BarChart3 className="h-4 w-4" />
                      <span className="truncate">Debug</span>
                      <ChevronDown className="ml-auto h-4 w-4 flex-shrink-0" />
                    </SidebarMenuButton>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent
                    className="w-[--radix-popper-anchor-width] max-w-72"
                    align="start"
                  >
                    <DropdownMenuLabel className="text-xs font-medium">
                      Debug Info
                    </DropdownMenuLabel>
                    <DropdownMenuSeparator />
                    <div className="space-y-1 px-2 py-1 text-[11px] leading-tight">
                      <div>Session: {session?.user?.email ?? "No session"}</div>
                      <div>Role: {userRole ?? "No role"}</div>
                      <div>Studies: {userStudies.length}</div>
                      <div>Selected: {selectedStudy?.name ?? "None"}</div>
                      <div>Auth: {session ? "✓" : "✗"}</div>
                      {debugData && (
                        <>
                          <div>DB User: {debugData.user?.email ?? "None"}</div>
                          <div>
                            System Roles:{" "}
                            {debugData.systemRoles.join(", ") || "None"}
                          </div>
                          <div>
                            Memberships: {debugData.studyMemberships.length}
                          </div>
                          <div>All Studies: {debugData.allStudies.length}</div>
                          <div>
                            Session ID: {debugData.session.userId.slice(0, 8)}
                            ...
                          </div>
                        </>
                      )}
                    </div>
                  </DropdownMenuContent>
                </DropdownMenu>
              )}
            </SidebarMenuItem>
          )}
          <SidebarMenuItem>
            {isCollapsed ? (
              <TooltipProvider>
                <Tooltip>
                  <TooltipTrigger asChild>
                    <DropdownMenu>
                      <DropdownMenuTrigger asChild>
                        <SidebarMenuButton
                          size="lg"
                          className="data-[state=open]:bg-sidebar-accent data-[state=open]:text-sidebar-accent-foreground group-data-[collapsible=icon]:h-8 group-data-[collapsible=icon]:w-8 group-data-[collapsible=icon]:justify-center"
                        >
                          <Avatar className="h-6 w-6 border-2 border-slate-300 group-data-[collapsible=icon]:h-6 group-data-[collapsible=icon]:w-6">
                            <AvatarImage
                              src={session?.user?.image ?? undefined}
                              alt={session?.user?.name ?? "User"}
                            />
                            <AvatarFallback className="bg-slate-600 text-xs text-white">
                              {(
                                session?.user?.name ??
                                session?.user?.email ??
                                "U"
                              )
                                .charAt(0)
                                .toUpperCase()}
                            </AvatarFallback>
                          </Avatar>
                          <div className="grid flex-1 text-left text-sm leading-tight group-data-[collapsible=icon]:hidden">
                            <span className="truncate font-semibold">
                              {session?.user?.name ?? "User"}
                            </span>
                            <span className="truncate text-xs">
                              {session?.user?.email ?? ""}
                            </span>
                          </div>
                          <MoreHorizontal className="ml-auto size-4 group-data-[collapsible=icon]:hidden" />
                        </SidebarMenuButton>
                      </DropdownMenuTrigger>
                      <DropdownMenuContent
                        className="w-[--radix-popper-anchor-width] min-w-56 rounded-lg"
                        side="bottom"
                        align="end"
                        sideOffset={4}
                      >
                        <DropdownMenuLabel className="p-0 font-normal">
                          <div className="flex items-center gap-2 px-1 py-1.5 text-left text-sm">
                            <Avatar className="h-8 w-8 border-2 border-slate-300">
                              <AvatarImage
                                src={session?.user?.image ?? undefined}
                                alt={session?.user?.name ?? "User"}
                              />
                              <AvatarFallback className="bg-slate-600 text-xs text-white">
                                {(
                                  session?.user?.name ??
                                  session?.user?.email ??
                                  "U"
                                )
                                  .charAt(0)
                                  .toUpperCase()}
                              </AvatarFallback>
                            </Avatar>
                            <div className="grid flex-1 text-left text-sm leading-tight">
                              <span className="truncate font-semibold">
                                {session?.user?.name ?? "User"}
                              </span>
                              <span className="truncate text-xs">
                                {session?.user?.email ?? ""}
                              </span>
                            </div>
                          </div>
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
                  </TooltipTrigger>
                  <TooltipContent side="right" className="text-sm">
                    {session?.user?.name ?? "User Menu"}
                  </TooltipContent>
                </Tooltip>
              </TooltipProvider>
            ) : (
              <DropdownMenu>
                <DropdownMenuTrigger asChild>
                  <SidebarMenuButton
                    size="lg"
                    className="data-[state=open]:bg-sidebar-accent data-[state=open]:text-sidebar-accent-foreground group-data-[collapsible=icon]:h-8 group-data-[collapsible=icon]:w-8 group-data-[collapsible=icon]:justify-center"
                  >
                    <Avatar className="h-6 w-6 border-2 border-slate-300 group-data-[collapsible=icon]:h-6 group-data-[collapsible=icon]:w-6">
                      <AvatarImage
                        src={session?.user?.image ?? undefined}
                        alt={session?.user?.name ?? "User"}
                      />
                      <AvatarFallback className="bg-slate-600 text-xs text-white">
                        {(session?.user?.name ?? session?.user?.email ?? "U")
                          .charAt(0)
                          .toUpperCase()}
                      </AvatarFallback>
                    </Avatar>
                    <div className="grid flex-1 text-left text-sm leading-tight group-data-[collapsible=icon]:hidden">
                      <span className="truncate font-semibold">
                        {session?.user?.name ?? "User"}
                      </span>
                      <span className="truncate text-xs">
                        {session?.user?.email ?? ""}
                      </span>
                    </div>
                    <MoreHorizontal className="ml-auto size-4 group-data-[collapsible=icon]:hidden" />
                  </SidebarMenuButton>
                </DropdownMenuTrigger>
                <DropdownMenuContent
                  className="w-[--radix-popper-anchor-width] min-w-56 rounded-lg"
                  side="bottom"
                  align="end"
                  sideOffset={4}
                >
                  <DropdownMenuLabel className="p-0 font-normal">
                    <div className="flex items-center gap-2 px-1 py-1.5 text-left text-sm">
                      <Avatar className="h-8 w-8 border-2 border-slate-300">
                        <AvatarImage
                          src={session?.user?.image ?? undefined}
                          alt={session?.user?.name ?? "User"}
                        />
                        <AvatarFallback className="bg-slate-600 text-xs text-white">
                          {(session?.user?.name ?? session?.user?.email ?? "U")
                            .charAt(0)
                            .toUpperCase()}
                        </AvatarFallback>
                      </Avatar>
                      <div className="grid flex-1 text-left text-sm leading-tight">
                        <span className="truncate font-semibold">
                          {session?.user?.name ?? "User"}
                        </span>
                        <span className="truncate text-xs">
                          {session?.user?.email ?? ""}
                        </span>
                      </div>
                    </div>
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
            )}
          </SidebarMenuItem>
        </SidebarMenu>
      </SidebarFooter>

      <SidebarRail />
    </Sidebar>
  );
}
