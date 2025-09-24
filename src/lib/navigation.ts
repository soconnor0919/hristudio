import {
  Activity,
  BarChart3,
  Calendar,
  FlaskConical,
  Home,
  Target,
  UserCog,
} from "lucide-react";

export interface NavigationItem {
  label: string;
  href: string;
  icon: React.ComponentType<{ className?: string }>;
  roles: string[];
  requiresStudy: boolean;
  description?: string;
}

// Core Navigation - Always accessible regardless of study selection
export const coreNavigationItems: NavigationItem[] = [
  {
    label: "Dashboard",
    href: "/dashboard",
    icon: Home,
    roles: ["administrator", "researcher", "wizard", "observer"],
    requiresStudy: false,
    description: "Overview of your research activities",
  },
  {
    label: "Studies",
    href: "/studies",
    icon: FlaskConical,
    roles: ["administrator", "researcher", "wizard", "observer"],
    requiresStudy: false,
    description: "Manage your research studies",
  },
];

// Research Workflow - Requires active study selection
export const researchWorkflowItems: NavigationItem[] = [
  {
    label: "Experiments",
    href: "/experiments",
    icon: Target,
    roles: ["administrator", "researcher"],
    requiresStudy: true,
    description: "Design experimental protocols",
  },
];

// Trial Execution - Active wizard controls
export const trialExecutionItems: NavigationItem[] = [
  {
    label: "Active Trials",
    href: "/trials?status=in_progress",
    icon: Activity,
    roles: ["administrator", "researcher", "wizard"],
    requiresStudy: true,
    description: "Currently running trials",
  },
  {
    label: "Schedule Trial",
    href: "/studies/{studyId}/trials/new",
    icon: Calendar,
    roles: ["administrator", "researcher"],
    requiresStudy: true,
    description: "Create new trial session",
  },
];

// Analysis & Data - Study-dependent analysis tools
export const analysisItems: NavigationItem[] = [
  {
    label: "Data Analysis",
    href: "/trials",
    icon: BarChart3,
    roles: ["administrator", "researcher"],
    requiresStudy: true,
    description: "Analyze trial data and results",
  },
];

// User Management - Personal and admin functions
export const userManagementItems: NavigationItem[] = [];

// Administration - System-wide admin functions
export const administrationItems: NavigationItem[] = [
  {
    label: "Administration",
    href: "/admin",
    icon: UserCog,
    roles: ["administrator"],
    requiresStudy: false,
    description: "System administration",
  },
];

// Sidebar sections configuration
export interface SidebarSection {
  id: string;
  label: string;
  items: NavigationItem[];
  alwaysVisible: boolean;
  order: number;
}

export const sidebarSections: SidebarSection[] = [
  {
    id: "core",
    label: "Navigation",
    items: coreNavigationItems,
    alwaysVisible: true,
    order: 1,
  },
  {
    id: "research",
    label: "Research",
    items: researchWorkflowItems,
    alwaysVisible: false,
    order: 2,
  },
  {
    id: "execution",
    label: "Trial Control",
    items: trialExecutionItems,
    alwaysVisible: false,
    order: 3,
  },
  {
    id: "analysis",
    label: "Analysis",
    items: analysisItems,
    alwaysVisible: false,
    order: 4,
  },
  {
    id: "user",
    label: "Account",
    items: userManagementItems,
    alwaysVisible: true,
    order: 5,
  },
  {
    id: "admin",
    label: "Administration",
    items: administrationItems,
    alwaysVisible: true,
    order: 6,
  },
];

// Helper functions
export function getAccessibleSections(
  userRole: string,
  hasActiveStudy: boolean,
): SidebarSection[] {
  return sidebarSections
    .filter((section) => {
      // Always show sections marked as alwaysVisible
      if (section.alwaysVisible) {
        return section.items.some((item) => item.roles.includes(userRole));
      }

      // For study-dependent sections, check both role and study selection
      return section.items.some((item) => {
        const hasRole = item.roles.includes(userRole);
        const studyRequirement = item.requiresStudy ? hasActiveStudy : true;
        return hasRole && studyRequirement;
      });
    })
    .sort((a, b) => a.order - b.order);
}

export function getAccessibleItems(
  items: NavigationItem[],
  userRole: string,
  hasActiveStudy: boolean,
): NavigationItem[] {
  return items.filter((item) => {
    const hasRole = item.roles.includes(userRole);
    const studyRequirement = item.requiresStudy ? hasActiveStudy : true;
    return hasRole && studyRequirement;
  });
}

export function getHiddenItemsCount(
  userRole: string,
  hasActiveStudy: boolean,
): number {
  const allStudyDependentItems = sidebarSections
    .flatMap((section) => section.items)
    .filter((item) => item.requiresStudy && item.roles.includes(userRole));

  const accessibleStudyDependentItems = getAccessibleItems(
    allStudyDependentItems,
    userRole,
    hasActiveStudy,
  );

  return allStudyDependentItems.length - accessibleStudyDependentItems.length;
}

// Legacy exports for backward compatibility
export const navigationItems = coreNavigationItems.concat(
  researchWorkflowItems,
);
export const wizardItems = trialExecutionItems;
export const profileItems = userManagementItems;
export const adminItems = administrationItems;
