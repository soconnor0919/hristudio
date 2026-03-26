import type { LucideIcon } from "lucide-react";

export const trustLevelConfig = {
  official: {
    label: "Official",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    description: "Official HRIStudio plugin",
  },
  verified: {
    label: "Verified",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    description: "Verified by the community",
  },
  community: {
    label: "Community",
    className: "bg-yellow-100 text-yellow-800 hover:bg-yellow-200",
    description: "Community contributed",
  },
};

export const statusConfig = {
  active: {
    label: "Active",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    description: "Plugin is active and working",
  },
  deprecated: {
    label: "Deprecated",
    className: "bg-orange-100 text-orange-800 hover:bg-orange-200",
    description: "Plugin is deprecated",
  },
  inactive: {
    label: "Inactive",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    description: "Plugin is not active",
  },
};

export const formStatusColors = {
  pending: "bg-yellow-100 text-yellow-700",
  completed: "bg-green-100 text-green-700",
  rejected: "bg-red-100 text-red-700",
};
