"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  Copy,
  ExternalLink,
  MoreHorizontal,
  Puzzle,
  Settings,
  Trash2,
  User,
} from "lucide-react";

import { toast } from "sonner";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Checkbox } from "~/components/ui/checkbox";
import { DataTableColumnHeader } from "~/components/ui/data-table-column-header";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";

export type Plugin = {
  plugin: {
    id: string;
    robotId: string | null;
    name: string;
    version: string;
    description: string | null;
    author: string | null;
    repositoryUrl: string | null;
    trustLevel: "official" | "verified" | "community" | null;
    status: "active" | "deprecated" | "disabled";
    createdAt: Date;
    updatedAt: Date;
  };
  installation: {
    id: string;
    configuration: Record<string, unknown>;
    installedAt: Date;
    installedBy: string;
  };
};

const trustLevelConfig = {
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

const statusConfig = {
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
  disabled: {
    label: "Disabled",
    className: "bg-red-100 text-red-800 hover:bg-red-200",
    description: "Plugin is disabled",
  },
};

function PluginActionsCell({ plugin }: { plugin: Plugin }) {
  const handleUninstall = async () => {
    if (
      window.confirm(
        `Are you sure you want to uninstall "${plugin.plugin.name}"?`,
      )
    ) {
      try {
        // TODO: Implement uninstall mutation
        toast.success("Plugin uninstalled successfully");
      } catch {
        toast.error("Failed to uninstall plugin");
      }
    }
  };

  const handleCopyId = () => {
    void navigator.clipboard.writeText(plugin.plugin.id);
    toast.success("Plugin ID copied to clipboard");
  };

  return (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="ghost" className="h-8 w-8 p-0">
          <span className="sr-only">Open menu</span>
          <MoreHorizontal className="h-4 w-4" />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end">
        <DropdownMenuLabel>Actions</DropdownMenuLabel>
        <DropdownMenuSeparator />

        <DropdownMenuItem>
          <Settings className="mr-2 h-4 w-4" />
          Configure
        </DropdownMenuItem>

        {plugin.plugin.repositoryUrl && (
          <DropdownMenuItem asChild>
            <a
              href={plugin.plugin.repositoryUrl}
              target="_blank"
              rel="noopener noreferrer"
            >
              <ExternalLink className="mr-2 h-4 w-4" />
              View Repository
            </a>
          </DropdownMenuItem>
        )}

        <DropdownMenuSeparator />

        <DropdownMenuItem onClick={handleCopyId}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Plugin ID
        </DropdownMenuItem>

        <DropdownMenuSeparator />
        <DropdownMenuItem
          onClick={handleUninstall}
          className="text-red-600 focus:text-red-600"
        >
          <Trash2 className="mr-2 h-4 w-4" />
          Uninstall
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

export const pluginsColumns: ColumnDef<Plugin>[] = [
  {
    id: "select",
    header: ({ table }) => (
      <Checkbox
        checked={
          table.getIsAllPageRowsSelected() ||
          (table.getIsSomePageRowsSelected() && "indeterminate")
        }
        onCheckedChange={(value) => table.toggleAllPageRowsSelected(!!value)}
        aria-label="Select all"
      />
    ),
    cell: ({ row }) => (
      <Checkbox
        checked={row.getIsSelected()}
        onCheckedChange={(value) => row.toggleSelected(!!value)}
        aria-label="Select row"
      />
    ),
    enableSorting: false,
    enableHiding: false,
  },
  {
    id: "name",
    accessorFn: (row) => row.plugin.name,
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Plugin Name" />
    ),
    cell: ({ row }) => {
      const plugin = row.original;
      return (
        <div className="max-w-[200px] min-w-0 space-y-1">
          <div className="flex items-center space-x-2">
            <Puzzle className="text-muted-foreground h-4 w-4 flex-shrink-0" />
            <span className="truncate font-medium" title={plugin.plugin.name}>
              {plugin.plugin.name}
            </span>
          </div>
          {plugin.plugin.description && (
            <p
              className="text-muted-foreground line-clamp-1 truncate text-sm"
              title={plugin.plugin.description}
            >
              {plugin.plugin.description}
            </p>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "plugin.version",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Version" />
    ),
    cell: ({ row }) => {
      const version = row.original.plugin.version;
      return (
        <Badge variant="outline" className="font-mono text-xs">
          v{version}
        </Badge>
      );
    },
  },
  {
    accessorKey: "plugin.author",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Author" />
    ),
    cell: ({ row }) => {
      const author = row.original.plugin.author;
      return (
        <div className="flex items-center space-x-1 text-sm">
          <User className="text-muted-foreground h-3 w-3" />
          <span className="max-w-[120px] truncate" title={author ?? undefined}>
            {author ?? "Unknown"}
          </span>
        </div>
      );
    },
  },
  {
    accessorKey: "plugin.trustLevel",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Trust Level" />
    ),
    cell: ({ row }) => {
      const trustLevel = row.original.plugin.trustLevel;
      if (!trustLevel) return "-";

      const config = trustLevelConfig[trustLevel];

      return (
        <Badge
          variant="secondary"
          className={config.className}
          title={config.description}
        >
          {config.label}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      const trustLevel = row.original.plugin.trustLevel;
      return trustLevel ? value.includes(trustLevel) : false;
    },
  },
  {
    accessorKey: "plugin.status",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Status" />
    ),
    cell: ({ row }) => {
      const status = row.original.plugin.status;
      const config = statusConfig[status];

      return (
        <Badge
          variant="secondary"
          className={config.className}
          title={config.description}
        >
          {config.label}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      return value.includes(row.original.plugin.status);
    },
  },
  {
    accessorKey: "installation.installedAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Installed" />
    ),
    cell: ({ row }) => {
      const date = row.original.installation.installedAt;
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date, { addSuffix: true })}
        </div>
      );
    },
  },
  {
    accessorKey: "plugin.updatedAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Updated" />
    ),
    cell: ({ row }) => {
      const date = row.original.plugin.updatedAt;
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date, { addSuffix: true })}
        </div>
      );
    },
  },
  {
    id: "actions",
    header: "Actions",
    cell: ({ row }) => <PluginActionsCell plugin={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
