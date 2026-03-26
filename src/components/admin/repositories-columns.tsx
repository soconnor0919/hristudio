"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  Copy,
  ExternalLink,
  MoreHorizontal,
  Database,
  RefreshCw,
  Settings,
  Trash2,
  Shield,
  CheckCircle,
  XCircle,
  Clock,
  AlertTriangle,
} from "lucide-react";
import Link from "next/link";

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

// Define error type for mutations
interface TRPCError {
  message: string;
}

export type Repository = {
  id: string;
  name: string;
  url: string;
  description: string | null;
  trustLevel: "official" | "verified" | "community";
  isEnabled: boolean;
  isOfficial: boolean;
  lastSyncAt: Date | null;
  syncStatus: string | null;
  syncError: string | null;
  createdAt: Date;
  updatedAt: Date;
};

const trustLevelConfig = {
  official: {
    label: "Official",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    icon: Shield,
    description: "Official HRIStudio repository",
  },
  verified: {
    label: "Verified",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    icon: Shield,
    description: "Verified by the community",
  },
  community: {
    label: "Community",
    className: "bg-yellow-100 text-yellow-800 hover:bg-yellow-200",
    icon: Shield,
    description: "Community repository",
  },
};

const syncStatusConfig = {
  pending: {
    label: "Pending",
    className: "bg-gray-100 text-gray-800",
    icon: Clock,
    description: "Waiting to sync",
  },
  syncing: {
    label: "Syncing",
    className: "bg-blue-100 text-blue-800",
    icon: RefreshCw,
    description: "Currently syncing",
  },
  completed: {
    label: "Success",
    className: "bg-green-100 text-green-800",
    icon: CheckCircle,
    description: "Last sync completed successfully",
  },
  failed: {
    label: "Failed",
    className: "bg-red-100 text-red-800",
    icon: AlertTriangle,
    description: "Last sync failed",
  },
};

function RepositoryUrlCell({ url }: { url: string }) {
  const handleCopy = () => {
    void navigator.clipboard.writeText(url);
    toast.success("URL copied to clipboard");
  };

  return (
    <div className="flex items-center gap-2">
      <Button variant="ghost" size="icon" onClick={handleCopy}>
        <Copy className="h-4 w-4" />
      </Button>
      {url && (
        <Link
          href={url}
          target="_blank"
          rel="noopener noreferrer"
          className="flex items-center gap-1 text-sm text-muted-foreground hover:text-foreground"
        >
          <ExternalLink className="mr-1 h-3 w-3" />
          Visit
        </Link>
      )}
    </div>
  );
}

export const repositoriesColumns: ColumnDef<Repository>[] = [
  {
    id: "select",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="#" />
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
    accessorKey: "name",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Repository" />
    ),
    cell: ({ row }) => {
      return (
        <div className="flex flex-col">
          <span className="font-medium">{row.original.name}</span>
          {row.original.description && (
            <span className="text-muted-foreground text-xs">
              {row.original.description}
            </span>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "url",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="URL" />
    ),
    cell: ({ row }) => (
      <RepositoryUrlCell url={row.original.url} />
    ),
  },
  {
    accessorKey: "trustLevel",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Trust Level" />
    ),
    cell: ({ row }) => {
      const trustLevel = row.original.trustLevel;
      const config = trustLevelConfig[trustLevel];
      const TrustIcon = config.icon;

      return (
        <Badge
          variant="secondary"
          className={config.className}
          title={config.description}
        >
          <TrustIcon className="mr-1 h-3 w-3" />
          {config.label}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      return value.includes(row.original.trustLevel);
    },
  },
  {
    accessorKey: "isEnabled",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Status" />
    ),
    cell: ({ row }) => {
      const isEnabled = row.original.isEnabled;
      return (
        <Badge
          variant={isEnabled ? "default" : "secondary"}
          className={isEnabled ? "bg-green-100 text-green-800" : "bg-gray-100 text-gray-800"}
        >
          {isEnabled ? "Enabled" : "Disabled"}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      return value.includes(row.original.isEnabled ? "enabled" : "disabled");
    },
  },
  {
    accessorKey: "syncStatus",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Sync Status" />
    ),
    cell: ({ row }) => {
      const status = row.original.syncStatus || "pending";
      const config = syncStatusConfig[status as keyof typeof syncStatusConfig];
      const StatusIcon = config?.icon ?? Clock;

      return (
        <div className="flex items-center gap-2">
          {config && (
            <Badge variant="secondary" className={config.className}>
              <StatusIcon className="mr-1 h-3 w-3" />
              {config.label}
            </Badge>
          )}
          {row.original.syncError && (
            <Button
              variant="ghost"
              size="sm"
              className="h-auto whitespace-normal text-xs text-destructive"
              title={row.original.syncError}
            >
              <AlertTriangle className="mr-1 h-3 w-3" />
              Error
            </Button>
          )}
        </div>
      );
    },
    filterFn: (row, id, value: string[]) => {
      const status = row.original.syncStatus || "pending";
      return value.includes(status);
    },
  },
  {
    accessorKey: "lastSyncAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Last Sync" />
    ),
    cell: ({ row }) => {
      const lastSync = row.original.lastSyncAt;
      return (
        <span className="text-muted-foreground text-sm">
          {lastSync ? formatDistanceToNow(lastSync, { addSuffix: true }) : "Never"}
        </span>
      );
    },
    sortingFn: "datetime",
  },
  {
    id: "actions",
    cell: ({ row }) => {
      const repository = row.original;
      return (
        <DropdownMenu>
          <DropdownMenuTrigger asChild>
            <Button variant="ghost" className="h-8 w-8 p-0">
              <MoreHorizontal className="h-4 w-4" />
            </Button>
          </DropdownMenuTrigger>
          <DropdownMenuContent align="end">
            <DropdownMenuLabel>Actions</DropdownMenuLabel>
            <DropdownMenuItem
              onClick={() => navigator.clipboard.writeText(repository.id)}
            >
              <Copy className="mr-2 h-4 w-4" />
              Copy ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/admin/repositories/${repository.id}`}>
                <Settings className="mr-2 h-4 w-4" />
                Settings
              </Link>
            </DropdownMenuItem>
            <DropdownMenuItem
              asChild
              disabled={!repository.url}
            >
              <Link href={repository.url ?? "#"} target="_blank">
                <ExternalLink className="mr-2 h-4 w-4" />
                Visit Repository
              </Link>
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            {!repository.isOfficial && (
              <DropdownMenuItem className="text-destructive">
                <Trash2 className="mr-2 h-4 w-4" />
                Delete Repository
              </DropdownMenuItem>
            )}
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  },
];
