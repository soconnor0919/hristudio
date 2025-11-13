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
import { api } from "~/trpc/react";

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

function RepositoryActionsCell({ repository }: { repository: Repository }) {
  const utils = api.useUtils();

  const syncMutation = api.admin.repositories.sync.useMutation({
    onSuccess: () => {
      toast.success("Repository sync started");
      void utils.admin.repositories.list.invalidate();
    },
    onError: (error: TRPCError) => {
      toast.error(error.message ?? "Failed to sync repository");
    },
  });

  const deleteMutation = api.admin.repositories.delete.useMutation({
    onSuccess: () => {
      toast.success("Repository deleted successfully");
      void utils.admin.repositories.list.invalidate();
    },
    onError: (error: TRPCError) => {
      toast.error(error.message ?? "Failed to delete repository");
    },
  });

  const handleSync = async () => {
    syncMutation.mutate({ id: repository.id });
  };

  const handleDelete = async () => {
    if (
      window.confirm(`Are you sure you want to delete "${repository.name}"?`)
    ) {
      deleteMutation.mutate({ id: repository.id });
    }
  };

  const handleCopyId = () => {
    void navigator.clipboard.writeText(repository.id);
    toast.success("Repository ID copied to clipboard");
  };

  const handleCopyUrl = () => {
    void navigator.clipboard.writeText(repository.url);
    toast.success("Repository URL copied to clipboard");
  };

  const canDelete = !repository.isOfficial;

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

        <DropdownMenuItem
          onClick={handleSync}
          disabled={syncMutation.isPending}
        >
          <RefreshCw
            className={`mr-2 h-4 w-4 ${syncMutation.isPending ? "animate-spin" : ""}`}
          />
          Sync Repository
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/admin/repositories/${repository.id}/edit`}>
            <Settings className="mr-2 h-4 w-4" />
            Edit Repository
          </Link>
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <a href={repository.url} target="_blank" rel="noopener noreferrer">
            <ExternalLink className="mr-2 h-4 w-4" />
            View Repository
          </a>
        </DropdownMenuItem>

        <DropdownMenuSeparator />

        <DropdownMenuItem onClick={handleCopyId}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Repository ID
        </DropdownMenuItem>

        <DropdownMenuItem onClick={handleCopyUrl}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Repository URL
        </DropdownMenuItem>

        {canDelete && (
          <>
            <DropdownMenuSeparator />
            <DropdownMenuItem
              onClick={handleDelete}
              disabled={deleteMutation.isPending}
              className="text-red-600 focus:text-red-600"
            >
              <Trash2 className="mr-2 h-4 w-4" />
              Delete Repository
            </DropdownMenuItem>
          </>
        )}
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

export const repositoriesColumns: ColumnDef<Repository>[] = [
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
    accessorKey: "name",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Repository Name" />
    ),
    cell: ({ row }) => {
      const repository = row.original;
      return (
        <div className="max-w-[200px] min-w-0 space-y-1">
          <div className="flex items-center space-x-2">
            <Database className="text-muted-foreground h-4 w-4 flex-shrink-0" />
            <Link
              href={`/admin/repositories/${repository.id}`}
              className="truncate font-medium hover:underline"
              title={repository.name}
            >
              {repository.name}
            </Link>
            {repository.isOfficial && (
              <Badge variant="outline" className="text-xs">
                Official
              </Badge>
            )}
          </div>
          {repository.description && (
            <p
              className="text-muted-foreground line-clamp-1 truncate text-sm"
              title={repository.description}
            >
              {repository.description}
            </p>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "url",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Repository URL" />
    ),
    cell: ({ row }) => {
      const url = row.original.url;
      return (
        <a
          href={url}
          target="_blank"
          rel="noopener noreferrer"
          className="max-w-[300px] truncate text-sm text-blue-600 hover:underline"
          title={url}
        >
          {url}
        </a>
      );
    },
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
          variant="secondary"
          className={
            isEnabled
              ? "bg-green-100 text-green-800"
              : "bg-red-100 text-red-800"
          }
        >
          {isEnabled ? (
            <CheckCircle className="mr-1 h-3 w-3" />
          ) : (
            <XCircle className="mr-1 h-3 w-3" />
          )}
          {isEnabled ? "Enabled" : "Disabled"}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      const isEnabled = row.original.isEnabled;
      return value.includes(isEnabled ? "enabled" : "disabled");
    },
  },
  {
    accessorKey: "syncStatus",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Sync Status" />
    ),
    cell: ({ row }) => {
      const syncStatus = row.original.syncStatus;
      const lastSyncAt = row.original.lastSyncAt;
      const syncError = row.original.syncError;

      if (!syncStatus) return "-";

      const config =
        syncStatusConfig[syncStatus as keyof typeof syncStatusConfig];
      if (!config) return syncStatus;

      const SyncIcon = config.icon;

      return (
        <div className="space-y-1">
          <Badge
            variant="secondary"
            className={config.className}
            title={config.description}
          >
            <SyncIcon
              className={`mr-1 h-3 w-3 ${syncStatus === "syncing" ? "animate-spin" : ""}`}
            />
            {config.label}
          </Badge>
          {lastSyncAt && syncStatus === "completed" && (
            <div className="text-muted-foreground text-xs">
              {formatDistanceToNow(lastSyncAt, { addSuffix: true })}
            </div>
          )}
          {syncError && syncStatus === "failed" && (
            <div
              className="max-w-[150px] truncate text-xs text-red-600"
              title={syncError}
            >
              {syncError}
            </div>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "createdAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Created" />
    ),
    cell: ({ row }) => {
      const date = row.original.createdAt;
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date, { addSuffix: true })}
        </div>
      );
    },
  },
  {
    accessorKey: "updatedAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Updated" />
    ),
    cell: ({ row }) => {
      const date = row.original.updatedAt;
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
    cell: ({ row }) => <RepositoryActionsCell repository={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
