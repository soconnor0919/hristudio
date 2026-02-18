"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  Copy,
  Edit,
  Eye,
  FlaskConical,
  LayoutTemplate,
  MoreHorizontal,
  Play,
  TestTube,
  Trash2,
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

export type Experiment = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "testing" | "ready" | "deprecated";
  createdAt: Date;
  updatedAt: Date;
  studyId: string;
  study: {
    id: string;
    name: string;
  };
  createdBy: string;
  owner: {
    name: string | null;
    email: string;
  };
  _count?: {
    steps: number;
    trials: number;
  };
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  canEdit?: boolean;
  canDelete?: boolean;
};

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    description: "Experiment in preparation",
  },
  testing: {
    label: "Testing",
    className: "bg-yellow-100 text-yellow-800 hover:bg-yellow-200",
    description: "Experiment being tested",
  },
  ready: {
    label: "Ready",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    description: "Experiment ready for trials",
  },
  deprecated: {
    label: "Deprecated",
    className: "bg-slate-100 text-slate-800 hover:bg-slate-200",
    description: "Experiment deprecated",
  },
};

function ExperimentActionsCell({ experiment }: { experiment: Experiment }) {
  const utils = api.useUtils();
  const deleteMutation = api.experiments.delete.useMutation({
    onSuccess: () => {
      toast.success("Experiment deleted successfully");
      utils.experiments.list.invalidate();
    },
    onError: (error) => {
      toast.error(`Failed to delete experiment: ${error.message}`);
    },
  });

  const handleDelete = () => {
    if (
      window.confirm(`Are you sure you want to delete "${experiment.name}"?`)
    ) {
      deleteMutation.mutate({ id: experiment.id });
    }
  };

  return (
    <div className="flex items-center gap-2">
      <Button
        variant="ghost"
        size="icon"
        asChild
        className="h-8 w-8 text-muted-foreground hover:text-primary"
        title="Open Designer"
      >
        <Link href={`/studies/${experiment.studyId}/experiments/${experiment.id}/designer`}>
          <LayoutTemplate className="h-4 w-4" />
          <span className="sr-only">Design</span>
        </Link>
      </Button>

      {experiment.canDelete && (
        <Button
          variant="ghost"
          size="icon"
          onClick={handleDelete}
          className="h-8 w-8 text-muted-foreground hover:text-destructive"
          title="Delete Experiment"
        >
          <Trash2 className="h-4 w-4" />
          <span className="sr-only">Delete</span>
        </Button>
      )}
    </div>
  );
}

export const experimentsColumns: ColumnDef<Experiment>[] = [
  {
    id: "select",
    header: ({ table }) => (
      <Checkbox
        checked={
          table.getIsAllPageRowsSelected() ||
          (table.getIsSomePageRowsSelected() && "indeterminate")
        }
        onCheckedChange={(value: boolean) =>
          table.toggleAllPageRowsSelected(!!value)
        }
        aria-label="Select all"
      />
    ),
    cell: ({ row }) => (
      <Checkbox
        checked={row.getIsSelected()}
        onCheckedChange={(value: boolean) => row.toggleSelected(!!value)}
        aria-label="Select row"
      />
    ),
    enableSorting: false,
    enableHiding: false,
  },
  {
    accessorKey: "name",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Experiment Name" />
    ),
    cell: ({ row }) => {
      const experiment = row.original;
      return (
        <div className="max-w-[200px] min-w-0 space-y-1">
          <Link
            href={`/studies/${experiment.studyId}/experiments/${experiment.id}`}
            className="block truncate font-medium hover:underline"
            title={experiment.name}
          >
            {experiment.name}
          </Link>
          {experiment.description && (
            <p
              className="text-muted-foreground line-clamp-1 truncate text-sm"
              title={experiment.description}
            >
              {experiment.description}
            </p>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "study",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Study" />
    ),
    cell: ({ row }) => {
      const study = row.original.study;
      if (!study?.id || !study?.name)
        return <span className="text-muted-foreground">No study</span>;
      return (
        <Link
          href={`/studies/${study.id}`}
          className="text-primary hover:underline"
        >
          {study.name}
        </Link>
      );
    },
    enableSorting: false,
  },
  {
    accessorKey: "status",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Status" />
    ),
    cell: ({ row }) => {
      const status = row.getValue("status");
      const config = statusConfig[status as keyof typeof statusConfig];

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
      return value.includes(row.getValue(id));
    },
  },
  {
    id: "stats",
    header: "Statistics",
    cell: ({ row }) => {
      const experiment = row.original;
      const counts = experiment._count;

      return (
        <div className="flex space-x-4 text-sm">
          <div className="flex items-center space-x-1" title="Steps">
            <FlaskConical className="text-muted-foreground h-3 w-3" />
            <span>{counts?.steps ?? 0}</span>
          </div>
          <div className="flex items-center space-x-1" title="Trials">
            <TestTube className="text-muted-foreground h-3 w-3" />
            <span>{counts?.trials ?? 0}</span>
          </div>
        </div>
      );
    },
    enableSorting: false,
    enableHiding: false,
  },
  {
    accessorKey: "owner",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Owner" />
    ),
    cell: ({ row }) => {
      const owner = row.original.owner;
      if (!owner) {
        return <span className="text-muted-foreground">No owner</span>;
      }
      return (
        <div className="max-w-[140px] space-y-1">
          <div
            className="truncate text-sm font-medium"
            title={owner.name ?? "Unknown"}
          >
            {owner.name ?? "Unknown"}
          </div>
          <div
            className="text-muted-foreground truncate text-xs"
            title={owner.email ?? ""}
          >
            {owner.email ?? ""}
          </div>
        </div>
      );
    },
    enableSorting: false,
  },

  {
    accessorKey: "updatedAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Updated" />
    ),
    cell: ({ row }) => {
      const date = row.getValue("updatedAt");
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date as Date, { addSuffix: true })}
        </div>
      );
    },
  },
  {
    id: "actions",
    header: "Actions",
    cell: ({ row }) => <ExperimentActionsCell experiment={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
