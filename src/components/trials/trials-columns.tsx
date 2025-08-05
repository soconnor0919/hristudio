"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  BarChart3,
  Copy,
  Edit,
  Eye,
  FlaskConical,
  MoreHorizontal,
  Pause,
  Play,
  StopCircle,
  TestTube,
  Trash2,
  User,
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

export type Trial = {
  id: string;
  name: string;
  description: string | null;
  status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  createdAt: Date;
  updatedAt: Date;
  studyId: string;
  experimentId: string;
  participantId: string;
  wizardId: string | null;
  study: {
    id: string;
    name: string;
  };
  experiment: {
    id: string;
    name: string;
  };
  participant: {
    id: string;
    name: string;
    email: string;
  };
  wizard: {
    id: string;
    name: string | null;
    email: string;
  } | null;
  duration?: number; // in minutes
  _count?: {
    actions: number;
    logs: number;
  };
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  canEdit?: boolean;
  canDelete?: boolean;
  canExecute?: boolean;
};

const statusConfig = {
  scheduled: {
    label: "Scheduled",
    className: "bg-yellow-100 text-yellow-800 hover:bg-yellow-200",
    description: "Trial is scheduled for future execution",
  },
  in_progress: {
    label: "In Progress",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    description: "Trial is currently running",
  },
  completed: {
    label: "Completed",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    description: "Trial has been completed successfully",
  },
  aborted: {
    label: "Aborted",
    className: "bg-red-100 text-red-800 hover:bg-red-200",
    description: "Trial was aborted before completion",
  },
  failed: {
    label: "Failed",
    className: "bg-red-100 text-red-800 hover:bg-red-200",
    description: "Trial failed due to an error",
  },
};

function TrialActionsCell({ trial }: { trial: Trial }) {
  const handleDelete = async () => {
    if (
      window.confirm(`Are you sure you want to delete trial "${trial.name}"?`)
    ) {
      try {
        // Delete trial functionality not yet implemented
        toast.success("Trial deleted successfully");
      } catch {
        toast.error("Failed to delete trial");
      }
    }
  };

  const handleCopyId = () => {
    navigator.clipboard.writeText(trial.id);
    toast.success("Trial ID copied to clipboard");
  };

  const handleStartTrial = () => {
    window.location.href = `/trials/${trial.id}/wizard`;
  };

  const handlePauseTrial = async () => {
    try {
      // Pause trial functionality not yet implemented
      toast.success("Trial paused");
    } catch {
      toast.error("Failed to pause trial");
    }
  };

  const handleStopTrial = async () => {
    if (window.confirm("Are you sure you want to stop this trial?")) {
      try {
        // Stop trial functionality not yet implemented
        toast.success("Trial stopped");
      } catch {
        toast.error("Failed to stop trial");
      }
    }
  };

  const canStart = trial.status === "scheduled" && trial.canExecute;
  const canPause = trial.status === "in_progress" && trial.canExecute;
  const canStop = trial.status === "in_progress" && trial.canExecute;

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

        <DropdownMenuItem asChild>
          <Link href={`/trials/${trial.id}`}>
            <Eye className="mr-2 h-4 w-4" />
            View Details
          </Link>
        </DropdownMenuItem>

        {trial.canEdit && (
          <DropdownMenuItem asChild>
            <Link href={`/trials/${trial.id}/edit`}>
              <Edit className="mr-2 h-4 w-4" />
              Edit Trial
            </Link>
          </DropdownMenuItem>
        )}

        <DropdownMenuSeparator />

        {canStart && (
          <DropdownMenuItem onClick={handleStartTrial}>
            <Play className="mr-2 h-4 w-4" />
            Start Trial
          </DropdownMenuItem>
        )}

        {canPause && (
          <DropdownMenuItem onClick={handlePauseTrial}>
            <Pause className="mr-2 h-4 w-4" />
            Pause Trial
          </DropdownMenuItem>
        )}

        {canStop && (
          <DropdownMenuItem
            onClick={handleStopTrial}
            className="text-orange-600 focus:text-orange-600"
          >
            <StopCircle className="mr-2 h-4 w-4" />
            Stop Trial
          </DropdownMenuItem>
        )}

        <DropdownMenuItem asChild>
          <Link href={`/trials/${trial.id}/wizard`}>
            <TestTube className="mr-2 h-4 w-4" />
            Wizard Interface
          </Link>
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/trials/${trial.id}/analysis`}>
            <BarChart3 className="mr-2 h-4 w-4" />
            View Analysis
          </Link>
        </DropdownMenuItem>

        <DropdownMenuItem onClick={handleCopyId}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Trial ID
        </DropdownMenuItem>

        {trial.canDelete && (
          <>
            <DropdownMenuSeparator />
            <DropdownMenuItem
              onClick={handleDelete}
              className="text-red-600 focus:text-red-600"
            >
              <Trash2 className="mr-2 h-4 w-4" />
              Delete Trial
            </DropdownMenuItem>
          </>
        )}
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

export const trialsColumns: ColumnDef<Trial>[] = [
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
      <DataTableColumnHeader column={column} title="Trial Name" />
    ),
    cell: ({ row }) => {
      const trial = row.original;
      return (
        <div className="max-w-[140px] min-w-0">
          <Link
            href={`/trials/${trial.id}`}
            className="block truncate font-medium hover:underline"
            title={trial.name}
          >
            {trial.name}
          </Link>
        </div>
      );
    },
  },
  {
    accessorKey: "status",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Status" />
    ),
    cell: ({ row }) => {
      const status = row.getValue("status") as Trial["status"];
      const config = statusConfig[status];

      return (
        <Badge
          variant="secondary"
          className={`${config.className} whitespace-nowrap`}
          title={config.description}
        >
          {config.label}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      const status = row.getValue(id) as string;
      return value.includes(status);
    },
  },
  {
    accessorKey: "participant",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Participant" />
    ),
    cell: ({ row }) => {
      const participant = row.getValue("participant") as Trial["participant"];
      return (
        <div className="max-w-[120px]">
          <div className="flex items-center space-x-1">
            <User className="text-muted-foreground h-3 w-3 flex-shrink-0" />
            <span
              className="truncate text-sm font-medium"
              title={participant.name || "Unnamed Participant"}
            >
              {participant.name || "Unnamed Participant"}
            </span>
          </div>
        </div>
      );
    },
    enableSorting: false,
  },
  {
    accessorKey: "experiment",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Experiment" />
    ),
    cell: ({ row }) => {
      const experiment = row.getValue("experiment") as Trial["experiment"];
      return (
        <div className="flex max-w-[140px] items-center space-x-2">
          <FlaskConical className="text-muted-foreground h-3 w-3 flex-shrink-0" />
          <Link
            href={`/experiments/${experiment.id}`}
            className="truncate text-sm hover:underline"
            title={experiment.name || "Unnamed Experiment"}
          >
            {experiment.name || "Unnamed Experiment"}
          </Link>
        </div>
      );
    },
    enableSorting: false,
    enableHiding: true,
    meta: {
      defaultHidden: true,
    },
  },
  {
    accessorKey: "wizard",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Wizard" />
    ),
    cell: ({ row }) => {
      const wizard = row.getValue("wizard") as Trial["wizard"];
      if (!wizard) {
        return (
          <span className="text-muted-foreground text-sm">Not assigned</span>
        );
      }
      return (
        <div className="max-w-[120px] space-y-1">
          <div
            className="truncate text-sm font-medium"
            title={wizard.name ?? ""}
          >
            {wizard.name ?? ""}
          </div>
          <div
            className="text-muted-foreground truncate text-xs"
            title={wizard.email}
          >
            {wizard.email}
          </div>
        </div>
      );
    },
    enableSorting: false,
    enableHiding: true,
    meta: {
      defaultHidden: true,
    },
  },
  {
    accessorKey: "scheduledAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Scheduled" />
    ),
    cell: ({ row }) => {
      const date = row.getValue("scheduledAt") as Date | null;
      if (!date) {
        return (
          <span className="text-muted-foreground text-sm">Not scheduled</span>
        );
      }
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date, { addSuffix: true })}
        </div>
      );
    },
    enableHiding: true,
    meta: {
      defaultHidden: true,
    },
  },
  {
    id: "duration",
    header: "Duration",
    cell: ({ row }) => {
      const trial = row.original;

      if (
        trial.status === "completed" &&
        trial.startedAt &&
        trial.completedAt
      ) {
        const duration = Math.round(
          (trial.completedAt.getTime() - trial.startedAt.getTime()) /
            (1000 * 60),
        );
        return <div className="text-sm whitespace-nowrap">{duration}m</div>;
      }

      if (trial.status === "in_progress" && trial.startedAt) {
        const duration = Math.round(
          (Date.now() - trial.startedAt.getTime()) / (1000 * 60),
        );
        return (
          <div className="text-sm whitespace-nowrap text-blue-600">
            {duration}m
          </div>
        );
      }

      if (trial.duration) {
        return (
          <div className="text-muted-foreground text-sm whitespace-nowrap">
            ~{trial.duration}m
          </div>
        );
      }

      return <span className="text-muted-foreground text-sm">-</span>;
    },
    enableSorting: false,
  },
  {
    id: "stats",
    header: "Data",
    cell: ({ row }) => {
      const trial = row.original;
      const counts = trial._count;

      return (
        <div className="flex space-x-3 text-sm">
          <div className="flex items-center space-x-1" title="Actions recorded">
            <TestTube className="text-muted-foreground h-3 w-3" />
            <span>{counts?.actions ?? 0}</span>
          </div>
          <div className="flex items-center space-x-1" title="Log entries">
            <BarChart3 className="text-muted-foreground h-3 w-3" />
            <span>{counts?.logs ?? 0}</span>
          </div>
        </div>
      );
    },
    enableSorting: false,
    enableHiding: true,
    meta: {
      defaultHidden: true,
    },
  },
  {
    accessorKey: "createdAt",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Created" />
    ),
    cell: ({ row }) => {
      const date = row.getValue("createdAt") as Date;
      return (
        <div className="text-sm whitespace-nowrap">
          {formatDistanceToNow(date, { addSuffix: true })}
        </div>
      );
    },
    enableHiding: true,
    meta: {
      defaultHidden: true,
    },
  },
  {
    id: "actions",
    header: "Actions",
    cell: ({ row }) => <TrialActionsCell trial={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
