"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { ArrowUpDown, ChevronDown, MoreHorizontal } from "lucide-react";
import * as React from "react";

import { format, formatDistanceToNow } from "date-fns";
import { AlertCircle } from "lucide-react";
import Link from "next/link";
import { useEffect } from "react";
import { Alert, AlertDescription } from "~/components/ui/alert";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Card, CardContent } from "~/components/ui/card";
import { Checkbox } from "~/components/ui/checkbox";
import { DataTable } from "~/components/ui/data-table";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { useStudyContext } from "~/lib/study-context";
import { api } from "~/trpc/react";

export type Trial = {
  id: string;
  sessionNumber: number;
  status: "scheduled" | "in_progress" | "completed" | "aborted" | "failed";
  scheduledAt: Date | null;
  startedAt: Date | null;
  completedAt: Date | null;
  createdAt: Date;
  experimentName: string;
  experimentId: string;
  studyName: string;
  studyId: string;
  participantCode: string | null;
  participantName: string | null;
  participantId: string | null;
  wizardName: string | null;
  wizardId: string | null;
  eventCount: number;
  mediaCount: number;
  latestEventAt: Date | null;
};

const statusConfig = {
  scheduled: {
    label: "Scheduled",
    className: "bg-blue-100 text-blue-800",
    icon: "📅",
  },
  in_progress: {
    label: "In Progress",
    className: "bg-yellow-100 text-yellow-800",
    icon: "▶️",
  },
  completed: {
    label: "Completed",
    className: "bg-green-100 text-green-800",
    icon: "✅",
  },
  aborted: {
    label: "Aborted",
    className: "bg-gray-100 text-gray-800",
    icon: "❌",
  },
  failed: {
    label: "Failed",
    className: "bg-red-100 text-red-800",
    icon: "⚠️",
  },
};

export const columns: ColumnDef<Trial>[] = [
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
    accessorKey: "sessionNumber",
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Session
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const sessionNumber = row.getValue("sessionNumber");
      return (
        <div className="font-mono text-sm">
          <Link href={`/trials/${row.original.id}`} className="hover:underline">
            #{Number(sessionNumber)}
          </Link>
        </div>
      );
    },
  },
  {
    accessorKey: "experimentName",
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Experiment
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const experimentName = row.getValue("experimentName");
      const experimentId = row.original.experimentId;
      const studyName = row.original.studyName;
      return (
        <div className="max-w-[250px]">
          <div className="font-medium">
            <Link
              href={`/experiments/${experimentId}`}
              className="truncate hover:underline"
            >
              {String(experimentName)}
            </Link>
          </div>
          <div className="text-muted-foreground truncate text-sm">
            {studyName}
          </div>
        </div>
      );
    },
  },
  {
    accessorKey: "participantCode",
    header: "Participant",
    cell: ({ row }) => {
      const participantCode = row.getValue("participantCode");
      const participantName = row.original?.participantName;
      const participantId = row.original?.participantId;

      if (!participantCode && !participantName) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            No participant
          </Badge>
        );
      }

      return (
        <div className="max-w-[150px]">
          {participantId ? (
            <Link
              href={`/participants/${participantId}`}
              className="font-mono text-sm hover:underline"
            >
              {(participantCode ?? "Unknown") as string}
            </Link>
          ) : (
            <span className="font-mono text-sm">
              {(participantCode ?? "Unknown") as string}
            </span>
          )}
          {participantName && (
            <div className="text-muted-foreground truncate text-xs">
              {participantName}
            </div>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "wizardName",
    header: "Wizard",
    cell: ({ row }) => {
      const wizardName = row.getValue("wizardName");

      if (!wizardName) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            No wizard
          </Badge>
        );
      }

      return (
        <div className="max-w-[150px] truncate text-sm">
          {wizardName as string}
        </div>
      );
    },
  },
  {
    accessorKey: "status",
    header: "Status",
    cell: ({ row }) => {
      const status = row.getValue("status");
      const statusInfo = statusConfig[status as keyof typeof statusConfig];

      if (!statusInfo) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            Unknown
          </Badge>
        );
      }

      return (
        <Badge className={statusInfo.className}>
          <span className="mr-1">{statusInfo.icon}</span>
          {statusInfo.label}
        </Badge>
      );
    },
  },
  {
    accessorKey: "scheduledAt",
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Scheduled
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const scheduledAt = row.getValue("scheduledAt");
      const startedAt = row.original?.startedAt;
      const completedAt = row.original?.completedAt;

      if (completedAt) {
        return (
          <div className="text-sm">
            <div className="font-medium">Completed</div>
            <div className="text-muted-foreground text-xs">
              {formatDistanceToNow(new Date(completedAt), { addSuffix: true })}
            </div>
          </div>
        );
      }

      if (startedAt) {
        return (
          <div className="text-sm">
            <div className="font-medium">Started</div>
            <div className="text-muted-foreground text-xs">
              {formatDistanceToNow(new Date(startedAt), { addSuffix: true })}
            </div>
          </div>
        );
      }

      if (scheduledAt) {
        const scheduleDate =
          scheduledAt != null
            ? new Date(scheduledAt as string | number | Date)
            : null;
        const isUpcoming = scheduleDate && scheduleDate > new Date();
        return (
          <div className="text-sm">
            <div className="font-medium">
              {isUpcoming ? "Upcoming" : "Overdue"}
            </div>
            <div className="text-muted-foreground text-xs">
              {scheduleDate ? format(scheduleDate, "MMM d, h:mm a") : "Unknown"}
            </div>
          </div>
        );
      }

      return (
        <span className="text-muted-foreground text-sm">Not scheduled</span>
      );
    },
  },
  {
    accessorKey: "eventCount",
    header: "Data",
    cell: ({ row }) => {
      const eventCount = row.getValue("eventCount") ?? 0;
      const mediaCount = row.original?.mediaCount ?? 0;
      const latestEventAt = row.original?.latestEventAt
        ? new Date(row.original.latestEventAt)
        : null;

      return (
        <div className="text-sm">
          <div className="flex flex-wrap items-center gap-1">
            <Badge className="bg-purple-100 text-purple-800">
              {Number(eventCount)} events
            </Badge>
            {mediaCount > 0 && (
              <Badge className="bg-orange-100 text-orange-800">
                {mediaCount} media
              </Badge>
            )}
          </div>
          {latestEventAt && (
            <div className="text-muted-foreground mt-1 text-[11px]">
              Last evt:{" "}
              {latestEventAt.toLocaleTimeString([], {
                hour: "2-digit",
                minute: "2-digit",
              })}
            </div>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "createdAt",
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Created
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const date = row.getValue("createdAt");
      if (!date)
        return <span className="text-muted-foreground text-sm">Unknown</span>;

      return (
        <div className="text-muted-foreground text-sm">
          {formatDistanceToNow(new Date(date as string | number | Date), {
            addSuffix: true,
          })}
        </div>
      );
    },
  },
  {
    id: "actions",
    enableHiding: false,
    cell: ({ row }) => {
      const trial = row.original;

      if (!trial?.id) {
        return (
          <span className="text-muted-foreground text-sm">No actions</span>
        );
      }

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
            <DropdownMenuItem
              onClick={() => navigator.clipboard.writeText(trial.id)}
            >
              Copy trial ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/trials/${trial.id}`}>View details</Link>
            </DropdownMenuItem>
            {trial.status === "scheduled" && (
              <DropdownMenuItem asChild>
                <Link href={`/trials/${trial.id}/start`}>Start trial</Link>
              </DropdownMenuItem>
            )}
            {trial.status === "in_progress" && (
              <DropdownMenuItem asChild>
                <Link href={`/trials/${trial.id}/control`}>Control trial</Link>
              </DropdownMenuItem>
            )}
            {trial.status === "completed" && (
              <DropdownMenuItem asChild>
                <Link href={`/trials/${trial.id}/analysis`}>View analysis</Link>
              </DropdownMenuItem>
            )}
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/trials/${trial.id}/edit`}>Edit trial</Link>
            </DropdownMenuItem>
            {(trial.status === "scheduled" || trial.status === "failed") && (
              <DropdownMenuItem className="text-red-600">
                Cancel trial
              </DropdownMenuItem>
            )}
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  },
];

interface TrialsTableProps {
  studyId?: string;
}

export function TrialsTable({ studyId }: TrialsTableProps = {}) {
  const { selectedStudyId } = useStudyContext();
  const [statusFilter, setStatusFilter] = React.useState<string>("all");

  const {
    data: trialsData,
    isLoading,
    error,
    refetch,
  } = api.trials.list.useQuery(
    {
      studyId: studyId ?? selectedStudyId ?? "",
      limit: 50,
    },
    {
      refetchOnWindowFocus: false,
      enabled: !!(studyId ?? selectedStudyId),
    },
  );

  // Refetch when active study changes
  useEffect(() => {
    if (selectedStudyId || studyId) {
      void refetch();
    }
  }, [selectedStudyId, studyId, refetch]);

  // Adapt trials.list payload (no wizard, counts, sessionNumber, scheduledAt in list response)
  const data: Trial[] = React.useMemo(() => {
    if (!Array.isArray(trialsData)) return [];

    interface ListTrial {
      id: string;
      participantId: string | null;
      experimentId: string;
      status: Trial["status"];
      sessionNumber: number | null;
      scheduledAt: Date | null;
      startedAt: Date | null;
      completedAt: Date | null;
      duration: number | null;
      notes: string | null;
      createdAt: Date;
      updatedAt: Date;
      experiment: { id: string; name: string; studyId: string };
      participant?: { id: string; participantCode: string } | null;
      wizard?: {
        id: string | null;
        name: string | null;
        email: string | null;
      } | null;
      eventCount?: number;
      mediaCount?: number;
      latestEventAt?: Date | null;
      userRole: string;
      canAccess: boolean;
    }

    const mapped = (trialsData as ListTrial[]).map((t) => ({
      id: t.id,
      sessionNumber: t.sessionNumber ?? 0,
      status: t.status,
      scheduledAt: t.scheduledAt ?? null,
      startedAt: t.startedAt ?? null,
      completedAt: t.completedAt ?? null,
      createdAt: t.createdAt,
      experimentName: t.experiment.name,
      experimentId: t.experiment.id,
      studyName: "Active Study",
      studyId: t.experiment.studyId,
      participantCode: t.participant?.participantCode ?? null,
      participantName: null,
      participantId: t.participant?.id ?? null,
      wizardName: t.wizard?.name ?? null,
      wizardId: t.wizard?.id ?? null,
      eventCount: t.eventCount ?? 0,
      mediaCount: t.mediaCount ?? 0,
      latestEventAt: t.latestEventAt ?? null,
    }));
    // Apply status filter (if not "all")
    if (statusFilter !== "all") {
      return mapped.filter((t) => t.status === statusFilter);
    }
    return mapped;
  }, [trialsData, statusFilter]);

  if (!selectedStudyId && !studyId) {
    return (
      <Card>
        <CardContent className="pt-6">
          <Alert>
            <AlertCircle className="h-4 w-4" />
            <AlertDescription>
              Please select a study to view trials.
            </AlertDescription>
          </Alert>
        </CardContent>
      </Card>
    );
  }

  if (error) {
    return (
      <Card>
        <CardContent className="pt-6">
          <Alert variant="destructive">
            <AlertCircle className="h-4 w-4" />
            <AlertDescription>
              Failed to load trials: {error.message}
              <Button
                variant="outline"
                size="sm"
                onClick={() => refetch()}
                className="ml-2"
              >
                Try Again
              </Button>
            </AlertDescription>
          </Alert>
        </CardContent>
      </Card>
    );
  }

  const statusFilterComponent = (
    <DropdownMenu>
      <DropdownMenuTrigger asChild>
        <Button variant="outline">
          Status <ChevronDown className="ml-2 h-4 w-4" />
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end">
        <DropdownMenuItem onClick={() => setStatusFilter("all")}>
          All Status
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("scheduled")}>
          Scheduled
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("in_progress")}>
          In Progress
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("completed")}>
          Completed
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("aborted")}>
          Aborted
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("failed")}>
          Failed
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );

  return (
    <DataTable
      columns={columns}
      data={data}
      searchKey="experimentName"
      searchPlaceholder="Filter trials..."
      isLoading={isLoading}
      filters={statusFilterComponent}
    />
  );
}
