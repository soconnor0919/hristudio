"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { ArrowUpDown, ChevronDown, MoreHorizontal, Play, Gamepad2, LineChart, Ban, Printer } from "lucide-react";
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
  },
  in_progress: {
    label: "In Progress",
    className: "bg-yellow-100 text-yellow-800",
  },
  completed: {
    label: "Completed",
    className: "bg-green-100 text-green-800",
  },
  aborted: {
    label: "Aborted",
    className: "bg-gray-100 text-gray-800",
  },
  failed: {
    label: "Failed",
    className: "bg-red-100 text-red-800",
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
      const status = row.original.status;
      const trialId = row.original.id;
      const studyId = row.original.studyId;

      let href = `/studies/${studyId}/trials/${trialId}`; // Fallback
      if (status === "scheduled" || status === "in_progress") {
        href = `/studies/${studyId}/trials/${trialId}/wizard`;
      } else if (status === "completed") {
        href = `/studies/${studyId}/trials/${trialId}/analysis`;
      } else {
        // for aborted/failed, maybe still link to detail or nowhere?
        // Let's keep detail for now as a fallback for metadata
        href = `/studies/${studyId}/trials/${trialId}`;
      }

      return (
        <div className="font-mono text-sm">
          <Link
            href={href}
            className="hover:underline"
          >
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
      return (
        <div className="max-w-[250px]">
          <div className="truncate font-medium">
            <Link
              href={`/studies/${row.original.studyId}/experiments/${experimentId}`}
              className="hover:underline"
            >
              {String(experimentName)}
            </Link>
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
              href={`/studies/${row.original.studyId}/participants/${participantId}`}
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
    id: "actions",
    enableHiding: false,
    cell: ({ row }) => <ActionsCell row={row} />,
  },
];

function ActionsCell({ row }: { row: { original: Trial } }) {
  const trial = row.original;
  // ActionsCell is a component rendered by the table.

  if (!trial?.id) {
    return <span className="text-muted-foreground text-sm">No actions</span>;
  }

  return (
    <div className="flex items-center gap-2 justify-end">
      {trial.status === "scheduled" && (
        <Button size="sm" asChild>
          <Link href={`/studies/${trial.studyId}/trials/${trial.id}/wizard`}>
            <Play className="mr-1.5 h-3.5 w-3.5" />
            Start
          </Link>
        </Button>
      )}
      {trial.status === "in_progress" && (
        <Button size="sm" variant="secondary" asChild>
          <Link href={`/studies/${trial.studyId}/trials/${trial.id}/wizard`}>
            <Gamepad2 className="mr-1.5 h-3.5 w-3.5" />
            Control
          </Link>
        </Button>
      )}
      {trial.status === "completed" && (
        <>
          <Button size="sm" variant="outline" asChild>
            <Link href={`/studies/${trial.studyId}/trials/${trial.id}/analysis`}>
              <LineChart className="mr-1.5 h-3.5 w-3.5" />
              View
            </Link>
          </Button>
          <Button size="sm" variant="outline" asChild>
            {/* We link to the analysis page with a query param to trigger print/export */}
            <Link href={`/studies/${trial.studyId}/trials/${trial.id}/analysis?export=true`}>
              <Printer className="mr-1.5 h-3.5 w-3.5" />
              Export
            </Link>
          </Button>
        </>
      )}
      {(trial.status === "scheduled" || trial.status === "failed") && (
        <Button size="sm" variant="ghost" className="h-8 w-8 p-0 text-muted-foreground hover:text-red-600">
          <Ban className="h-4 w-4" />
          <span className="sr-only">Cancel</span>
        </Button>
      )}
    </div>
  );
}

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
