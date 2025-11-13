"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { ArrowUpDown, MoreHorizontal } from "lucide-react";
import * as React from "react";

import { formatDistanceToNow } from "date-fns";
import { AlertCircle } from "lucide-react";
import Link from "next/link";
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

export type Experiment = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "testing" | "ready" | "deprecated";
  version: number;
  estimatedDuration: number | null;
  createdAt: Date;
  studyId: string;
  studyName: string;
  createdByName: string;
  trialCount: number;
  stepCount: number;
  actionCount: number;
  latestActivityAt: Date | null;
};

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800",
  },
  testing: {
    label: "Testing",
    className: "bg-yellow-100 text-yellow-800",
  },
  ready: {
    label: "Ready",
    className: "bg-green-100 text-green-800",
  },
  deprecated: {
    label: "Deprecated",
    className: "bg-red-100 text-red-800",
  },
};

export const columns: ColumnDef<Experiment>[] = [
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
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Name
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const name = row.getValue("name");
      const description = row.original.description;
      return (
        <div className="max-w-[200px]">
          <div className="truncate font-medium">
            <Link
              href={`/experiments/${row.original.id}`}
              className="hover:underline"
            >
              {String(name)}
            </Link>
          </div>
          {description && (
            <div className="text-muted-foreground truncate text-sm">
              {description}
            </div>
          )}
        </div>
      );
    },
  },
  // Study column removed (active study context already selected)
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

      return <Badge className={statusInfo.className}>{statusInfo.label}</Badge>;
    },
  },
  {
    accessorKey: "version",
    header: "Version",
    cell: ({ row }) => {
      const version = row.getValue("version");
      return <Badge variant="outline">v{String(version)}</Badge>;
    },
  },
  {
    accessorKey: "stepCount",
    header: "Steps",
    cell: ({ row }) => {
      const stepCount = row.getValue("stepCount");
      return (
        <Badge className="bg-purple-100 text-purple-800">
          {Number(stepCount)} step{Number(stepCount) !== 1 ? "s" : ""}
        </Badge>
      );
    },
  },
  {
    accessorKey: "actionCount",
    header: "Actions",
    cell: ({ row }) => {
      const actionCount = row.getValue("actionCount");
      return (
        <Badge className="bg-indigo-100 text-indigo-800">
          {Number(actionCount)} action{Number(actionCount) !== 1 ? "s" : ""}
        </Badge>
      );
    },
  },
  {
    accessorKey: "trialCount",
    header: "Trials",
    cell: ({ row }) => {
      const trialCount = row.getValue("trialCount");
      if (trialCount === 0) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            No trials
          </Badge>
        );
      }
      return (
        <Badge className="bg-blue-100 text-blue-800">
          {Number(trialCount)} trial{Number(trialCount) !== 1 ? "s" : ""}
        </Badge>
      );
    },
  },
  {
    accessorKey: "latestActivityAt",
    header: "Last Activity",
    cell: ({ row }) => {
      const ts = row.getValue("latestActivityAt");
      if (!ts) {
        return <span className="text-muted-foreground text-sm">—</span>;
      }
      return (
        <span className="text-sm">
          {formatDistanceToNow(new Date(ts as string | number | Date), {
            addSuffix: true,
          })}
        </span>
      );
    },
  },
  {
    accessorKey: "estimatedDuration",
    header: "Duration",
    cell: ({ row }) => {
      const duration = row.getValue("estimatedDuration");
      if (!duration) {
        return <span className="text-muted-foreground text-sm">—</span>;
      }
      return <span className="text-sm">{Number(duration)}m</span>;
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
      const experiment = row.original;

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
              onClick={() => navigator.clipboard.writeText(experiment.id)}
            >
              Copy experiment ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/experiments/${experiment.id}`}>View details</Link>
            </DropdownMenuItem>
            <DropdownMenuItem asChild>
              <Link href={`/experiments/${experiment.id}/edit`}>
                Edit experiment
              </Link>
            </DropdownMenuItem>
            <DropdownMenuItem asChild>
              <Link href={`/experiments/${experiment.id}/designer`}>
                Open designer
              </Link>
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link
                href={`/studies/${experiment.studyId}/trials/new?experimentId=${experiment.id}`}
              >
                Create trial
              </Link>
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem className="text-red-600">
              Archive experiment
            </DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  },
];

export function ExperimentsTable() {
  const { selectedStudyId } = useStudyContext();

  const {
    data: experimentsData,
    isLoading,
    error,
    refetch,
  } = api.experiments.list.useQuery(
    {
      studyId: selectedStudyId ?? "",
    },
    {
      refetchOnWindowFocus: false,
      enabled: !!selectedStudyId,
    },
  );

  const data: Experiment[] = React.useMemo(() => {
    if (!experimentsData) return [];

    interface RawExperiment {
      id: string;
      name: string;
      description?: string | null;
      status: Experiment["status"];
      version: number;
      estimatedDuration?: number | null;
      createdAt: string | Date;
      studyId: string;
      createdBy?: { name?: string | null; email?: string | null } | null;
      trialCount?: number | null;
      stepCount?: number | null;
      actionCount?: number | null;
      latestActivityAt?: string | Date | null;
    }

    const adapt = (exp: RawExperiment): Experiment => {
      const createdAt =
        exp.createdAt instanceof Date ? exp.createdAt : new Date(exp.createdAt);
      const latestActivityAt = exp.latestActivityAt
        ? exp.latestActivityAt instanceof Date
          ? exp.latestActivityAt
          : new Date(exp.latestActivityAt)
        : null;
      return {
        id: exp.id,
        name: exp.name,
        description: exp.description ?? "",
        status: exp.status,
        version: exp.version,
        estimatedDuration: exp.estimatedDuration ?? 0,
        createdAt,
        studyId: exp.studyId,
        studyName: "Active Study",
        createdByName: exp.createdBy?.name ?? exp.createdBy?.email ?? "Unknown",
        trialCount: exp.trialCount ?? 0,
        stepCount: exp.stepCount ?? 0,
        actionCount: exp.actionCount ?? 0,
        latestActivityAt,
      };
    };

    return experimentsData.map((e) => adapt(e as unknown as RawExperiment));
  }, [experimentsData]);

  if (!selectedStudyId) {
    return (
      <Card>
        <CardContent className="pt-6">
          <Alert>
            <AlertCircle className="h-4 w-4" />
            <AlertDescription>
              Please select a study to view experiments.
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
              Failed to load experiments: {error.message}
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

  return (
    <div className="space-y-4">
      <DataTable
        columns={columns}
        data={data}
        searchKey="name"
        searchPlaceholder="Filter experiments..."
        isLoading={isLoading}
      />
    </div>
  );
}
