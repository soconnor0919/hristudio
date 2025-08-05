"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { ArrowUpDown, MoreHorizontal } from "lucide-react";
import * as React from "react";

import { formatDistanceToNow } from "date-fns";
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
    DropdownMenuTrigger
} from "~/components/ui/dropdown-menu";
import { useActiveStudy } from "~/hooks/useActiveStudy";
import { api } from "~/trpc/react";

export type Participant = {
  id: string;
  participantCode: string;
  email: string | null;
  name: string | null;
  consentGiven: boolean;
  consentDate: Date | null;
  createdAt: Date;
  trialCount: number;
};

export const columns: ColumnDef<Participant>[] = [
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
    accessorKey: "participantCode",
    header: ({ column }) => {
      return (
        <Button
          variant="ghost"
          onClick={() => column.toggleSorting(column.getIsSorted() === "asc")}
        >
          Code
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => (
      <div className="font-mono text-sm">
        <Link
          href={`/participants/${row.original.id}`}
          className="hover:underline"
        >
          {row.getValue("participantCode")}
        </Link>
      </div>
    ),
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
      const email = row.original.email;
      return (
        <div>
          <div className="truncate font-medium">
            {String(name) || "No name provided"}
          </div>
          {email && (
            <div className="text-muted-foreground truncate text-sm">
              {email}
            </div>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "consentGiven",
    header: "Consent",
    cell: ({ row }) => {
      const consentGiven = row.getValue("consentGiven");

      if (consentGiven) {
        return <Badge className="bg-green-100 text-green-800">Consented</Badge>;
      }

      return <Badge className="bg-red-100 text-red-800">Pending</Badge>;
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
          {formatDistanceToNow(new Date(date as string | number | Date), { addSuffix: true })}
        </div>
      );
    },
  },
  {
    id: "actions",
    enableHiding: false,
    cell: ({ row }) => {
      const participant = row.original;

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
              onClick={() => navigator.clipboard.writeText(participant.id)}
            >
              Copy participant ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/participants/${participant.id}`}>View details</Link>
            </DropdownMenuItem>
            <DropdownMenuItem asChild>
              <Link href={`/participants/${participant.id}/edit`}>
                Edit participant
              </Link>
            </DropdownMenuItem>
            {!participant.consentGiven && (
              <DropdownMenuItem>Send consent form</DropdownMenuItem>
            )}
            <DropdownMenuSeparator />
            <DropdownMenuItem className="text-red-600">
              Remove participant
            </DropdownMenuItem>
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  },
];

interface ParticipantsTableProps {
  studyId?: string;
}

export function ParticipantsTable({ studyId }: ParticipantsTableProps = {}) {
  const { activeStudy } = useActiveStudy();

  const {
    data: participantsData,
    isLoading,
    error,
    refetch,
  } = api.participants.list.useQuery(
    {
      studyId: studyId ?? activeStudy?.id ?? "",
    },
    {
      refetchOnWindowFocus: false,
      enabled: !!(studyId ?? activeStudy?.id),
    },
  );

  // Refetch when active study changes
  useEffect(() => {
    if (activeStudy?.id || studyId) {
      refetch();
    }
  }, [activeStudy?.id, studyId, refetch]);

  const data: Participant[] = React.useMemo(() => {
    if (!participantsData?.participants) return [];

    return participantsData.participants.map((p) => ({
      id: p.id,
      participantCode: p.participantCode,
      email: p.email,
      name: p.name,
      consentGiven: p.hasConsent,
      consentDate: p.latestConsent?.signedAt
        ? new Date(p.latestConsent.signedAt as unknown as string)
        : null,
      createdAt: p.createdAt,
      trialCount: p.trialCount,
    }));
  }, [participantsData]);

  if (!studyId && !activeStudy) {
    return (
      <Card>
        <CardContent className="pt-6">
          <Alert>
            <AlertCircle className="h-4 w-4" />
            <AlertDescription>
              Please select a study to view participants.
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
              Failed to load participants: {error.message}
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
        searchPlaceholder="Filter participants..."
        isLoading={isLoading}
      />
    </div>
  );
}
