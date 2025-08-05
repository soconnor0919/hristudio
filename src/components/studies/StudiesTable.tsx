"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { ArrowUpDown, MoreHorizontal } from "lucide-react";
import * as React from "react";

import { formatDistanceToNow } from "date-fns";
import { AlertCircle, Filter } from "lucide-react";
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
    DropdownMenuTrigger
} from "~/components/ui/dropdown-menu";
import { api } from "~/trpc/react";

type StudyFromAPI = {
  id: string;
  name: string;
  description: string;
  status: "draft" | "active" | "completed" | "archived";
  institution: string;
  irbProtocolNumber: string | null;
  createdAt: Date;
  ownerId: string;
  createdBy: {
    id: string;
    name: string | null;
    email: string;
  };
  members: Array<{
    role: "owner" | "researcher" | "wizard" | "observer";
    user: {
      id: string;
      name: string | null;
      email: string;
    };
  }>;
  experiments?: Array<{ id: string }>;
  participants?: Array<{ id: string }>;
};

export type Study = {
  id: string;
  name: string;
  description: string;
  status: "draft" | "active" | "completed" | "archived";
  institution: string;
  irbProtocolNumber: string | null;
  createdAt: Date;
  createdByName: string;
  memberCount: number;
  experimentCount: number;
  participantCount: number;
  userRole: string;
  isOwner: boolean;
};

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800",
    icon: "📝",
  },
  active: {
    label: "Active",
    className: "bg-green-100 text-green-800",
    icon: "🟢",
  },
  completed: {
    label: "Completed",
    className: "bg-blue-100 text-blue-800",
    icon: "✅",
  },
  archived: {
    label: "Archived",
    className: "bg-orange-100 text-orange-800",
    icon: "📦",
  },
};

export const columns: ColumnDef<Study>[] = [
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
          Study Name
          <ArrowUpDown className="ml-2 h-4 w-4" />
        </Button>
      );
    },
    cell: ({ row }) => {
      const name = row.getValue("name");
      const description = row.original.description;
      return (
        <div className="max-w-[250px]">
          <div className="truncate font-medium">
            <Link
              href={`/studies/${row.original.id}`}
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
  {
    accessorKey: "institution",
    header: "Institution",
    cell: ({ row }) => {
      const institution = row.getValue("institution");
      const irbProtocol = row.original.irbProtocolNumber;
      return (
        <div className="max-w-[150px]">
          <div className="truncate font-medium">{String(institution)}</div>
          {irbProtocol && (
            <div className="text-muted-foreground truncate text-sm">
              IRB: {irbProtocol}
            </div>
          )}
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
      return (
        <Badge className={statusInfo.className}>
          <span className="mr-1">{statusInfo.icon}</span>
          {statusInfo.label}
        </Badge>
      );
    },
  },
  {
    accessorKey: "userRole",
    header: "Your Role",
    cell: ({ row }) => {
      const userRole = row.getValue("userRole");
      const isOwner = row.original.isOwner;

      return (
        <Badge variant={isOwner ? "default" : "secondary"}>{String(userRole)}</Badge>
      );
    },
  },
  {
    accessorKey: "memberCount",
    header: "Team",
    cell: ({ row }) => {
      const memberCount = row.getValue("memberCount");
      return (
        <Badge className="bg-purple-100 text-purple-800">
          {Number(memberCount)} member{Number(memberCount) !== 1 ? "s" : ""}
        </Badge>
      );
    },
  },
  {
    accessorKey: "experimentCount",
    header: "Experiments",
    cell: ({ row }) => {
      const experimentCount = row.getValue("experimentCount");
      if (experimentCount === 0) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            None
          </Badge>
        );
      }
      return (
        <Badge className="bg-blue-100 text-blue-800">{Number(experimentCount)}</Badge>
      );
    },
  },
  {
    accessorKey: "participantCount",
    header: "Participants",
    cell: ({ row }) => {
      const participantCount = row.getValue("participantCount");
      if (participantCount === 0) {
        return (
          <Badge variant="outline" className="text-muted-foreground">
            None
          </Badge>
        );
      }
      return (
        <Badge className="bg-green-100 text-green-800">
          {Number(participantCount)}
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
      const createdBy = row.original.createdByName;
      return (
        <div className="max-w-[120px]">
          <div className="text-sm">
            {formatDistanceToNow(new Date(date as string | number | Date), { addSuffix: true })}
          </div>
          <div className="text-muted-foreground truncate text-xs">
            by {createdBy}
          </div>
        </div>
      );
    },
  },
  {
    id: "actions",
    enableHiding: false,
    cell: ({ row }) => {
      const study = row.original;
      const canEdit =
        study.isOwner ||
        study.userRole === "owner" ||
        study.userRole === "researcher";

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
              onClick={() => navigator.clipboard.writeText(study.id)}
            >
              Copy study ID
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            <DropdownMenuItem asChild>
              <Link href={`/studies/${study.id}`}>View details</Link>
            </DropdownMenuItem>
            {canEdit && (
              <DropdownMenuItem asChild>
                <Link href={`/studies/${study.id}/edit`}>Edit study</Link>
              </DropdownMenuItem>
            )}
            <DropdownMenuItem asChild>
              <Link href={`/studies/${study.id}/experiments`}>
                View experiments
              </Link>
            </DropdownMenuItem>
            <DropdownMenuItem asChild>
              <Link href={`/studies/${study.id}/participants`}>
                View participants
              </Link>
            </DropdownMenuItem>
            <DropdownMenuSeparator />
            {canEdit && study.status === "draft" && (
              <DropdownMenuItem className="text-red-600">
                Archive study
              </DropdownMenuItem>
            )}
          </DropdownMenuContent>
        </DropdownMenu>
      );
    },
  },
];

export function StudiesTable() {
  const [statusFilter, setStatusFilter] = React.useState("all");

  const {
    data: studiesData,
    isLoading,
    error,
    refetch,
  } = api.studies.list.useQuery(
    {
      memberOnly: true,
      status:
        statusFilter === "all"
          ? undefined
          : (statusFilter as "draft" | "active" | "completed" | "archived"),
    },
    {
      refetchOnWindowFocus: false,
    },
  );

  const { data: session, isLoading: isSessionLoading } = api.auth.me.useQuery();

  const data: Study[] = React.useMemo(() => {
    if (!studiesData?.studies || !session) return [];

    return (studiesData.studies as StudyFromAPI[]).map((study) => {
      // Find current user's membership
      const currentUserId = session?.id;
      const userMembership = study.members?.find(
        (member) => member.user.id === currentUserId,
      );

      return {
        id: study.id,
        name: study.name,
        description: study.description,
        status: study.status,
        institution: study.institution,
        irbProtocolNumber: study.irbProtocolNumber,
        createdAt: study.createdAt,
        createdByName:
          study.createdBy?.name ?? study.createdBy?.email ?? "Unknown",
        memberCount: study.members?.length ?? 0,
        experimentCount: study.experiments?.length ?? 0,
        participantCount: study.participants?.length ?? 0,
        userRole: userMembership?.role ?? "observer",
        isOwner: study.ownerId === currentUserId,
      };
    });
  }, [studiesData, session]);

  if (error) {
    return (
      <Card>
        <CardContent className="pt-6">
          <Alert variant="destructive">
            <AlertCircle className="h-4 w-4" />
            <AlertDescription>
              Failed to load studies: {error.message}
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
        <Button variant="outline" size="sm">
          <Filter className="mr-2 h-4 w-4" />
          {statusFilter === "all"
            ? "All Status"
            : statusFilter.charAt(0).toUpperCase() + statusFilter.slice(1)}
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end">
        <DropdownMenuItem onClick={() => setStatusFilter("all")}>
          All Status
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("draft")}>
          Draft
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("active")}>
          Active
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("completed")}>
          Completed
        </DropdownMenuItem>
        <DropdownMenuItem onClick={() => setStatusFilter("archived")}>
          Archived
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  );

  return (
    <div className="space-y-4">
      <DataTable
        columns={columns}
        data={data}
        searchKey="name"
        searchPlaceholder="Filter studies..."
        isLoading={isLoading || isSessionLoading}
        filters={statusFilterComponent}
      />
    </div>
  );
}
