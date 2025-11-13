"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  Copy,
  Edit,
  Eye,
  FlaskConical,
  MoreHorizontal,
  TestTube,
  Trash2,
  Users,
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
import { useStudyManagement } from "~/hooks/useStudyManagement";

export type Study = {
  id: string;
  name: string;
  description: string | null;
  status: "draft" | "active" | "completed" | "archived";
  createdAt: Date;
  updatedAt: Date;
  institution: string | null;
  irbProtocolNumber?: string;
  owner: {
    name: string | null;
    email: string;
  };
  _count?: {
    studyMembers: number;
  };
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  isOwner?: boolean;
};

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    description: "Study in preparation",
  },
  active: {
    label: "Active",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    description: "Currently recruiting/running",
  },
  completed: {
    label: "Completed",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    description: "Data collection finished",
  },
  archived: {
    label: "Archived",
    className: "bg-slate-100 text-slate-800 hover:bg-slate-200",
    description: "Study concluded",
  },
};

function StudyActionsCell({ study }: { study: Study }) {
  const { deleteStudy, selectStudy } = useStudyManagement();

  const handleDelete = async () => {
    if (window.confirm(`Are you sure you want to delete "${study.name}"?`)) {
      try {
        await deleteStudy(study.id);
        toast.success("Study deleted successfully");
      } catch {
        toast.error("Failed to delete study");
      }
    }
  };

  const handleCopyId = () => {
    void navigator.clipboard.writeText(study.id);
    toast.success("Study ID copied to clipboard");
  };

  const handleSelect = () => {
    void selectStudy(study.id);
    toast.success(`Selected study: ${study.name}`);
  };

  const canEdit = study.userRole === "owner" || study.userRole === "researcher";
  const canDelete = study.userRole === "owner";

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

        <DropdownMenuItem onClick={handleSelect}>
          <Eye className="mr-2 h-4 w-4" />
          Select & View
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/studies/${study.id}`}>
            <Eye className="mr-2 h-4 w-4" />
            View Details
          </Link>
        </DropdownMenuItem>

        {canEdit && (
          <DropdownMenuItem asChild>
            <Link href={`/studies/${study.id}/edit`}>
              <Edit className="mr-2 h-4 w-4" />
              Edit Study
            </Link>
          </DropdownMenuItem>
        )}

        <DropdownMenuSeparator />

        <DropdownMenuItem onClick={handleCopyId}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Study ID
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/studies/${study.id}/experiments`}>
            <FlaskConical className="mr-2 h-4 w-4" />
            View Experiments
          </Link>
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/studies/${study.id}/participants`}>
            <Users className="mr-2 h-4 w-4" />
            View Participants
          </Link>
        </DropdownMenuItem>

        <DropdownMenuItem asChild>
          <Link href={`/studies/${study.id}/trials`}>
            <TestTube className="mr-2 h-4 w-4" />
            View Trials
          </Link>
        </DropdownMenuItem>

        {canDelete && (
          <>
            <DropdownMenuSeparator />
            <DropdownMenuItem
              onClick={handleDelete}
              className="text-red-600 focus:text-red-600"
            >
              <Trash2 className="mr-2 h-4 w-4" />
              Delete Study
            </DropdownMenuItem>
          </>
        )}
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

export const studiesColumns: ColumnDef<Study>[] = [
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
      <DataTableColumnHeader column={column} title="Study Name" />
    ),
    cell: ({ row }) => {
      const study = row.original;
      return (
        <div className="max-w-[200px] min-w-0 space-y-1">
          <Link
            href={`/studies/${study.id}`}
            className="block truncate font-medium hover:underline"
            title={study.name}
          >
            {study.name}
          </Link>
          {study.description && (
            <p
              className="text-muted-foreground line-clamp-1 truncate text-sm"
              title={study.description}
            >
              {study.description}
            </p>
          )}
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
    accessorKey: "institution",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Institution" />
    ),
    cell: ({ row }) => {
      const institution = row.original.institution;
      return (
        <span
          className="block max-w-[120px] truncate text-sm"
          title={institution ?? undefined}
        >
          {institution ?? "-"}
        </span>
      );
    },
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
    id: "members",
    header: "Members",
    cell: ({ row }) => {
      const study = row.original;
      const counts = study._count;

      return (
        <div className="flex items-center space-x-1 text-sm">
          <Users className="text-muted-foreground h-3 w-3" />
          <span>
            {counts?.studyMembers ?? 0} member
            {(counts?.studyMembers ?? 0) !== 1 ? "s" : ""}
          </span>
        </div>
      );
    },
    enableSorting: false,
    enableHiding: false,
  },
  {
    accessorKey: "userRole",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Your Role" />
    ),
    cell: ({ row }) => {
      const role = row.getValue("userRole");
      if (!role) return "-";

      const roleConfig = {
        owner: { label: "Owner", className: "bg-purple-100 text-purple-800" },
        researcher: {
          label: "Researcher",
          className: "bg-blue-100 text-blue-800",
        },
        wizard: { label: "Wizard", className: "bg-green-100 text-green-800" },
        observer: { label: "Observer", className: "bg-gray-100 text-gray-800" },
      };

      const config = roleConfig[role as keyof typeof roleConfig];

      return (
        <Badge variant="secondary" className={config.className}>
          {config.label}
        </Badge>
      );
    },
    filterFn: (row, id, value: string[]) => {
      return value.includes(row.getValue(id));
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
          {formatDistanceToNow(date ?? new Date(), { addSuffix: true })}
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
          {formatDistanceToNow(date ?? new Date(), { addSuffix: true })}
        </div>
      );
    },
  },
  {
    id: "actions",
    header: "Actions",
    cell: ({ row }) => <StudyActionsCell study={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
