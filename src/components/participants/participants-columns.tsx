"use client";

import { type ColumnDef } from "@tanstack/react-table";
import { formatDistanceToNow } from "date-fns";
import {
  MoreHorizontal,
  Eye,
  Edit,
  Trash2,
  Copy,
  User,
  Mail,
  TestTube,
} from "lucide-react";
import Link from "next/link";

import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import { Checkbox } from "~/components/ui/checkbox";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { DataTableColumnHeader } from "~/components/ui/data-table-column-header";
import { toast } from "sonner";

export type Participant = {
  id: string;
  participantCode: string;
  email: string | null;
  name: string | null;
  consentGiven: boolean;
  consentDate: Date | null;
  createdAt: Date;
  trialCount: number;
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  canEdit?: boolean;
  canDelete?: boolean;
};

function ParticipantActionsCell({ participant }: { participant: Participant }) {
  const handleDelete = async () => {
    if (
      window.confirm(
        `Are you sure you want to delete participant "${participant.name ?? participant.participantCode}"?`,
      )
    ) {
      try {
        // TODO: Implement delete participant mutation
        toast.success("Participant deleted successfully");
      } catch {
        toast.error("Failed to delete participant");
      }
    }
  };

  const handleCopyId = () => {
    void navigator.clipboard.writeText(participant.id);
    toast.success("Participant ID copied to clipboard");
  };

  const handleCopyCode = () => {
    void navigator.clipboard.writeText(participant.participantCode);
    toast.success("Participant code copied to clipboard");
  };

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
          <Link href={`/participants/${participant.id}`}>
            <Eye className="mr-2 h-4 w-4" />
            View Details
          </Link>
        </DropdownMenuItem>

        {participant.canEdit && (
          <DropdownMenuItem asChild>
            <Link href={`/participants/${participant.id}/edit`}>
              <Edit className="mr-2 h-4 w-4" />
              Edit Participant
            </Link>
          </DropdownMenuItem>
        )}

        <DropdownMenuSeparator />

        <DropdownMenuItem onClick={handleCopyId}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Participant ID
        </DropdownMenuItem>

        <DropdownMenuItem onClick={handleCopyCode}>
          <Copy className="mr-2 h-4 w-4" />
          Copy Participant Code
        </DropdownMenuItem>

        {!participant.consentGiven && (
          <DropdownMenuItem>
            <Mail className="mr-2 h-4 w-4" />
            Send Consent Form
          </DropdownMenuItem>
        )}

        {participant.canDelete && (
          <>
            <DropdownMenuSeparator />
            <DropdownMenuItem
              onClick={handleDelete}
              className="text-red-600 focus:text-red-600"
            >
              <Trash2 className="mr-2 h-4 w-4" />
              Delete Participant
            </DropdownMenuItem>
          </>
        )}
      </DropdownMenuContent>
    </DropdownMenu>
  );
}

export const participantsColumns: ColumnDef<Participant>[] = [
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
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Code" />
    ),
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
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Name" />
    ),
    cell: ({ row }) => {
      const name = row.getValue("name") as string | null;
      const email = row.original.email;
      return (
        <div className="max-w-[160px] space-y-1">
          <div className="flex items-center space-x-2">
            <User className="text-muted-foreground h-3 w-3 flex-shrink-0" />
            <span
              className="truncate font-medium"
              title={name ?? "No name provided"}
            >
              {name ?? "No name provided"}
            </span>
          </div>
          {email && (
            <div className="text-muted-foreground flex items-center space-x-1 text-xs">
              <Mail className="h-3 w-3 flex-shrink-0" />
              <span className="truncate" title={email}>
                {email}
              </span>
            </div>
          )}
        </div>
      );
    },
  },
  {
    accessorKey: "consentGiven",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Consent" />
    ),
    cell: ({ row }) => {
      const consentGiven = row.getValue("consentGiven");
      const consentDate = row.original.consentDate;

      if (consentGiven) {
        return (
          <Badge
            variant="secondary"
            className="bg-green-100 whitespace-nowrap text-green-800"
            title={
              consentDate
                ? `Consented on ${consentDate.toLocaleDateString()}`
                : "Consented"
            }
          >
            Consented
          </Badge>
        );
      }

      return (
        <Badge
          variant="secondary"
          className="bg-red-100 whitespace-nowrap text-red-800"
        >
          Pending
        </Badge>
      );
    },
    filterFn: (row, id, value) => {
      const consentGiven = row.getValue(id) as boolean;
      if (value === "consented") return !!consentGiven;
      if (value === "pending") return !consentGiven;
      return true;
    },
  },
  {
    accessorKey: "trialCount",
    header: ({ column }) => (
      <DataTableColumnHeader column={column} title="Trials" />
    ),
    cell: ({ row }) => {
      const trialCount = row.getValue("trialCount") as number;

      return (
        <div className="flex items-center space-x-1 text-sm whitespace-nowrap">
          <TestTube className="text-muted-foreground h-3 w-3" />
          <span>{trialCount as number}</span>
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
      const date = row.getValue("createdAt") as Date;
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
    cell: ({ row }) => <ParticipantActionsCell participant={row.original} />,
    enableSorting: false,
    enableHiding: false,
  },
];
