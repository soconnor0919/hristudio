"use client";

import {
    type ColumnDef,
    type ColumnFiltersState,
    type SortingState,
    type VisibilityState,
    flexRender,
    getCoreRowModel,
    getFilteredRowModel,
    getPaginationRowModel,
    getSortedRowModel,
    useReactTable,
} from "@tanstack/react-table";
import {
    Table,
    TableBody,
    TableCell,
    TableHead,
    TableHeader,
    TableRow,
} from "~/components/ui/table";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { useState } from "react";
import {
    ArrowUpDown,
    MoreHorizontal,
    Calendar,
    Clock,
    Activity,
    Eye,
    Video
} from "lucide-react";
import {
    DropdownMenu,
    DropdownMenuContent,
    DropdownMenuItem,
    DropdownMenuLabel,
    DropdownMenuSeparator,
    DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { Badge } from "~/components/ui/badge";
import Link from "next/link";
import { formatDistanceToNow } from "date-fns";

export type AnalyticsTrial = {
    id: string;
    sessionNumber: number;
    status: string;
    createdAt: Date;
    startedAt: Date | null;
    completedAt: Date | null;
    duration: number | null;
    eventCount: number;
    mediaCount: number;
    experimentId: string;
    participant: {
        participantCode: string;
    };
    experiment: {
        name: string;
        studyId: string;
    };
};

export const columns: ColumnDef<AnalyticsTrial>[] = [
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
        cell: ({ row }) => <div className="font-mono text-center">#{row.getValue("sessionNumber")}</div>,
    },
    {
        accessorKey: "participant.participantCode",
        id: "participantCode",
        header: "Participant",
        cell: ({ row }) => (
            <div className="font-medium">{row.original.participant?.participantCode ?? "Unknown"}</div>
        ),
    },
    {
        accessorKey: "status",
        header: "Status",
        cell: ({ row }) => {
            const status = row.getValue("status") as string;
            return (
                <Badge
                    variant="outline"
                    className={`capitalize ${status === "completed"
                        ? "bg-green-500/10 text-green-500 border-green-500/20"
                        : status === "in_progress"
                            ? "bg-blue-500/10 text-blue-500 border-blue-500/20"
                            : "bg-slate-500/10 text-slate-500 border-slate-500/20"
                        }`}
                >
                    {status.replace("_", " ")}
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
                    Date
                    <ArrowUpDown className="ml-2 h-4 w-4" />
                </Button>
            );
        },
        cell: ({ row }) => {
            const date = new Date(row.getValue("createdAt"));
            return (
                <div className="flex flex-col">
                    <span className="text-sm">{date.toLocaleDateString()}</span>
                    <span className="text-xs text-muted-foreground">{formatDistanceToNow(date, { addSuffix: true })}</span>
                </div>
            )
        },
    },
    {
        accessorKey: "duration",
        header: "Duration",
        cell: ({ row }) => {
            const duration = row.getValue("duration") as number | null;
            if (!duration) return <span className="text-muted-foreground">-</span>;
            const m = Math.floor(duration / 60);
            const s = Math.floor(duration % 60);
            return <div className="font-mono">{`${m}m ${s}s`}</div>;
        },
    },
    {
        accessorKey: "eventCount",
        header: "Events",
        cell: ({ row }) => {
            return (
                <div className="flex items-center gap-1">
                    <Activity className="h-3 w-3 text-muted-foreground" />
                    <span>{row.getValue("eventCount")}</span>
                </div>
            )
        },
    },
    {
        accessorKey: "mediaCount",
        header: "Media",
        cell: ({ row }) => {
            const count = row.getValue("mediaCount") as number;
            if (count === 0) return <span className="text-muted-foreground">-</span>;
            return (
                <div className="flex items-center gap-1">
                    <Video className="h-3 w-3 text-muted-foreground" />
                    <span>{count}</span>
                </div>
            )
        },
    },
    {
        id: "actions",
        cell: ({ row }) => {
            const trial = row.original;
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
                        <DropdownMenuItem asChild>
                            <Link href={`/studies/${trial.experiment?.studyId}/trials/${trial.id}/analysis`}>
                                <Eye className="mr-2 h-4 w-4" />
                                View Analysis
                            </Link>
                        </DropdownMenuItem>
                        <DropdownMenuItem asChild>
                            <Link href={`/experiments/${trial.experimentId}/trials/${trial.id}`}>
                                View Trial Details
                            </Link>
                        </DropdownMenuItem>
                    </DropdownMenuContent>
                </DropdownMenu>
            );
        },
    },
];

interface StudyAnalyticsDataTableProps {
    data: AnalyticsTrial[];
}

export function StudyAnalyticsDataTable({ data }: StudyAnalyticsDataTableProps) {
    const [sorting, setSorting] = useState<SortingState>([]);
    const [columnFilters, setColumnFilters] = useState<ColumnFiltersState>([]);
    const [columnVisibility, setColumnVisibility] = useState<VisibilityState>({});
    const [rowSelection, setRowSelection] = useState({});

    const table = useReactTable({
        data,
        columns,
        onSortingChange: setSorting,
        onColumnFiltersChange: setColumnFilters,
        getCoreRowModel: getCoreRowModel(),
        getPaginationRowModel: getPaginationRowModel(),
        getSortedRowModel: getSortedRowModel(),
        getFilteredRowModel: getFilteredRowModel(),
        onColumnVisibilityChange: setColumnVisibility,
        onRowSelectionChange: setRowSelection,
        state: {
            sorting,
            columnFilters,
            columnVisibility,
            rowSelection,
        },
    });

    return (
        <div className="w-full">
            <div className="flex items-center py-4">
                <Input
                    placeholder="Filter participants..."
                    value={(table.getColumn("participantCode")?.getFilterValue() as string) ?? ""}
                    onChange={(event) =>
                        table.getColumn("participantCode")?.setFilterValue(event.target.value)
                    }
                    className="max-w-sm"
                />
            </div>
            <div className="rounded-md border bg-card">
                <Table>
                    <TableHeader>
                        {table.getHeaderGroups().map((headerGroup) => (
                            <TableRow key={headerGroup.id}>
                                {headerGroup.headers.map((header) => {
                                    return (
                                        <TableHead key={header.id}>
                                            {header.isPlaceholder
                                                ? null
                                                : flexRender(
                                                    header.column.columnDef.header,
                                                    header.getContext()
                                                )}
                                        </TableHead>
                                    );
                                })}
                            </TableRow>
                        ))}
                    </TableHeader>
                    <TableBody>
                        {table.getRowModel().rows?.length ? (
                            table.getRowModel().rows.map((row) => (
                                <TableRow
                                    key={row.id}
                                    data-state={row.getIsSelected() && "selected"}
                                >
                                    {row.getVisibleCells().map((cell) => (
                                        <TableCell key={cell.id}>
                                            {flexRender(
                                                cell.column.columnDef.cell,
                                                cell.getContext()
                                            )}
                                        </TableCell>
                                    ))}
                                </TableRow>
                            ))
                        ) : (
                            <TableRow>
                                <TableCell
                                    colSpan={columns.length}
                                    className="h-24 text-center"
                                >
                                    No results.
                                </TableCell>
                            </TableRow>
                        )}
                    </TableBody>
                </Table>
            </div>
            <div className="flex items-center justify-end space-x-2 py-4">
                <div className="flex-1 text-sm text-muted-foreground">
                    {table.getFilteredSelectedRowModel().rows.length} of{" "}
                    {table.getFilteredRowModel().rows.length} row(s) selected.
                </div>
                <div className="space-x-2">
                    <Button
                        variant="outline"
                        size="sm"
                        onClick={() => table.previousPage()}
                        disabled={!table.getCanPreviousPage()}
                    >
                        Previous
                    </Button>
                    <Button
                        variant="outline"
                        size="sm"
                        onClick={() => table.nextPage()}
                        disabled={!table.getCanNextPage()}
                    >
                        Next
                    </Button>
                </div>
            </div>
        </div>
    );
}
