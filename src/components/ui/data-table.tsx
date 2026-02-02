"use client";

import {
  flexRender,
  getCoreRowModel,
  getFilteredRowModel,
  getPaginationRowModel,
  getSortedRowModel,
  useReactTable,
  type ColumnDef,
  type ColumnFiltersState,
  type SortingState,
  type VisibilityState,
} from "@tanstack/react-table";
import { ChevronDown } from "lucide-react";
import * as React from "react";

import { Button } from "~/components/ui/button";
import {
  DropdownMenu,
  DropdownMenuCheckboxItem,
  DropdownMenuContent,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { Input } from "~/components/ui/input";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
// Remove unused import

// Safe flexRender wrapper to prevent undefined className errors
function safeFlexRender(component: unknown, props: unknown) {
  try {
    if (!component || component === null || component === undefined) {
      return <span>-</span>;
    }

    // Ensure props is always an object
    const safeProps = props && typeof props === "object" ? props : {};

    if (typeof component === "function") {
      try {
        const result = (component as (props: unknown) => React.ReactNode)(
          safeProps,
        );
        // Check if result is a valid React element or component
        if (result === null || result === undefined || result === false) {
          return <span>-</span>;
        }
        return result;
      } catch (funcError) {
        console.error("Component function error:", funcError);
        return <span>-</span>;
      }
    }

    // For non-function components, use flexRender with extra safety
    if (typeof component === "string" || React.isValidElement(component)) {
      return flexRender(
        component as unknown as React.ComponentType<unknown>,
        safeProps,
      );
    }

    // If component is an object but not a valid React element
    if (typeof component === "object") {
      console.warn("Invalid component object:", component);
      return <span>-</span>;
    }

    return flexRender(
      component as unknown as React.ComponentType<unknown>,
      safeProps,
    );
  } catch (_error) {
    console.error("FlexRender error:", _error, "Component:", component);
    return <span className="text-xs text-red-500">Error</span>;
  }
}

interface DataTableProps<TData, TValue> {
  columns: ColumnDef<TData, TValue>[];
  data: TData[];
  searchKey?: string;
  searchPlaceholder?: string;
  isLoading?: boolean;
  loadingRowCount?: number;
  filters?: React.ReactNode;
}

export function DataTable<TData, TValue>({
  columns,
  data,
  searchKey,
  searchPlaceholder = "Search...",
  isLoading = false,
  loadingRowCount = 5,
  filters,
}: DataTableProps<TData, TValue>) {
  // Safety checks before hooks
  const safeColumns = columns && Array.isArray(columns) ? columns : [];
  const safeData = data && Array.isArray(data) ? data : [];

  const [sorting, setSorting] = React.useState<SortingState>([]);
  const [columnFilters, setColumnFilters] = React.useState<ColumnFiltersState>(
    [],
  );
  const [columnVisibility, setColumnVisibility] =
    React.useState<VisibilityState>(() => {
      // Initialize with defaultHidden columns set to false
      const initialVisibility: VisibilityState = {};
      safeColumns.forEach((column) => {
        const meta = column.meta as { defaultHidden?: boolean } | undefined;
        if (meta?.defaultHidden) {
          const columnKey =
            column.id ?? (column as { accessorKey?: string }).accessorKey;
          if (columnKey) {
            initialVisibility[columnKey] = false;
          }
        }
      });
      return initialVisibility;
    });
  const [rowSelection, setRowSelection] = React.useState({});

  const table = useReactTable({
    data: safeData,
    columns: safeColumns,
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

  // Safety checks after table creation
  if (!columns || !Array.isArray(columns) || columns.length === 0) {
    return (
      <div className="text-muted-foreground w-full p-4 text-center">
        No table configuration available
      </div>
    );
  }

  if (!data || !Array.isArray(data)) {
    return (
      <div className="text-muted-foreground w-full p-4 text-center">
        No data available
      </div>
    );
  }

  return (
    <div className="w-full min-w-0 space-y-4">
      <div className="flex min-w-0 items-center justify-between">
        <div className="flex min-w-0 flex-1 items-center space-x-2">
          {searchKey && (
            <Input
              placeholder={searchPlaceholder}
              value={
                (table.getColumn(searchKey)?.getFilterValue() as string) ?? ""
              }
              onChange={(event) =>
                table.getColumn(searchKey)?.setFilterValue(event.target.value)
              }
              className="h-8 w-[150px] flex-shrink-0 lg:w-[250px]"
            />
          )}
          <div className="min-w-0 flex-1">{filters}</div>
        </div>
        <div className="flex-shrink-0">
          <DropdownMenu>
            <DropdownMenuTrigger asChild>
              <Button variant="outline" className="h-8">
                Columns <ChevronDown className="ml-2 h-4 w-4" />
              </Button>
            </DropdownMenuTrigger>
            <DropdownMenuContent align="end">
              {table
                .getAllColumns()
                .filter((column) => column.getCanHide())
                .map((column) => {
                  return (
                    <DropdownMenuCheckboxItem
                      key={column.id}
                      className="capitalize"
                      checked={column.getIsVisible()}
                      onCheckedChange={(value) =>
                        column.toggleVisibility(!!value)
                      }
                    >
                      {column.id}
                    </DropdownMenuCheckboxItem>
                  );
                })}
            </DropdownMenuContent>
          </DropdownMenu>
        </div>
      </div>
      <div className="min-w-0 overflow-hidden rounded-md border shadow-sm bg-card">
        <div className="min-w-0 overflow-x-auto overflow-y-hidden">
          <Table className="min-w-[600px]">
            <TableHeader>
              {table.getHeaderGroups().map((headerGroup) => (
                <TableRow key={headerGroup.id}>
                  {headerGroup.headers.map((header) => {
                    if (!header?.id) return null;

                    let headerContent: React.ReactNode;
                    try {
                      if (header.isPlaceholder) {
                        headerContent = null;
                      } else {
                        const headerDef = header.column?.columnDef?.header;
                        const context =
                          typeof header.getContext === "function"
                            ? header.getContext()
                            : ({} as Record<string, unknown>);
                        headerContent = safeFlexRender(headerDef, context);
                      }
                    } catch (headerError) {
                      console.error("Header rendering error:", headerError);
                      headerContent = <span>-</span>;
                    }

                    return (
                      <TableHead key={header.id}>{headerContent}</TableHead>
                    );
                  })}
                </TableRow>
              ))}
            </TableHeader>
            <TableBody>
              {isLoading ? (
                Array.from({ length: loadingRowCount }).map((_, index) => (
                  <TableRow key={index}>
                    {columns.map((_, cellIndex) => (
                      <TableCell key={cellIndex}>
                        <div className="bg-muted h-4 animate-pulse rounded" />
                      </TableCell>
                    ))}
                  </TableRow>
                ))
              ) : table.getRowModel().rows?.length && columns.length > 0 ? (
                table.getRowModel().rows.map((row) => (
                  <TableRow
                    key={row.id}
                    data-state={row.getIsSelected() && "selected"}
                  >
                    {row.getVisibleCells().map((cell) => {
                      if (!cell?.id) return null;

                      let cellContent: React.ReactNode;
                      try {
                        const cellDef = cell.column?.columnDef?.cell;
                        const context =
                          typeof cell.getContext === "function"
                            ? cell.getContext()
                            : ({} as Record<string, unknown>);

                        if (!cellDef) {
                          cellContent = <span>-</span>;
                        } else {
                          cellContent = safeFlexRender(cellDef, context);
                        }
                      } catch (cellError) {
                        console.error("Cell rendering error:", cellError);
                        cellContent = <span>-</span>;
                      }

                      return <TableCell key={cell.id}>{cellContent}</TableCell>;
                    })}
                  </TableRow>
                ))
              ) : (
                <TableRow>
                  <TableCell
                    colSpan={safeColumns.length || 1}
                    className="h-24 text-center"
                  >
                    {safeColumns.length === 0 ? "Loading..." : "No results."}
                  </TableCell>
                </TableRow>
              )}
            </TableBody>
          </Table>
        </div>
      </div>
      <div className="flex items-center justify-between space-x-2 py-4">
        <div className="text-muted-foreground flex-1 text-sm">
          {table.getFilteredSelectedRowModel().rows.length} of{" "}
          {table.getFilteredRowModel().rows.length} row(s) selected.
        </div>
        <div className="flex items-center space-x-2">
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
