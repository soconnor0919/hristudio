"use client";

import * as React from "react";
import { DataTable } from "~/components/ui/data-table";
import { type TrialEvent, eventsColumns } from "./events-columns";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "~/components/ui/select";
import { Input } from "~/components/ui/input";

interface EventsDataTableProps {
    data: TrialEvent[];
    startTime?: Date;
}

export function EventsDataTable({ data, startTime }: EventsDataTableProps) {
    const [eventTypeFilter, setEventTypeFilter] = React.useState<string>("all");
    const [globalFilter, setGlobalFilter] = React.useState<string>("");

    const columns = React.useMemo(() => eventsColumns(startTime), [startTime]);

    // Enhanced filtering logic
    const filteredData = React.useMemo(() => {
        return data.filter(event => {
            // Type filter
            if (eventTypeFilter !== "all" && !event.eventType.includes(eventTypeFilter)) {
                return false;
            }

            // Global text search (checks type and data)
            if (globalFilter) {
                const searchLower = globalFilter.toLowerCase();
                const typeMatch = event.eventType.toLowerCase().includes(searchLower);
                // Safe JSON stringify check
                const dataString = event.data ? JSON.stringify(event.data).toLowerCase() : "";
                const dataMatch = dataString.includes(searchLower);

                return typeMatch || dataMatch;
            }

            return true;
        });
    }, [data, eventTypeFilter, globalFilter]);

    // Custom Filters UI
    const filters = (
        <div className="flex items-center gap-2">
            <Select value={eventTypeFilter} onValueChange={setEventTypeFilter}>
                <SelectTrigger className="h-8 w-[160px]">
                    <SelectValue placeholder="All Events" />
                </SelectTrigger>
                <SelectContent>
                    <SelectItem value="all">All Events</SelectItem>
                    <SelectItem value="intervention">Interventions</SelectItem>
                    <SelectItem value="robot">Robot Actions</SelectItem>
                    <SelectItem value="step">Step Changes</SelectItem>
                    <SelectItem value="error">Errors</SelectItem>
                </SelectContent>
            </Select>
        </div>
    );

    return (
        <div className="space-y-4">
            {/* We instruct DataTable to use our filtered data, but DataTable also has internal filtering.
                 Since we implemented custom external filtering for "type" dropdown and "global" search,
                 we pass the filtered data directly. 
                 
                 However, the shared DataTable component has a `searchKey` prop that drives an internal Input.
                 If we want to use OUR custom search input (to search JSON data), we should probably NOT use 
                 DataTable's internal search or pass a custom filter.
                 
                 The shared DataTable's `searchKey` only filters a specific column string value. 
                 Since "data" is an object, we can't easily use the built-in single-column search.
                 So we'll implement our own search input and pass `filters={filters}` which creates 
                 additional dropdowns, but we might want to REPLACE the standard search input.
                 
                 Looking at `DataTable` implementation:
                 It renders `<Input ... />` if `searchKey` is provided. If we don't provide `searchKey`, 
                 no input is rendered, and we can put ours in `filters`.
             */}

            <div className="flex items-center justify-between">
                <div className="flex flex-1 items-center space-x-2">
                    <Input
                        placeholder="Search event data..."
                        value={globalFilter}
                        onChange={(e) => setGlobalFilter(e.target.value)}
                        className="h-8 w-[150px] lg:w-[250px]"
                    />
                    {filters}
                </div>
            </div>

            <DataTable
                columns={columns}
                data={filteredData}
                // No searchKey, we handle it externally
                isLoading={false}
            />
        </div>
    );
}
