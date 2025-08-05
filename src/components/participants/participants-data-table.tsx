"use client";

import { Plus, Users } from "lucide-react";
import React from "react";

import { Button } from "~/components/ui/button";
import { DataTable } from "~/components/ui/data-table";

import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { ActionButton, PageHeader } from "~/components/ui/page-header";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { api } from "~/trpc/react";
import { participantsColumns, type Participant } from "./participants-columns";

export function ParticipantsDataTable() {
  const [consentFilter, setConsentFilter] = React.useState("all");

  const {
    data: participantsData,
    isLoading,
    error,
    refetch,
  } = api.participants.getUserParticipants.useQuery(
    {
      page: 1,
      limit: 50,
    },
    {
      refetchOnWindowFocus: false,
    },
  );

  // Auto-refresh participants when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refetch();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refetch]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Participants" },
  ]);

  // Transform participants data to match the Participant type expected by columns
  const participants: Participant[] = React.useMemo(() => {
    if (!participantsData?.participants) return [];

    return participantsData.participants.map((p) => ({
      id: p.id,
      participantCode: p.participantCode,
      email: p.email,
      name: p.name,
      consentGiven: (p as any).hasConsent || false,
      consentDate: (p as any).latestConsent?.signedAt
        ? new Date((p as any).latestConsent.signedAt as unknown as string)
        : null,
      createdAt: p.createdAt,
      trialCount: (p as any).trialCount || 0,
      userRole: undefined,
      canEdit: true,
      canDelete: true,
    }));
  }, [participantsData]);

  // Consent filter options
  const consentOptions = [
    { label: "All Participants", value: "all" },
    { label: "Consented", value: "consented" },
    { label: "Pending Consent", value: "pending" },
  ];

  // Filter participants based on selected filters
  const filteredParticipants = React.useMemo(() => {
    return participants.filter((participant) => {
      if (consentFilter === "all") return true;
      if (consentFilter === "consented") return participant.consentGiven;
      if (consentFilter === "pending") return !participant.consentGiven;
      return true;
    });
  }, [participants, consentFilter]);

  const filters = (
    <div className="flex items-center space-x-2">
      <Select value={consentFilter} onValueChange={setConsentFilter}>
        <SelectTrigger className="w-[160px]">
          <SelectValue placeholder="Consent Status" />
        </SelectTrigger>
        <SelectContent>
          {consentOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );

  // Show error state
  if (error) {
    return (
      <div className="space-y-6">
        <PageHeader
          title="Participants"
          description="Manage participant registration, consent, and trial assignments"
          icon={Users}
          actions={
            <ActionButton href="/participants/new">
              <Plus className="mr-2 h-4 w-4" />
              Add Participant
            </ActionButton>
          }
        />
        <div className="rounded-lg border border-red-200 bg-red-50 p-6 text-center">
          <div className="text-red-800">
            <h3 className="mb-2 text-lg font-semibold">
              Failed to Load Participants
            </h3>
            <p className="mb-4">
              {error.message || "An error occurred while loading participants."}
            </p>
            <Button onClick={() => refetch()} variant="outline">
              Try Again
            </Button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      <PageHeader
        title="Participants"
        description="Manage participant registration, consent, and trial assignments"
        icon={Users}
        actions={
          <ActionButton href="/participants/new">
            <Plus className="mr-2 h-4 w-4" />
            Add Participant
          </ActionButton>
        }
      />

      <div className="space-y-4">
        {/* Data Table */}
        <DataTable
          columns={participantsColumns}
          data={filteredParticipants}
          searchKey="name"
          searchPlaceholder="Search participants..."
          isLoading={isLoading}
          loadingRowCount={5}
          filters={filters}
        />
      </div>
    </div>
  );
}
