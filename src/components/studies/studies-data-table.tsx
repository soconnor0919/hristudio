"use client";

import React from "react";
import { Plus } from "lucide-react";
import { DataTable } from "~/components/ui/data-table";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { PageHeader, ActionButton } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { useStudyManagement } from "~/hooks/useStudyManagement";
import { studiesColumns, type Study } from "./studies-columns";
import { FlaskConical } from "lucide-react";

export function StudiesDataTable() {
  const { userStudies, isLoadingUserStudies, refreshStudyData } =
    useStudyManagement();

  // Auto-refresh studies when component mounts to catch external changes
  React.useEffect(() => {
    const interval = setInterval(() => {
      void refreshStudyData();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [refreshStudyData]);

  // Set breadcrumbs
  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies" },
  ]);

  // Transform userStudies to match the Study type expected by columns
  const studies: Study[] = React.useMemo(() => {
    if (!userStudies) return [];

    return userStudies.map((study) => ({
      id: study.id,
      name: study.name,
      description: study.description,
      status: study.status,
      createdAt: study.createdAt,
      updatedAt: study.updatedAt,
      institution: study.institution,
      irbProtocolNumber: study.irbProtocol ?? undefined,
      owner: {
        name: study.members?.find((m) => m.role === "owner")?.user.name ?? null,
        email: study.members?.find((m) => m.role === "owner")?.user.email ?? "",
      },
      _count: {
        studyMembers: study.members?.length ?? 0,
      },
      userRole: study.members?.find((m) => m.user.id === study.createdBy)?.role,
      isOwner: study.members?.some((m) => m.role === "owner") ?? false,
    }));
  }, [userStudies]);

  // Status filter options
  const statusOptions = [
    { label: "All Statuses", value: "all" },
    { label: "Draft", value: "draft" },
    { label: "Active", value: "active" },
    { label: "Completed", value: "completed" },
    { label: "Archived", value: "archived" },
  ];

  // Role filter options
  const roleOptions = [
    { label: "All Roles", value: "all" },
    { label: "Owner", value: "owner" },
    { label: "Researcher", value: "researcher" },
    { label: "Wizard", value: "wizard" },
    { label: "Observer", value: "observer" },
  ];

  const [statusFilter, setStatusFilter] = React.useState("all");
  const [roleFilter, setRoleFilter] = React.useState("all");

  // Filter studies based on selected filters
  const filteredStudies = React.useMemo(() => {
    return studies.filter((study) => {
      const statusMatch =
        statusFilter === "all" || study.status === statusFilter;
      const roleMatch = roleFilter === "all" || study.userRole === roleFilter;
      return statusMatch && roleMatch;
    });
  }, [studies, statusFilter, roleFilter]);

  const filters = (
    <div className="flex items-center space-x-2">
      <Select value={statusFilter} onValueChange={setStatusFilter}>
        <SelectTrigger className="w-[140px]">
          <SelectValue placeholder="Status" />
        </SelectTrigger>
        <SelectContent>
          {statusOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>

      <Select value={roleFilter} onValueChange={setRoleFilter}>
        <SelectTrigger className="w-[140px]">
          <SelectValue placeholder="Role" />
        </SelectTrigger>
        <SelectContent>
          {roleOptions.map((option) => (
            <SelectItem key={option.value} value={option.value}>
              {option.label}
            </SelectItem>
          ))}
        </SelectContent>
      </Select>
    </div>
  );

  return (
    <div className="space-y-6">
      <PageHeader
        title="Studies"
        description="Manage your Human-Robot Interaction research studies"
        icon={FlaskConical}
        actions={
          <ActionButton href="/studies/new">
            <Plus className="mr-2 h-4 w-4" />
            New Study
          </ActionButton>
        }
      />

      <div className="space-y-4">
        <DataTable
          columns={studiesColumns}
          data={filteredStudies}
          searchKey="name"
          searchPlaceholder="Search studies..."
          isLoading={isLoadingUserStudies}
          loadingRowCount={5}
          filters={filters}
        />
      </div>
    </div>
  );
}
