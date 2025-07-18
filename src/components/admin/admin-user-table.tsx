"use client";

import { useState } from "react";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { api } from "~/trpc/react";
import { formatRole, getAvailableRoles } from "~/lib/auth-client";
import type { SystemRole } from "~/lib/auth-client";

interface UserWithRoles {
  id: string;
  name: string | null;
  email: string;
  image: string | null;
  createdAt: Date;
  roles: SystemRole[];
}

export function AdminUserTable() {
  const [search, setSearch] = useState("");
  const [selectedRole, setSelectedRole] = useState<SystemRole | "">("");
  const [page, setPage] = useState(1);
  const [selectedUser, setSelectedUser] = useState<UserWithRoles | null>(null);
  const [roleToAssign, setRoleToAssign] = useState<SystemRole | "">("");

  const {
    data: usersData,
    isLoading,
    refetch,
  } = api.users.list.useQuery({
    page,
    limit: 10,
    search: search || undefined,
    role: selectedRole || undefined,
  });

  const assignRole = api.users.assignRole.useMutation({
    onSuccess: () => {
      void refetch();
      setSelectedUser(null);
      setRoleToAssign("");
    },
  });

  const removeRole = api.users.removeRole.useMutation({
    onSuccess: () => {
      void refetch();
    },
  });

  const handleAssignRole = () => {
    if (!selectedUser || !roleToAssign) return;

    assignRole.mutate({
      userId: selectedUser.id,
      role: roleToAssign,
    });
  };

  const handleRemoveRole = (userId: string, role: SystemRole) => {
    removeRole.mutate({
      userId,
      role,
    });
  };

  const availableRoles = getAvailableRoles();

  if (isLoading) {
    return (
      <div className="flex items-center justify-center py-8">
        <div className="h-8 w-8 animate-spin rounded-full border-b-2 border-blue-600"></div>
      </div>
    );
  }

  return (
    <div className="space-y-4">
      {/* Filters */}
      <div className="flex flex-col gap-4 sm:flex-row sm:items-end">
        <div className="flex-1">
          <Label htmlFor="search">Search Users</Label>
          <Input
            id="search"
            placeholder="Search by name or email..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
          />
        </div>
        <div className="w-full sm:w-48">
          <Label htmlFor="role-filter">Filter by Role</Label>
          <Select
            value={selectedRole}
            onValueChange={(value) => setSelectedRole(value as SystemRole | "")}
          >
            <SelectTrigger>
              <SelectValue placeholder="All roles" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="">All roles</SelectItem>
              {availableRoles.map((role) => (
                <SelectItem key={role.value} value={role.value}>
                  {role.label}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* Users Table */}
      <div className="rounded-md border">
        <div className="overflow-x-auto">
          <table className="w-full">
            <thead>
              <tr className="border-b bg-slate-50">
                <th className="px-4 py-3 text-left text-sm font-medium text-slate-700">
                  User
                </th>
                <th className="px-4 py-3 text-left text-sm font-medium text-slate-700">
                  Email
                </th>
                <th className="px-4 py-3 text-left text-sm font-medium text-slate-700">
                  Roles
                </th>
                <th className="px-4 py-3 text-left text-sm font-medium text-slate-700">
                  Created
                </th>
                <th className="px-4 py-3 text-left text-sm font-medium text-slate-700">
                  Actions
                </th>
              </tr>
            </thead>
            <tbody className="divide-y divide-slate-200">
              {usersData?.users.map((user) => (
                <tr key={user.id} className="hover:bg-slate-50">
                  <td className="px-4 py-3">
                    <div className="flex items-center gap-3">
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                        <span className="text-sm font-semibold text-blue-600">
                          {(user.name ?? user.email).charAt(0).toUpperCase()}
                        </span>
                      </div>
                      <div>
                        <p className="font-medium text-slate-900">
                          {user.name ?? "Unnamed User"}
                        </p>
                        <p className="text-sm text-slate-500">
                          ID: {user.id.slice(0, 8)}...
                        </p>
                      </div>
                    </div>
                  </td>
                  <td className="px-4 py-3">
                    <p className="text-sm text-slate-900">{user.email}</p>
                  </td>
                  <td className="px-4 py-3">
                    <div className="flex flex-wrap gap-1">
                      {user.roles.length > 0 ? (
                        user.roles.map((role) => (
                          <div key={role} className="flex items-center gap-1">
                            <Badge variant="secondary" className="text-xs">
                              {formatRole(role)}
                            </Badge>
                            <Button
                              variant="ghost"
                              size="sm"
                              className="h-4 w-4 p-0 text-red-500 hover:text-red-700"
                              onClick={() => handleRemoveRole(user.id, role)}
                              disabled={removeRole.isPending}
                            >
                              ×
                            </Button>
                          </div>
                        ))
                      ) : (
                        <span className="text-xs text-slate-500">No roles</span>
                      )}
                    </div>
                  </td>
                  <td className="px-4 py-3">
                    <p className="text-sm text-slate-600">
                      {new Date(user.createdAt).toLocaleDateString()}
                    </p>
                  </td>
                  <td className="px-4 py-3">
                    <Dialog>
                      <DialogTrigger asChild>
                        <Button
                          variant="outline"
                          size="sm"
                          onClick={() => setSelectedUser(user)}
                        >
                          Manage
                        </Button>
                      </DialogTrigger>
                      <DialogContent>
                        <DialogHeader>
                          <DialogTitle>Manage User Roles</DialogTitle>
                          <DialogDescription>
                            Assign or remove roles for {user.name ?? user.email}
                          </DialogDescription>
                        </DialogHeader>
                        <div className="space-y-4">
                          <div>
                            <Label htmlFor="role-select">Assign Role</Label>
                            <div className="flex gap-2">
                              <Select
                                value={roleToAssign}
                                onValueChange={(value) =>
                                  setRoleToAssign(value as SystemRole)
                                }
                              >
                                <SelectTrigger>
                                  <SelectValue placeholder="Select a role" />
                                </SelectTrigger>
                                <SelectContent>
                                  {availableRoles
                                    .filter(
                                      (role) =>
                                        !user.roles.includes(role.value),
                                    )
                                    .map((role) => (
                                      <SelectItem
                                        key={role.value}
                                        value={role.value}
                                      >
                                        {role.label}
                                      </SelectItem>
                                    ))}
                                </SelectContent>
                              </Select>
                              <Button
                                onClick={handleAssignRole}
                                disabled={!roleToAssign || assignRole.isPending}
                              >
                                {assignRole.isPending
                                  ? "Assigning..."
                                  : "Assign"}
                              </Button>
                            </div>
                          </div>

                          <div>
                            <Label>Current Roles</Label>
                            <div className="mt-2 space-y-2">
                              {user.roles.length > 0 ? (
                                user.roles.map((role) => (
                                  <div
                                    key={role}
                                    className="flex items-center justify-between rounded-md border p-2"
                                  >
                                    <div>
                                      <Badge variant="secondary">
                                        {formatRole(role)}
                                      </Badge>
                                      <p className="mt-1 text-xs text-slate-600">
                                        {
                                          availableRoles.find(
                                            (r) => r.value === role,
                                          )?.description
                                        }
                                      </p>
                                    </div>
                                    <Button
                                      variant="outline"
                                      size="sm"
                                      onClick={() =>
                                        handleRemoveRole(user.id, role)
                                      }
                                      disabled={removeRole.isPending}
                                    >
                                      Remove
                                    </Button>
                                  </div>
                                ))
                              ) : (
                                <p className="text-sm text-slate-500">
                                  No roles assigned
                                </p>
                              )}
                            </div>
                          </div>
                        </div>
                      </DialogContent>
                    </Dialog>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>

      {/* Pagination */}
      {usersData && usersData.pagination.pages > 1 && (
        <div className="flex items-center justify-between">
          <p className="text-sm text-slate-600">
            Showing {usersData.users.length} of {usersData.pagination.total}{" "}
            users
          </p>
          <div className="flex gap-2">
            <Button
              variant="outline"
              size="sm"
              onClick={() => setPage(page - 1)}
              disabled={page === 1}
            >
              Previous
            </Button>
            <span className="flex items-center px-3 text-sm">
              {page} of {usersData.pagination.pages}
            </span>
            <Button
              variant="outline"
              size="sm"
              onClick={() => setPage(page + 1)}
              disabled={page === usersData.pagination.pages}
            >
              Next
            </Button>
          </div>
        </div>
      )}

      {/* Error Messages */}
      {assignRole.error && (
        <div className="rounded-md bg-red-50 p-3 text-sm text-red-700">
          <p className="font-medium">Error assigning role</p>
          <p>{assignRole.error.message}</p>
        </div>
      )}

      {removeRole.error && (
        <div className="rounded-md bg-red-50 p-3 text-sm text-red-700">
          <p className="font-medium">Error removing role</p>
          <p>{removeRole.error.message}</p>
        </div>
      )}
    </div>
  );
}
