'use client';

import { useState, useEffect, useCallback } from "react";
import { UserAvatar } from "~/components/user-avatar";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { useToast } from "~/hooks/use-toast";
import { PERMISSIONS } from "~/lib/permissions-client";
import { InviteUserDialog } from "./invite-user-dialog";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
import { Trash2Icon } from "lucide-react";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "~/components/ui/alert-dialog";

interface User {
  id: string;
  email: string;
  name: string | null;
  roles: Array<{ id: number; name: string }>;
  imageUrl: string;
}

interface Invitation {
  id: string;
  email: string;
  roleName: string;
  accepted: boolean;
  expiresAt: string;
}

interface Role {
  id: number;
  name: string;
  description: string;
}

interface UsersTabProps {
  studyId: number;
  permissions: string[];
}

export function UsersTab({ studyId, permissions }: UsersTabProps) {
  const [users, setUsers] = useState<User[]>([]);
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [roles, setRoles] = useState<Role[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const { toast } = useToast();

  const hasPermission = (permission: string) => permissions.includes(permission);
  const canManageRoles = hasPermission(PERMISSIONS.MANAGE_ROLES);

  const fetchUsers = useCallback(async () => {
    try {
      const response = await fetch(`/api/studies/${studyId}/users`);
      if (!response.ok) throw new Error("Failed to fetch users");
      const data = await response.json();
      setUsers(data.data || []);
    } catch (error) {
      console.error("Error fetching users:", error);
      toast({
        title: "Error",
        description: "Failed to load users",
        variant: "destructive",
      });
    }
  }, [studyId, toast]);

  const fetchInvitations = useCallback(async () => {
    try {
      const response = await fetch(`/api/invitations?studyId=${studyId}`);
      if (!response.ok) throw new Error("Failed to fetch invitations");
      const data = await response.json();
      setInvitations(data.data || []);
    } catch (error) {
      console.error("Error fetching invitations:", error);
      toast({
        title: "Error",
        description: "Failed to load invitations",
        variant: "destructive",
      });
    }
  }, [studyId, toast]);

  const fetchRoles = useCallback(async () => {
    try {
      const response = await fetch("/api/roles");
      if (!response.ok) throw new Error("Failed to fetch roles");
      const data = await response.json();
      setRoles(data.filter((role: Role) => 
        !['admin'].includes(role.name)
      ));
    } catch (error) {
      console.error("Error fetching roles:", error);
      toast({
        title: "Error",
        description: "Failed to load roles",
        variant: "destructive",
      });
    }
  }, [toast]);

  const fetchData = useCallback(async () => {
    setIsLoading(true);
    try {
      await Promise.all([
        fetchUsers(),
        fetchInvitations(),
        fetchRoles(),
      ]);
    } finally {
      setIsLoading(false);
    }
  }, [fetchUsers, fetchInvitations, fetchRoles]);

  useEffect(() => {
    fetchData();
  }, [fetchData]);

  const handleRoleChange = async (userId: string, newRoleId: string) => {
    try {
      const response = await fetch(`/api/studies/${studyId}/users/${userId}/role`, {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          roleId: parseInt(newRoleId, 10),
        }),
      });

      if (!response.ok) throw new Error("Failed to update role");

      toast({
        title: "Success",
        description: "User role updated successfully",
      });

      // Refresh users list
      fetchUsers();
    } catch (error) {
      console.error("Error updating role:", error);
      toast({
        title: "Error",
        description: "Failed to update user role",
        variant: "destructive",
      });
    }
  };

  const handleDeleteInvitation = async (invitationId: string) => {
    try {
      const response = await fetch(`/api/invitations/${invitationId}`, {
        method: "DELETE",
      });

      if (!response.ok) throw new Error("Failed to delete invitation");

      setInvitations(invitations.filter(inv => inv.id !== invitationId));
      toast({
        title: "Success",
        description: "Invitation deleted successfully",
      });
    } catch (error) {
      console.error("Error deleting invitation:", error);
      toast({
        title: "Error",
        description: "Failed to delete invitation",
        variant: "destructive",
      });
    }
  };

  const formatName = (user: User) => {
    return user.name || user.email;
  };

  const formatRoleName = (name: string) => {
    return name
      .split('_')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
      .join(' ');
  };

  if (isLoading) {
    return <div>Loading...</div>;
  }

  const pendingInvitations = invitations.filter(inv => !inv.accepted);

  return (
    <div className="space-y-6">
      <Card>
        <CardHeader className="flex flex-col justify-between">
          <div className="flex justify-between items-center">
            <div>
              <CardTitle>Study Members</CardTitle>
              <CardDescription>
                Manage users and their roles in this study
              </CardDescription>
            </div>
            {canManageRoles && <InviteUserDialog studyId={studyId} onInviteSent={fetchInvitations} />}
          </div>
        </CardHeader>
        <CardContent>
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>User</TableHead>
                <TableHead>Email</TableHead>
                <TableHead>Role</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {users.map((user) => (
                <TableRow key={user.id}>
                  <TableCell className="flex items-center gap-2">
                    <UserAvatar
                      user={{
                        name: formatName(user),
                        email: user.email,
                        imageUrl: user.imageUrl,
                      }}
                    />
                    <span>{formatName(user)}</span>
                  </TableCell>
                  <TableCell>{user.email}</TableCell>
                  <TableCell>
                    {canManageRoles ? (
                      <Select
                        value={user.roles[0]?.id.toString()}
                        onValueChange={(value) => handleRoleChange(user.id, value)}
                      >
                        <SelectTrigger className="w-[180px]">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent>
                          {roles.map((role) => (
                            <SelectItem key={role.id} value={role.id.toString()}>
                              {formatRoleName(role.name)}
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    ) : (
                      <span>{formatRoleName(user.roles[0]?.name || '')}</span>
                    )}
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </CardContent>
      </Card>

      {pendingInvitations.length > 0 && (
        <Card>
          <CardHeader>
            <CardTitle>Pending Invitations</CardTitle>
            <CardDescription>
              Outstanding invitations to join the study
            </CardDescription>
          </CardHeader>
          <CardContent>
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Email</TableHead>
                  <TableHead>Role</TableHead>
                  <TableHead>Expires</TableHead>
                  {canManageRoles && <TableHead className="w-[100px]">Actions</TableHead>}
                </TableRow>
              </TableHeader>
              <TableBody>
                {pendingInvitations.map((invitation) => (
                  <TableRow key={invitation.id}>
                    <TableCell>{invitation.email}</TableCell>
                    <TableCell>{formatRoleName(invitation.roleName)}</TableCell>
                    <TableCell>{new Date(invitation.expiresAt).toLocaleDateString()}</TableCell>
                    {canManageRoles && (
                      <TableCell>
                        <AlertDialog>
                          <AlertDialogTrigger asChild>
                            <Button variant="ghost" size="icon">
                              <Trash2Icon className="h-4 w-4" />
                            </Button>
                          </AlertDialogTrigger>
                          <AlertDialogContent>
                            <AlertDialogHeader>
                              <AlertDialogTitle>Delete Invitation</AlertDialogTitle>
                              <AlertDialogDescription>
                                Are you sure you want to delete this invitation? This action cannot be undone.
                              </AlertDialogDescription>
                            </AlertDialogHeader>
                            <AlertDialogFooter>
                              <AlertDialogCancel>Cancel</AlertDialogCancel>
                              <AlertDialogAction
                                onClick={() => handleDeleteInvitation(invitation.id)}
                              >
                                Delete
                              </AlertDialogAction>
                            </AlertDialogFooter>
                          </AlertDialogContent>
                        </AlertDialog>
                      </TableCell>
                    )}
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </CardContent>
        </Card>
      )}
    </div>
  );
} 