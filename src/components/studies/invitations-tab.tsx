'use client';

import { useCallback, useEffect, useState } from "react";
import { Button } from "~/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { useToast } from "~/hooks/use-toast";
import { PERMISSIONS } from "~/lib/permissions-client";
import { InviteUserDialog } from "./invite-user-dialog";

interface Invitation {
  id: string;
  email: string;
  roleName: string;
  accepted: boolean;
  expiresAt: string;
}

interface InvitationsTabProps {
  studyId: number;
  permissions: string[];
}

export function InvitationsTab({ studyId, permissions }: InvitationsTabProps) {
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const { toast } = useToast();

  const hasPermission = (permission: string) => permissions.includes(permission);
  const canManageRoles = hasPermission(PERMISSIONS.MANAGE_ROLES);

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
    } finally {
      setIsLoading(false);
    }
  }, [studyId, toast]);

  useEffect(() => {
    fetchInvitations();
  }, [fetchInvitations]);


  const handleDeleteInvitation = async (invitationId: string) => {
    try {
      const response = await fetch(`/api/invitations/${invitationId}`, {
        method: "DELETE",
      });

      if (!response.ok) {
        throw new Error("Failed to delete invitation");
      }

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

  if (isLoading) {
    return (
      <Card>
        <CardContent className="py-8">
          <p className="text-center text-muted-foreground">Loading invitations...</p>
        </CardContent>
      </Card>
    );
  }

  return (
    <div className="space-y-6">
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Manage Invitations</CardTitle>
              <CardDescription>
                Invite researchers and participants to collaborate on this study
              </CardDescription>
            </div>
            <InviteUserDialog studyId={studyId} onInviteSent={fetchInvitations} />
          </div>
        </CardHeader>
        <CardContent className="space-y-6">
          {invitations.length > 0 ? (
            <div className="space-y-4">
              {invitations.map((invitation) => (
                <div
                  key={invitation.id}
                  className="flex items-center justify-between p-4 border rounded-lg bg-card"
                >
                  <div>
                    <p className="font-medium">{invitation.email}</p>
                    <p className="text-sm text-muted-foreground">
                      Role: {invitation.roleName}
                      {invitation.accepted ? " • Accepted" : " • Pending"}
                    </p>
                  </div>
                  {!invitation.accepted && (
                    <Button
                      variant="outline"
                      size="sm"
                      onClick={() => handleDeleteInvitation(invitation.id)}
                    >
                      Cancel
                    </Button>
                  )}
                </div>
              ))}
            </div>
          ) : (
            <p className="text-muted-foreground">No invitations sent yet.</p>
          )}
        </CardContent>
      </Card>
    </div>
  );
} 