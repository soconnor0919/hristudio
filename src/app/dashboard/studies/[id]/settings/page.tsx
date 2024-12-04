'use client';

import { useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Button } from "~/components/ui/button";
import { 
  Card, 
  CardHeader, 
  CardTitle, 
  CardDescription, 
  CardContent,
  CardFooter 
} from "~/components/ui/card";
import { InviteUserDialog } from "~/components/invite-user-dialog";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { Badge } from "~/components/ui/badge";
import { format } from "date-fns";

interface Invitation {
  id: number;
  email: string;
  accepted: boolean;
  expiresAt: string;
  createdAt: string;
  roleName: string;
  inviterName: string;
}

interface Study {
  id: number;
  title: string;
  description: string | null;
  createdAt: string;
}

export default function StudySettings() {
  const params = useParams();
  const studyId = parseInt(params.id as string);
  const [study, setStudy] = useState<Study | null>(null);
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    fetchStudyData();
    fetchInvitations();
  }, [studyId]);

  const fetchStudyData = async () => {
    try {
      const response = await fetch(`/api/studies/${studyId}`);
      if (response.ok) {
        const data = await response.json();
        setStudy(data);
      }
    } catch (error) {
      console.error('Error fetching study:', error);
    }
  };

  const fetchInvitations = async () => {
    try {
      const response = await fetch(`/api/invitations?studyId=${studyId}`);
      if (response.ok) {
        const data = await response.json();
        setInvitations(data);
      }
    } catch (error) {
      console.error('Error fetching invitations:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleInviteSent = () => {
    fetchInvitations();
  };

  const handleDeleteInvitation = async (invitationId: number) => {
    try {
      const response = await fetch(`/api/invitations/${invitationId}`, {
        method: 'DELETE',
      });

      if (response.ok) {
        // Update the local state to remove the deleted invitation
        setInvitations(invitations.filter(inv => inv.id !== invitationId));
      } else {
        console.error('Error deleting invitation:', response.statusText);
      }
    } catch (error) {
      console.error('Error deleting invitation:', error);
    }
  };

  if (loading) {
    return <div>Loading...</div>;
  }

  if (!study) {
    return <div>Study not found</div>;
  }

  return (
    <div className="max-w-4xl mx-auto">
      <div className="flex justify-between items-center mb-8">
        <div>
          <h1 className="text-3xl font-bold">{study.title}</h1>
          <p className="text-muted-foreground mt-1">Study Settings</p>
        </div>
      </div>

      <Tabs defaultValue="invites" className="space-y-4">
        <TabsList>
          <TabsTrigger value="invites">Invites</TabsTrigger>
          <TabsTrigger value="settings">Settings</TabsTrigger>
        </TabsList>

        <TabsContent value="invites">
          <Card>
            <CardHeader>
              <div className="flex justify-between items-start">
                <div>
                  <CardTitle>Study Invitations</CardTitle>
                  <CardDescription>
                    Manage invitations to collaborate on this study
                  </CardDescription>
                </div>
                <InviteUserDialog studyId={studyId} onInviteSent={handleInviteSent} />
              </div>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                {invitations.length > 0 ? (
                  invitations.map((invitation) => (
                    <div
                      key={invitation.id}
                      className="flex items-center justify-between p-4 border rounded-lg"
                    >
                      <div className="space-y-1">
                        <div className="font-medium">{invitation.email}</div>
                        <div className="text-sm text-muted-foreground">
                          Role: {invitation.roleName}
                        </div>
                        <div className="text-sm text-muted-foreground">
                          Invited by: {invitation.inviterName} on{" "}
                          {format(new Date(invitation.createdAt), "PPP")}
                        </div>
                      </div>
                      <div className="flex items-center gap-4">
                        <Badge
                          variant={invitation.accepted ? "success" : "secondary"}
                        >
                          {invitation.accepted ? "Accepted" : "Pending"}
                        </Badge>
                        {!invitation.accepted && (
                          <Button
                            variant="ghost"
                            size="sm"
                            className="text-destructive"
                            onClick={() => handleDeleteInvitation(invitation.id)}
                          >
                            Cancel
                          </Button>
                        )}
                      </div>
                    </div>
                  ))
                ) : (
                  <div className="text-center py-8 text-muted-foreground">
                    No invitations sent yet. Use the "Invite User" button to get started.
                  </div>
                )}
              </div>
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="settings">
          <Card>
            <CardHeader>
              <CardTitle>Study Settings</CardTitle>
              <CardDescription>
                Configure general settings for your study
              </CardDescription>
            </CardHeader>
            <CardContent>
              {/* TODO: Add study settings form */}
              <div className="text-center py-8 text-muted-foreground">
                Study settings coming soon...
              </div>
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
} 