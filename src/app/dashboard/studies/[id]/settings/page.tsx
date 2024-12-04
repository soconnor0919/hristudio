'use client';

import { useEffect, useState } from "react";
import { useParams } from "next/navigation";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "~/components/ui/tabs";
import { InviteUserDialog } from "~/components/invite-user-dialog";
import { Button } from "~/components/ui/button";
import { Loader2 } from "lucide-react";
import { useToast } from "~/hooks/use-toast";

interface Study {
  id: number;
  title: string;
  description: string;
}

interface Invitation {
  id: string;
  email: string;
  roleName: string;
  accepted: boolean;
  expiresAt: string;
}

export default function StudySettingsPage() {
  const params = useParams();
  const [study, setStudy] = useState<Study | null>(null);
  const [invitations, setInvitations] = useState<Invitation[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { toast } = useToast();

  const fetchStudyData = async () => {
    try {
      const response = await fetch(`/api/studies/${params.id}`);
      if (!response.ok) throw new Error("Failed to fetch study");
      const data = await response.json();
      setStudy(data);
    } catch (error) {
      setError("Failed to load study details");
      console.error("Error fetching study:", error);
      toast({
        title: "Error",
        description: "Failed to load study details",
        variant: "destructive",
      });
    }
  };

  const fetchInvitations = async () => {
    try {
      const response = await fetch(`/api/invitations?studyId=${params.id}`);
      if (!response.ok) throw new Error("Failed to fetch invitations");
      const data = await response.json();
      setInvitations(data);
    } catch (error) {
      setError("Failed to load invitations");
      console.error("Error fetching invitations:", error);
      toast({
        title: "Error",
        description: "Failed to load invitations",
        variant: "destructive",
      });
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    const loadData = async () => {
      await Promise.all([fetchStudyData(), fetchInvitations()]);
    };
    loadData();
  }, [params.id]); // eslint-disable-line react-hooks/exhaustive-deps

  const handleDeleteInvitation = async (invitationId: string) => {
    try {
      const response = await fetch(`/api/invitations/${invitationId}`, {
        method: "DELETE",
      });

      if (!response.ok) {
        throw new Error("Failed to delete invitation");
      }

      // Update local state
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
      <div className="flex items-center justify-center min-h-[400px]">
        <Loader2 className="h-8 w-8 animate-spin" />
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <p className="text-red-500">{error}</p>
      </div>
    );
  }

  if (!study) {
    return (
      <div className="flex items-center justify-center min-h-[400px]">
        <p className="text-gray-500">Study not found</p>
      </div>
    );
  }

  return (
    <div className="container py-6 space-y-6">
      <div>
        <h1 className="text-2xl font-bold">{study.title}</h1>
        <p className="text-muted-foreground">{study.description}</p>
      </div>

      <Tabs defaultValue="invites" className="space-y-4">
        <TabsList>
          <TabsTrigger value="invites">Invites</TabsTrigger>
          <TabsTrigger value="settings">Settings</TabsTrigger>
        </TabsList>
        
        <TabsContent value="invites">
          <Card>
            <CardHeader>
              <CardTitle>Manage Invitations</CardTitle>
              <CardDescription>
                Invite researchers and participants to collaborate on &ldquo;{study.title}&rdquo;
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-6">
              <div>
                <InviteUserDialog studyId={study.id} onInviteSent={fetchInvitations} />
              </div>

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
        </TabsContent>
        
        <TabsContent value="settings">
          <Card>
            <CardHeader>
              <CardTitle>Study Settings</CardTitle>
              <CardDescription>
                Configure study settings and permissions
              </CardDescription>
            </CardHeader>
            <CardContent>
              <p className="text-muted-foreground">Settings coming soon...</p>
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  );
} 