"use client";

import { useState } from "react";
import { useSession } from "next-auth/react";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "~/components/ui/card";
import { Button } from "~/components/ui/button";
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from "~/components/ui/table";
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
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
} from "~/components/ui/alert-dialog";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { api } from "~/trpc/react";
import { useToast } from "~/hooks/use-toast";
import { ROLES } from "~/lib/permissions/constants";
import { Plus, UserPlus, Crown } from "lucide-react";

interface StudyMembersProps {
  studyId: number;
  role: string;
}

export function StudyMembers({ studyId, role }: StudyMembersProps) {
  const { data: session } = useSession();
  const [isInviteOpen, setIsInviteOpen] = useState(false);
  const [isTransferOpen, setIsTransferOpen] = useState(false);
  const [transferToUserId, setTransferToUserId] = useState<string | null>(null);
  const [email, setEmail] = useState("");
  const [selectedRole, setSelectedRole] = useState(ROLES.RESEARCHER);
  const { toast } = useToast();

  const { data: members, refetch: refetchMembers } = api.study.getMembers.useQuery({ studyId });
  const { data: pendingInvitations, refetch: refetchInvitations } = api.study.getPendingInvitations.useQuery({ studyId });
  
  const { mutate: inviteMember, isPending: isInviting } = api.study.inviteMember.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Member invited successfully",
      });
      setIsInviteOpen(false);
      setEmail("");
      setSelectedRole(ROLES.RESEARCHER);
      refetchMembers();
      refetchInvitations();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const { mutate: transferOwnership, isPending: isTransferring } = api.study.transferOwnership.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Study ownership transferred successfully",
      });
      setIsTransferOpen(false);
      setTransferToUserId(null);
      refetchMembers();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const { mutate: revokeInvitation } = api.study.revokeInvitation.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Invitation revoked successfully",
      });
      refetchInvitations();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const { mutate: updateMemberRole } = api.study.updateMemberRole.useMutation({
    onSuccess: () => {
      toast({
        title: "Success",
        description: "Member role updated successfully",
      });
      refetchMembers();
    },
    onError: (error) => {
      toast({
        title: "Error",
        description: error.message,
        variant: "destructive",
      });
    },
  });

  const canManageMembers = role.toUpperCase() === ROLES.OWNER.toUpperCase() || role.toUpperCase() === ROLES.ADMIN.toUpperCase();
  const isOwner = role.toUpperCase() === ROLES.OWNER.toUpperCase();

  // Get available roles based on current user's role
  const getAvailableRoles = (userRole: string) => {
    const roleHierarchy = {
      [ROLES.OWNER.toUpperCase()]: [ROLES.ADMIN, ROLES.PRINCIPAL_INVESTIGATOR, ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD],
      [ROLES.ADMIN.toUpperCase()]: [ROLES.PRINCIPAL_INVESTIGATOR, ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD],
      [ROLES.PRINCIPAL_INVESTIGATOR.toUpperCase()]: [ROLES.RESEARCHER, ROLES.OBSERVER, ROLES.WIZARD],
    };

    return roleHierarchy[userRole.toUpperCase()] ?? [];
  };

  const availableRoles = getAvailableRoles(role);

  return (
    <div className="space-y-6">
      <Card>
        <CardHeader>
          <div className="flex items-center justify-between">
            <div>
              <CardTitle>Study Members</CardTitle>
              <CardDescription>Manage members and their roles</CardDescription>
            </div>
            {canManageMembers && (
              <Dialog open={isInviteOpen} onOpenChange={setIsInviteOpen}>
                <DialogTrigger asChild>
                  <Button size="sm">
                    <UserPlus className="h-4 w-4 mr-2" />
                    Invite Member
                  </Button>
                </DialogTrigger>
                <DialogContent>
                  <DialogHeader>
                    <DialogTitle>Invite New Member</DialogTitle>
                    <DialogDescription>
                      Enter the email address of the person you want to invite to this study.
                    </DialogDescription>
                  </DialogHeader>
                  <div className="grid gap-4 py-4">
                    <div className="grid gap-2">
                      <Label htmlFor="email">Email address</Label>
                      <Input
                        id="email"
                        type="email"
                        placeholder="Enter email address"
                        value={email}
                        onChange={(e) => setEmail(e.target.value)}
                      />
                    </div>
                    <div className="grid gap-2">
                      <Label htmlFor="role">Role</Label>
                      <Select
                        value={selectedRole}
                        onValueChange={setSelectedRole}
                        defaultValue={availableRoles[0]}
                      >
                        <SelectTrigger>
                          <SelectValue placeholder="Select a role" />
                        </SelectTrigger>
                        <SelectContent>
                          {availableRoles.map((availableRole) => (
                            <SelectItem key={availableRole} value={availableRole}>
                              {availableRole === ROLES.PRINCIPAL_INVESTIGATOR ? "Principal Investigator" : availableRole}
                            </SelectItem>
                          ))}
                        </SelectContent>
                      </Select>
                    </div>
                  </div>
                  <DialogFooter>
                    <Button
                      onClick={() => inviteMember({ studyId, email, role: selectedRole })}
                      disabled={isInviting}
                    >
                      {isInviting ? "Inviting..." : "Send Invite"}
                    </Button>
                  </DialogFooter>
                </DialogContent>
              </Dialog>
            )}
          </div>
        </CardHeader>
        <CardContent>
          {!members || members.length === 0 ? (
            <div className="text-center py-6 text-muted-foreground">
              No members found
            </div>
          ) : (
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Name</TableHead>
                  <TableHead>Email</TableHead>
                  <TableHead>Role</TableHead>
                  {canManageMembers && <TableHead>Actions</TableHead>}
                </TableRow>
              </TableHeader>
              <TableBody>
                {members.map((member) => (
                  <TableRow key={member.userId}>
                    <TableCell>{member.name}</TableCell>
                    <TableCell>{member.email}</TableCell>
                    <TableCell>
                      {canManageMembers && member.role.toUpperCase() !== ROLES.OWNER.toUpperCase() ? (
                        <Select
                          value={member.role}
                          onValueChange={(newRole) =>
                            updateMemberRole({
                              studyId,
                              userId: member.userId,
                              role: newRole,
                            })
                          }
                          disabled={member.userId === session?.user.id}
                        >
                          <SelectTrigger className="w-[140px]">
                            <SelectValue />
                          </SelectTrigger>
                          <SelectContent>
                            <SelectItem value={ROLES.ADMIN}>Admin</SelectItem>
                            <SelectItem value={ROLES.PRINCIPAL_INVESTIGATOR}>Principal Investigator</SelectItem>
                            <SelectItem value={ROLES.RESEARCHER}>Researcher</SelectItem>
                            <SelectItem value={ROLES.OBSERVER}>Observer</SelectItem>
                            <SelectItem value={ROLES.WIZARD}>Wizard</SelectItem>
                          </SelectContent>
                        </Select>
                      ) : (
                        <div className="px-3 py-2">{member.role}</div>
                      )}
                    </TableCell>
                    {canManageMembers && (
                      <TableCell>
                        {isOwner && member.userId !== session?.user.id && (
                          <AlertDialog open={isTransferOpen && transferToUserId === member.userId} onOpenChange={(open) => {
                            setIsTransferOpen(open);
                            if (!open) setTransferToUserId(null);
                          }}>
                            <Button
                              variant="outline"
                              size="sm"
                              onClick={() => {
                                setTransferToUserId(member.userId);
                                setIsTransferOpen(true);
                              }}
                            >
                              <Crown className="h-4 w-4 mr-2" />
                              Transfer Ownership
                            </Button>
                            <AlertDialogContent>
                              <AlertDialogHeader>
                                <AlertDialogTitle>Transfer Study Ownership</AlertDialogTitle>
                                <AlertDialogDescription>
                                  Are you sure you want to transfer ownership to {member.name}? This action cannot be undone.
                                  You will become an admin of the study.
                                </AlertDialogDescription>
                              </AlertDialogHeader>
                              <AlertDialogFooter>
                                <AlertDialogCancel>Cancel</AlertDialogCancel>
                                <AlertDialogAction
                                  onClick={() => {
                                    transferOwnership({
                                      studyId,
                                      newOwnerId: member.userId,
                                    });
                                  }}
                                  disabled={isTransferring}
                                >
                                  {isTransferring ? "Transferring..." : "Transfer Ownership"}
                                </AlertDialogAction>
                              </AlertDialogFooter>
                            </AlertDialogContent>
                          </AlertDialog>
                        )}
                      </TableCell>
                    )}
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          )}
        </CardContent>
      </Card>

      {canManageMembers && (
        <Card>
          <CardHeader>
            <CardTitle>Pending Invitations</CardTitle>
            <CardDescription>Manage outstanding invitations to join the study</CardDescription>
          </CardHeader>
          <CardContent>
            {!pendingInvitations || pendingInvitations.length === 0 ? (
              <div className="text-center py-6 text-muted-foreground">
                No pending invitations
              </div>
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Email</TableHead>
                    <TableHead>Role</TableHead>
                    <TableHead>Invited By</TableHead>
                    <TableHead>Expires</TableHead>
                    <TableHead>Actions</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {pendingInvitations.map((invitation) => (
                    <TableRow key={invitation.id}>
                      <TableCell>{invitation.email}</TableCell>
                      <TableCell>{invitation.role}</TableCell>
                      <TableCell>{invitation.creatorName}</TableCell>
                      <TableCell>{new Date(invitation.expiresAt).toLocaleDateString()}</TableCell>
                      <TableCell>
                        <Button
                          variant="destructive"
                          size="sm"
                          onClick={() => revokeInvitation({ studyId, invitationId: invitation.id })}
                        >
                          Revoke
                        </Button>
                      </TableCell>
                    </TableRow>
                  ))}
                </TableBody>
              </Table>
            )}
          </CardContent>
        </Card>
      )}
    </div>
  );
} 