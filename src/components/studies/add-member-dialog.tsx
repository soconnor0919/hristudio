"use client";

import * as React from "react";
import { useSession } from "~/lib/auth-client";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";
import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "~/components/ui/select";
import { Plus, Loader2, Trash2, Shield, UserMinus } from "lucide-react";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";

interface Member {
  id: string;
  userId: string;
  role: string;
  user: {
    name: string | null;
    email: string;
  };
}

interface AddMemberDialogProps {
  studyId: string;
  children?: React.ReactNode;
}

export function AddMemberDialog({ studyId, children }: AddMemberDialogProps) {
  const utils = api.useUtils();
  const [open, setOpen] = React.useState(false);
  const [email, setEmail] = React.useState("");
  const [role, setRole] = React.useState<string>("researcher");

  const { data: membersData } = api.studies.getMembers.useQuery({ studyId });

  const addMember = api.studies.addMember.useMutation({
    onSuccess: () => {
      toast.success("Member added successfully");
      void utils.studies.getMembers.invalidate();
      setEmail("");
      setRole("researcher");
      setOpen(false);
    },
    onError: (error) => {
      toast.error(error.message || "Failed to add member");
    },
  });

  const removeMember = api.studies.removeMember.useMutation({
    onSuccess: () => {
      toast.success("Member removed");
      void utils.studies.getMembers.invalidate();
    },
    onError: (error) => {
      toast.error(error.message || "Failed to remove member");
    },
  });

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (!email || !role) return;
    addMember.mutate({ studyId, email, role: role as "researcher" | "wizard" | "observer" });
  };

  const handleRemove = (memberId: string, memberName: string | null) => {
    if (confirm(`Remove ${memberName ?? memberId} from this study?`)) {
      removeMember.mutate({ studyId, memberId });
    }
  };

  const members = membersData ?? [];
  const currentUser = members.find((m) => m.userId);
  const isOwner = currentUser?.role === "owner";

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        {children ?? (
          <Button size="sm">
            <Plus className="mr-2 h-4 w-4" />
            Add Member
          </Button>
        )}
      </DialogTrigger>
      <DialogContent className="max-w-md">
        <DialogHeader>
          <DialogTitle>Manage Team Members</DialogTitle>
          <DialogDescription>
            Add researchers, wizards, or observers to collaborate on this study.
          </DialogDescription>
        </DialogHeader>

        {/* Current Members */}
        <div className="max-h-[200px] space-y-2 overflow-y-auto">
          {members.map((member) => (
            <div
              key={member.id}
              className="flex items-center justify-between rounded-lg border p-3"
            >
              <div className="flex items-center gap-3">
                <div className="flex h-8 w-8 items-center justify-center rounded-full bg-primary/10">
                  <span className="text-sm font-medium text-primary">
                    {(member.user.name ?? member.user.email).charAt(0).toUpperCase()}
                  </span>
                </div>
                <div>
                  <p className="text-sm font-medium">
                    {member.user.name ?? member.user.email}
                    {member.role === "owner" && (
                      <Shield className="ml-2 inline h-3 w-3 text-amber-500" />
                    )}
                  </p>
                  <p className="text-muted-foreground text-xs capitalize">{member.role}</p>
                </div>
              </div>
              {isOwner && member.role !== "owner" && (
                <Button
                  variant="ghost"
                  size="sm"
                  onClick={() => handleRemove(member.id, member.user.name)}
                  disabled={removeMember.isPending}
                >
                  <UserMinus className="h-4 w-4 text-destructive" />
                </Button>
              )}
            </div>
          ))}
        </div>

        <form onSubmit={handleSubmit} className="space-y-4">
          <div className="space-y-2">
            <Label htmlFor="email">Email Address</Label>
            <Input
              id="email"
              type="email"
              placeholder="researcher@university.edu"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>

          <div className="space-y-2">
            <Label htmlFor="role">Role</Label>
            <Select value={role} onValueChange={setRole}>
              <SelectTrigger>
                <SelectValue placeholder="Select a role" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="researcher">Researcher</SelectItem>
                <SelectItem value="wizard">Wizard</SelectItem>
                <SelectItem value="observer">Observer</SelectItem>
              </SelectContent>
            </Select>
            <p className="text-muted-foreground text-xs">
              Researchers can design experiments, Wizards execute trials, Observers have read-only access.
            </p>
          </div>

          <DialogFooter>
            <Button type="button" variant="outline" onClick={() => setOpen(false)}>
              Cancel
            </Button>
            <Button type="submit" disabled={addMember.isPending}>
              {addMember.isPending && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
              Add Member
            </Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
}
