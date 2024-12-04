import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
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
import { ROLES } from "~/lib/roles";
import { UserPlusIcon } from "lucide-react";

interface Role {
  id: number;
  name: string;
}

interface InviteUserDialogProps {
  studyId: number;
  onInviteSent?: () => void;
}

export function InviteUserDialog({ studyId, onInviteSent }: InviteUserDialogProps) {
  const [open, setOpen] = useState(false);
  const [email, setEmail] = useState("");
  const [roleId, setRoleId] = useState("");
  const [loading, setLoading] = useState(false);
  const [roles, setRoles] = useState<Role[]>([]);
  const router = useRouter();

  useEffect(() => {
    // Fetch available roles when dialog opens
    if (open) {
      fetchRoles();
    }
  }, [open]);

  const fetchRoles = async () => {
    try {
      const response = await fetch("/api/roles");
      if (response.ok) {
        const data = await response.json();
        setRoles(data.filter((role: Role) => 
          role.name !== ROLES.ADMIN && role.name !== ROLES.PRINCIPAL_INVESTIGATOR
        ));
      }
    } catch (error) {
      console.error("Error fetching roles:", error);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch("/api/invitations", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          email,
          studyId,
          roleId: parseInt(roleId),
        }),
      });

      if (!response.ok) {
        throw new Error("Failed to send invitation");
      }

      setOpen(false);
      setEmail("");
      setRoleId("");
      onInviteSent?.();
      router.refresh();
    } catch (error) {
      console.error("Error sending invitation:", error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button>
          <UserPlusIcon className="w-4 h-4 mr-2" />
          Invite User
        </Button>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Invite User</DialogTitle>
          <DialogDescription>
            Send an invitation to join your study. The user will receive an email with instructions.
          </DialogDescription>
        </DialogHeader>
        <form onSubmit={handleSubmit}>
          <div className="space-y-4 py-4">
            <div className="space-y-2">
              <Label htmlFor="email">Email Address</Label>
              <Input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="researcher@university.edu"
                required
              />
            </div>
            <div className="space-y-2">
              <Label htmlFor="role">Role</Label>
              <Select
                value={roleId}
                onValueChange={setRoleId}
                required
              >
                <SelectTrigger>
                  <SelectValue placeholder="Select a role" />
                </SelectTrigger>
                <SelectContent>
                  {roles.map((role) => (
                    <SelectItem key={role.id} value={role.id.toString()}>
                      {role.name}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
          </div>
          <DialogFooter>
            <Button
              type="submit"
              disabled={loading}
            >
              {loading ? "Sending..." : "Send Invitation"}
            </Button>
          </DialogFooter>
        </form>
      </DialogContent>
    </Dialog>
  );
} 