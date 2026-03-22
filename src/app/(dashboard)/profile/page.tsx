"use client";

import * as React from "react";
import { redirect } from "next/navigation";
import Link from "next/link";
import { useSession } from "~/lib/auth-client";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { format } from "date-fns";
import {
  User,
  Mail,
  Shield,
  Lock,
  Settings,
  Building,
  Calendar,
  ChevronRight,
  Loader2,
  Save,
  X,
  Crown,
  FlaskConical,
  Eye,
  UserCheck,
} from "lucide-react";

import { Button } from "~/components/ui/button";
import { Input } from "~/components/ui/input";
import { Label } from "~/components/ui/label";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "~/components/ui/dialog";

interface Membership {
  studyId: string;
  role: string;
  joinedAt: Date;
}

function getMemberRole(memberships: Membership[], studyId: string): string {
  const membership = memberships.find((m) => m.studyId === studyId);
  return membership?.role ?? "observer";
}

function ProfilePageContent() {
  const { data: session } = useSession();
  const utils = api.useUtils();
  const [isEditing, setIsEditing] = React.useState(false);
  const [name, setName] = React.useState(session?.user?.name ?? "");
  const [email, setEmail] = React.useState(session?.user?.email ?? "");
  const [passwordOpen, setPasswordOpen] = React.useState(false);
  const [currentPassword, setCurrentPassword] = React.useState("");
  const [newPassword, setNewPassword] = React.useState("");
  const [confirmPassword, setConfirmPassword] = React.useState("");

  const { data: userData } = api.users.get.useQuery(
    { id: session?.user?.id ?? "" },
    { enabled: !!session?.user?.id },
  );

  const { data: userStudies } = api.studies.list.useQuery({
    memberOnly: true,
    limit: 10,
  });

  const { data: membershipsData } = api.studies.getMyMemberships.useQuery();

  const studyMemberships = membershipsData ?? [];

  const updateProfile = api.users.update.useMutation({
    onSuccess: () => {
      toast.success("Profile updated successfully");
      void utils.users.get.invalidate();
      setIsEditing(false);
    },
    onError: (error) => {
      toast.error("Failed to update profile", { description: error.message });
    },
  });

  const changePassword = api.users.changePassword.useMutation({
    onSuccess: () => {
      toast.success("Password changed successfully");
      setPasswordOpen(false);
      setCurrentPassword("");
      setNewPassword("");
      setConfirmPassword("");
    },
    onError: (error) => {
      toast.error("Failed to change password", { description: error.message });
    },
  });

  const handleSave = () => {
    if (!name.trim()) {
      toast.error("Name is required");
      return;
    }
    updateProfile.mutate({ id: session?.user?.id ?? "", name, email });
  };

  const handlePasswordChange = (e: React.FormEvent) => {
    e.preventDefault();
    if (newPassword !== confirmPassword) {
      toast.error("Passwords don't match");
      return;
    }
    if (newPassword.length < 8) {
      toast.error("Password must be at least 8 characters");
      return;
    }
    changePassword.mutate({
      currentPassword,
      newPassword,
    });
  };

  const user = userData ?? session?.user;
  const roles = (userData as any)?.systemRoles ?? [];
  const initials = (user?.name ?? user?.email ?? "U").charAt(0).toUpperCase();

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <div className="flex h-16 w-16 items-center justify-center rounded-full bg-primary text-xl font-bold text-primary-foreground">
            {initials}
          </div>
          <div>
            <h1 className="text-2xl font-bold">{user?.name ?? "User"}</h1>
            <p className="text-muted-foreground">{user?.email}</p>
            {roles.length > 0 && (
              <div className="mt-1 flex gap-2">
                {roles.map((role: any) => (
                  <Badge key={role.role} variant="secondary" className="text-xs">
                    {role.role}
                  </Badge>
                ))}
              </div>
            )}
          </div>
        </div>
        <div className="flex gap-2">
          {isEditing ? (
            <>
              <Button variant="outline" onClick={() => setIsEditing(false)}>
                <X className="mr-2 h-4 w-4" />
                Cancel
              </Button>
              <Button onClick={handleSave} disabled={updateProfile.isPending}>
                {updateProfile.isPending ? (
                  <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                ) : (
                  <Save className="mr-2 h-4 w-4" />
                )}
                Save Changes
              </Button>
            </>
          ) : (
            <Button variant="outline" onClick={() => setIsEditing(true)}>
              <Settings className="mr-2 h-4 w-4" />
              Edit Profile
            </Button>
          )}
        </div>
      </div>

      {/* Main Content */}
      <div className="grid gap-6 lg:grid-cols-3">
        {/* Left Column - Profile Info */}
        <div className="space-y-6 lg:col-span-2">
          {/* Personal Information */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <User className="h-5 w-5 text-primary" />
                Personal Information
              </CardTitle>
              <CardDescription>
                Your public profile information
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="grid gap-4 md:grid-cols-2">
                <div className="space-y-2">
                  <Label htmlFor="name">Full Name</Label>
                  {isEditing ? (
                    <Input
                      id="name"
                      value={name}
                      onChange={(e) => setName(e.target.value)}
                      placeholder="Your name"
                    />
                  ) : (
                    <div className="flex items-center gap-2 rounded-md border bg-muted/50 p-2">
                      <User className="text-muted-foreground h-4 w-4" />
                      <span>{name || "Not set"}</span>
                    </div>
                  )}
                </div>
                <div className="space-y-2">
                  <Label htmlFor="email">Email Address</Label>
                  {isEditing ? (
                    <Input
                      id="email"
                      type="email"
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      placeholder="you@example.com"
                    />
                  ) : (
                    <div className="flex items-center gap-2 rounded-md border bg-muted/50 p-2">
                      <Mail className="text-muted-foreground h-4 w-4" />
                      <span>{email}</span>
                    </div>
                  )}
                </div>
              </div>
              <div className="space-y-2">
                <Label>User ID</Label>
                <div className="rounded-md border bg-muted/50 p-2 font-mono text-sm">
                  {user?.id ?? session?.user?.id}
                </div>
              </div>
            </CardContent>
          </Card>

          {/* Recent Activity */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Calendar className="h-5 w-5 text-primary" />
                Recent Activity
              </CardTitle>
              <CardDescription>
                Your recent actions across the platform
              </CardDescription>
            </CardHeader>
            <CardContent>
              <div className="flex flex-col items-center justify-center py-8 text-center">
                <Calendar className="text-muted-foreground/50 mb-3 h-12 w-12" />
                <p className="font-medium">No recent activity</p>
                <p className="text-muted-foreground text-sm">
                  Your recent actions will appear here
                </p>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Right Column - Settings */}
        <div className="space-y-6">
          {/* Security */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Shield className="h-5 w-5 text-primary" />
                Security
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex items-center justify-between rounded-lg border p-3">
                <div className="flex items-center gap-3">
                  <Lock className="text-muted-foreground h-4 w-4" />
                  <div>
                    <p className="text-sm font-medium">Password</p>
                    <p className="text-muted-foreground text-xs">Last changed: Never</p>
                  </div>
                </div>
                <Dialog open={passwordOpen} onOpenChange={setPasswordOpen}>
                  <DialogTrigger asChild>
                    <Button variant="ghost" size="sm">
                      Change
                    </Button>
                  </DialogTrigger>
                  <DialogContent>
                    <DialogHeader>
                      <DialogTitle>Change Password</DialogTitle>
                      <DialogDescription>
                        Enter your current password and choose a new one.
                      </DialogDescription>
                    </DialogHeader>
                    <form onSubmit={handlePasswordChange} className="space-y-4">
                      <div className="space-y-2">
                        <Label htmlFor="current">Current Password</Label>
                        <Input
                          id="current"
                          type="password"
                          value={currentPassword}
                          onChange={(e) => setCurrentPassword(e.target.value)}
                          required
                        />
                      </div>
                      <div className="space-y-2">
                        <Label htmlFor="new">New Password</Label>
                        <Input
                          id="new"
                          type="password"
                          value={newPassword}
                          onChange={(e) => setNewPassword(e.target.value)}
                          required
                          minLength={8}
                        />
                      </div>
                      <div className="space-y-2">
                        <Label htmlFor="confirm">Confirm Password</Label>
                        <Input
                          id="confirm"
                          type="password"
                          value={confirmPassword}
                          onChange={(e) => setConfirmPassword(e.target.value)}
                          required
                        />
                      </div>
                      <DialogFooter>
                        <Button type="button" variant="outline" onClick={() => setPasswordOpen(false)}>
                          Cancel
                        </Button>
                        <Button type="submit" disabled={changePassword.isPending}>
                          {changePassword.isPending && <Loader2 className="mr-2 h-4 w-4 animate-spin" />}
                          Change Password
                        </Button>
                      </DialogFooter>
                    </form>
                  </DialogContent>
                </Dialog>
              </div>

              <Separator />

              <div className="rounded-lg border bg-destructive/5 p-3">
                <p className="text-sm font-medium text-destructive">Danger Zone</p>
                <p className="text-muted-foreground mt-1 text-xs">
                  Account deletion is not available. Contact an administrator for assistance.
                </p>
              </div>
            </CardContent>
          </Card>

          {/* Studies Access */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Building className="h-5 w-5 text-primary" />
                Studies Access
              </CardTitle>
              <CardDescription>
                Studies you have access to
              </CardDescription>
            </CardHeader>
            <CardContent className="space-y-3">
              {userStudies?.studies.slice(0, 5).map((study) => (
                <Link
                  key={study.id}
                  href={`/studies/${study.id}`}
                  className="hover:bg-accent/50 flex items-center justify-between rounded-md border p-3 transition-colors"
                >
                  <div className="flex items-center gap-3">
                    <div className="flex h-8 w-8 items-center justify-center rounded-full bg-primary/10">
                      <span className="text-xs font-medium text-primary">
                        {(study.name ?? "S").charAt(0).toUpperCase()}
                      </span>
                    </div>
                    <div>
                      <p className="text-sm font-medium">{study.name}</p>
                      <p className="text-muted-foreground text-xs capitalize">
                        {getMemberRole(studyMemberships, study.id)}
                      </p>
                    </div>
                  </div>
                  <ChevronRight className="h-4 w-4 text-muted-foreground" />
                </Link>
              ))}
              {(!userStudies?.studies.length) && (
                <div className="flex flex-col items-center justify-center py-4 text-center">
                  <Building className="text-muted-foreground/50 mb-2 h-8 w-8" />
                  <p className="text-sm">No studies yet</p>
                  <Button variant="link" size="sm" asChild className="mt-1">
                    <Link href="/studies/new">Create a study</Link>
                  </Button>
                </div>
              )}
              {userStudies && userStudies.studies.length > 5 && (
                <Button variant="ghost" size="sm" asChild className="w-full">
                  <Link href="/studies">
                    View all {userStudies.studies.length} studies <ChevronRight className="ml-1 h-3 w-3" />
                  </Link>
                </Button>
              )}
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}

export default function ProfilePage() {
  const { data: session, isPending } = useSession();

  if (isPending) {
    return (
      <div className="flex items-center justify-center p-12">
        <Loader2 className="text-muted-foreground h-8 w-8 animate-spin" />
      </div>
    );
  }

  if (!session?.user) {
    redirect("/auth/signin");
  }

  return <ProfilePageContent />;
}
