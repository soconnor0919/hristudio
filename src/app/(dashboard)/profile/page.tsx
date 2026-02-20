"use client";

import { redirect } from "next/navigation";
import { PasswordChangeForm } from "~/components/profile/password-change-form";
import { ProfileEditForm } from "~/components/profile/profile-edit-form";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import { PageHeader } from "~/components/ui/page-header";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { formatRole, getRoleDescription } from "~/lib/auth-client";
import {
  User,
  Shield,
  Download,
  Trash2,
  ExternalLink,
  Lock,
  UserCog,
  Mail,
  Fingerprint
} from "lucide-react";
import { useSession } from "next-auth/react";
import { cn } from "~/lib/utils";

interface ProfileUser {
  id: string;
  name: string | null;
  email: string;
  image: string | null;
  roles?: Array<{
    role: "administrator" | "researcher" | "wizard" | "observer";
    grantedAt: string | Date;
  }>;
}

function ProfileContent({ user }: { user: ProfileUser }) {
  return (
    <div className="space-y-8 animate-in fade-in duration-500">
      <PageHeader
        title={user.name ?? "User"}
        description={user.email}
        icon={User}
        badges={[
          { label: `ID: ${user.id}`, variant: "outline" },
          ...(user.roles?.map((r) => ({
            label: formatRole(r.role),
            variant: "secondary" as const,
          })) ?? []),
        ]}
      />

      <div className="grid grid-cols-1 gap-8 lg:grid-cols-3">
        {/* Main Content (Left Column) */}
        <div className="space-y-8 lg:col-span-2">

          {/* Personal Information */}
          <section className="space-y-4">
            <div className="flex items-center gap-2 pb-2 border-b">
              <User className="h-5 w-5 text-primary" />
              <h3 className="text-lg font-semibold">Personal Information</h3>
            </div>
            <Card className="border-border/60 hover:border-border transition-colors">
              <CardHeader>
                <CardTitle className="text-base">Contact Details</CardTitle>
                <CardDescription>Update your public profile information</CardDescription>
              </CardHeader>
              <CardContent>
                <ProfileEditForm
                  user={{
                    id: user.id,
                    name: user.name,
                    email: user.email,
                    image: user.image,
                  }}
                />
              </CardContent>
            </Card>
          </section>

          {/* Security */}
          <section className="space-y-4">
            <div className="flex items-center gap-2 pb-2 border-b">
              <Lock className="h-5 w-5 text-primary" />
              <h3 className="text-lg font-semibold">Security</h3>
            </div>
            <Card className="border-border/60 hover:border-border transition-colors">
              <CardHeader>
                <CardTitle className="text-base">Password</CardTitle>
                <CardDescription>Ensure your account stays secure</CardDescription>
              </CardHeader>
              <CardContent>
                <PasswordChangeForm />
              </CardContent>
            </Card>
          </section>
        </div>

        {/* Sidebar (Right Column) */}
        <div className="space-y-8">

          {/* Permissions */}
          <section className="space-y-4">
            <div className="flex items-center gap-2 pb-2 border-b">
              <Shield className="h-5 w-5 text-primary" />
              <h3 className="text-lg font-semibold">Permissions</h3>
            </div>
            <Card>
              <CardContent className="pt-6">
                {user.roles && user.roles.length > 0 ? (
                  <div className="space-y-4">
                    {user.roles.map((roleInfo, index) => (
                      <div key={index} className="space-y-2">
                        <div className="flex items-center justify-between">
                          <span className="font-medium text-sm">{formatRole(roleInfo.role)}</span>
                          <span className="text-[10px] text-muted-foreground bg-muted px-1.5 py-0.5 rounded">
                            Since {new Date(roleInfo.grantedAt).toLocaleDateString()}
                          </span>
                        </div>
                        <p className="text-xs text-muted-foreground leading-relaxed">
                          {getRoleDescription(roleInfo.role)}
                        </p>
                        {index < (user.roles?.length || 0) - 1 && <Separator className="my-2" />}
                      </div>
                    ))}
                    <div className="bg-blue-50/50 dark:bg-blue-900/10 p-3 rounded-lg border border-blue-100 dark:border-blue-900/30 text-xs text-muted-foreground mt-4">
                      <div className="flex items-center gap-2 mb-1 text-primary font-medium">
                        <Shield className="h-3 w-3" />
                        <span>Role Management</span>
                      </div>
                      System roles are managed by administrators. Contact support if you need access adjustments.
                    </div>
                  </div>
                ) : (
                  <div className="text-center py-4">
                    <p className="text-sm font-medium">No Roles Assigned</p>
                    <p className="text-xs text-muted-foreground mt-1">Contact an admin to request access.</p>
                    <Button size="sm" variant="outline" className="mt-3 w-full">Request Access</Button>
                  </div>
                )}
              </CardContent>
            </Card>
          </section>

          {/* Data & Privacy */}
          <section className="space-y-4">
            <div className="flex items-center gap-2 pb-2 border-b">
              <Download className="h-5 w-5 text-primary" />
              <h3 className="text-lg font-semibold">Data & Privacy</h3>
            </div>

            <Card className="border-destructive/10 bg-destructive/5 overflow-hidden">
              <CardContent className="pt-6 space-y-4">
                <div>
                  <h4 className="text-sm font-semibold mb-1">Export Data</h4>
                  <p className="text-xs text-muted-foreground mb-3">Download a copy of your personal data.</p>
                  <Button variant="outline" size="sm" className="w-full bg-background" disabled>
                    <Download className="mr-2 h-3 w-3" />
                    Download Archive
                  </Button>
                </div>
                <Separator className="bg-destructive/10" />
                <div>
                  <h4 className="text-sm font-semibold text-destructive mb-1">Delete Account</h4>
                  <p className="text-xs text-muted-foreground mb-3">This action is irreversible.</p>
                  <Button variant="destructive" size="sm" className="w-full" disabled>
                    <Trash2 className="mr-2 h-3 w-3" />
                    Delete Account
                  </Button>
                </div>
              </CardContent>
            </Card>
          </section>
        </div>
      </div>
    </div>
  );
}

export default function ProfilePage() {
  const { data: session, status } = useSession();

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Profile" },
  ]);

  if (status === "loading") {
    return <div className="p-8 text-muted-foreground animate-pulse">Loading profile...</div>;
  }

  if (!session?.user) {
    redirect("/auth/signin");
  }

  const user = session.user;

  return <ProfileContent user={user} />;
}
