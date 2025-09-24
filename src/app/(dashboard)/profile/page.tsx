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
import { User, Shield, Download, Trash2, ExternalLink } from "lucide-react";
import { useSession } from "next-auth/react";

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
    <div className="space-y-6">
      <PageHeader
        title="Profile"
        description="Manage your account settings and preferences"
        icon={User}
      />

      <div className="grid grid-cols-1 gap-6 lg:grid-cols-3">
        {/* Profile Information */}
        <div className="space-y-6 lg:col-span-2">
          {/* Basic Information */}
          <Card>
            <CardHeader>
              <CardTitle>Basic Information</CardTitle>
              <CardDescription>
                Your personal account information
              </CardDescription>
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

          {/* Password Change */}
          <Card>
            <CardHeader>
              <CardTitle>Password</CardTitle>
              <CardDescription>Change your account password</CardDescription>
            </CardHeader>
            <CardContent>
              <PasswordChangeForm />
            </CardContent>
          </Card>

          {/* Account Actions */}
          <Card>
            <CardHeader>
              <CardTitle>Account Actions</CardTitle>
              <CardDescription>Manage your account settings</CardDescription>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex items-center justify-between">
                <div>
                  <h4 className="text-sm font-medium">Export Data</h4>
                  <p className="text-muted-foreground text-sm">
                    Download all your research data and account information
                  </p>
                </div>
                <Button variant="outline" disabled>
                  <Download className="mr-2 h-4 w-4" />
                  Export Data
                </Button>
              </div>

              <Separator />

              <div className="flex items-center justify-between">
                <div>
                  <h4 className="text-destructive text-sm font-medium">
                    Delete Account
                  </h4>
                  <p className="text-muted-foreground text-sm">
                    Permanently delete your account and all associated data
                  </p>
                </div>
                <Button variant="destructive" disabled>
                  <Trash2 className="mr-2 h-4 w-4" />
                  Delete Account
                </Button>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Sidebar */}
        <div className="space-y-6">
          {/* User Summary */}
          <Card>
            <CardHeader>
              <CardTitle>Account Summary</CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex items-center space-x-3">
                <div className="bg-primary/10 flex h-12 w-12 items-center justify-center rounded-full">
                  <span className="text-primary text-lg font-semibold">
                    {(user.name ?? user.email ?? "U").charAt(0).toUpperCase()}
                  </span>
                </div>
                <div>
                  <p className="font-medium">{user.name ?? "Unnamed User"}</p>
                  <p className="text-muted-foreground text-sm">{user.email}</p>
                </div>
              </div>

              <Separator />

              <div>
                <p className="mb-2 text-sm font-medium">User ID</p>
                <p className="text-muted-foreground bg-muted rounded p-2 font-mono text-xs break-all">
                  {user.id}
                </p>
              </div>
            </CardContent>
          </Card>

          {/* System Roles */}
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center gap-2">
                <Shield className="h-4 w-4" />
                System Roles
              </CardTitle>
              <CardDescription>Your current system permissions</CardDescription>
            </CardHeader>
            <CardContent>
              {user.roles && user.roles.length > 0 ? (
                <div className="space-y-3">
                  {user.roles.map((roleInfo, index: number) => (
                    <div
                      key={index}
                      className="flex items-start justify-between"
                    >
                      <div className="flex-1">
                        <div className="mb-1 flex items-center gap-2">
                          <Badge variant="secondary">
                            {formatRole(roleInfo.role)}
                          </Badge>
                        </div>
                        <p className="text-muted-foreground text-xs">
                          {getRoleDescription(roleInfo.role)}
                        </p>
                        <p className="text-muted-foreground/80 mt-1 text-xs">
                          Granted{" "}
                          {new Date(roleInfo.grantedAt).toLocaleDateString()}
                        </p>
                      </div>
                    </div>
                  ))}

                  <Separator />

                  <div className="text-center">
                    <p className="text-muted-foreground text-xs">
                      Need additional permissions?{" "}
                      <Button
                        variant="link"
                        size="sm"
                        className="h-auto p-0 text-xs"
                      >
                        Contact an administrator
                        <ExternalLink className="ml-1 h-3 w-3" />
                      </Button>
                    </p>
                  </div>
                </div>
              ) : (
                <div className="py-6 text-center">
                  <div className="bg-muted mx-auto mb-3 flex h-12 w-12 items-center justify-center rounded-lg">
                    <Shield className="text-muted-foreground h-6 w-6" />
                  </div>
                  <p className="mb-1 text-sm font-medium">No Roles Assigned</p>
                  <p className="text-muted-foreground text-xs">
                    You don&apos;t have any system roles yet. Contact an
                    administrator to get access to HRIStudio features.
                  </p>
                  <Button size="sm" variant="outline">
                    Request Access
                  </Button>
                </div>
              )}
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
}

export default function ProfilePage() {
  const { data: session } = useSession();

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Profile" },
  ]);

  if (!session?.user) {
    redirect("/auth/signin");
  }

  const user = session.user;

  return <ProfileContent user={user} />;
}
