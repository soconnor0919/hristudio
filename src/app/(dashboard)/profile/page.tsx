import { auth } from "~/server/auth";
import { redirect } from "next/navigation";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Badge } from "~/components/ui/badge";
import { Separator } from "~/components/ui/separator";
import { formatRole, getRoleDescription } from "~/lib/auth-client";
import { ProfileEditForm } from "~/components/profile/profile-edit-form";
import { PasswordChangeForm } from "~/components/profile/password-change-form";

export default async function ProfilePage() {
  const session = await auth();

  if (!session?.user) {
    redirect("/auth/signin");
  }

  const user = session.user;

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-slate-100">
      <div className="container mx-auto px-4 py-8">
        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold text-slate-900">Profile</h1>
            <p className="text-slate-600">
              Manage your account settings and preferences
            </p>
          </div>

          <div className="flex items-center gap-4">
            <span className="text-sm text-slate-600">
              Welcome, {user.name ?? user.email}
            </span>
            <div className="flex gap-2">
              <Button asChild variant="outline" size="sm">
                <Link href="/auth/signout">Sign Out</Link>
              </Button>
              <Button asChild variant="outline">
                <Link href="/">← Back to Home</Link>
              </Button>
            </div>
          </div>
        </div>

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
                    <p className="text-sm text-slate-600">
                      Download all your research data and account information
                    </p>
                  </div>
                  <Button variant="outline" disabled>
                    Export Data
                  </Button>
                </div>

                <Separator />

                <div className="flex items-center justify-between">
                  <div>
                    <h4 className="text-sm font-medium text-red-700">
                      Delete Account
                    </h4>
                    <p className="text-sm text-slate-600">
                      Permanently delete your account and all associated data
                    </p>
                  </div>
                  <Button variant="destructive" disabled>
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
                  <div className="flex h-12 w-12 items-center justify-center rounded-full bg-blue-100">
                    <span className="text-lg font-semibold text-blue-600">
                      {(user.name ?? user.email ?? "U").charAt(0).toUpperCase()}
                    </span>
                  </div>
                  <div>
                    <p className="font-medium">{user.name ?? "Unnamed User"}</p>
                    <p className="text-sm text-slate-600">{user.email}</p>
                  </div>
                </div>

                <Separator />

                <div>
                  <p className="mb-2 text-sm font-medium">User ID</p>
                  <p className="rounded bg-slate-100 p-2 font-mono text-xs break-all text-slate-600">
                    {user.id}
                  </p>
                </div>
              </CardContent>
            </Card>

            {/* System Roles */}
            <Card>
              <CardHeader>
                <CardTitle>System Roles</CardTitle>
                <CardDescription>
                  Your current system permissions
                </CardDescription>
              </CardHeader>
              <CardContent>
                {user.roles && user.roles.length > 0 ? (
                  <div className="space-y-3">
                    {user.roles.map((roleInfo, index) => (
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
                          <p className="text-xs text-slate-600">
                            {getRoleDescription(roleInfo.role)}
                          </p>
                          <p className="mt-1 text-xs text-slate-500">
                            Granted {roleInfo.grantedAt.toLocaleDateString()}
                          </p>
                        </div>
                      </div>
                    ))}

                    <Separator />

                    <div className="text-center">
                      <p className="text-xs text-slate-500">
                        Need additional permissions?{" "}
                        <Link
                          href="/contact"
                          className="text-blue-600 hover:text-blue-500"
                        >
                          Contact an administrator
                        </Link>
                      </p>
                    </div>
                  </div>
                ) : (
                  <div className="py-6 text-center">
                    <div className="mx-auto mb-3 flex h-12 w-12 items-center justify-center rounded-lg bg-yellow-100">
                      <svg
                        className="h-6 w-6 text-yellow-600"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-2.5L13.732 4c-.77-.833-1.728-.833-2.498 0L4.316 16.5c-.77.833.192 2.5 1.732 2.5z"
                        />
                      </svg>
                    </div>
                    <p className="mb-1 text-sm font-medium text-slate-900">
                      No Roles Assigned
                    </p>
                    <p className="mb-3 text-xs text-slate-600">
                      You don&apos;t have any system roles yet. Contact an
                      administrator to get access to HRIStudio features.
                    </p>
                    <Button asChild size="sm" variant="outline">
                      <Link href="/contact">Request Access</Link>
                    </Button>
                  </div>
                )}
              </CardContent>
            </Card>

            {/* Quick Actions */}
            <Card>
              <CardHeader>
                <CardTitle>Quick Actions</CardTitle>
              </CardHeader>
              <CardContent className="space-y-2">
                <Button
                  asChild
                  variant="outline"
                  className="w-full justify-start"
                >
                  <Link href="/studies">
                    <svg
                      className="mr-2 h-4 w-4"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z"
                      />
                    </svg>
                    My Studies
                  </Link>
                </Button>

                <Button
                  asChild
                  variant="outline"
                  className="w-full justify-start"
                  disabled
                >
                  <Link href="/experiments">
                    <svg
                      className="mr-2 h-4 w-4"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z"
                      />
                    </svg>
                    Experiments
                  </Link>
                </Button>

                <Button
                  asChild
                  variant="outline"
                  className="w-full justify-start"
                  disabled
                >
                  <Link href="/wizard">
                    <svg
                      className="mr-2 h-4 w-4"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M13 10V3L4 14h7v7l9-11h-7z"
                      />
                    </svg>
                    Wizard Interface
                  </Link>
                </Button>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </div>
  );
}
