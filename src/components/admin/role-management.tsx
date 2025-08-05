"use client";

import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Separator } from "~/components/ui/separator";
import {
    getAvailableRoles, getRoleColor, getRolePermissions
} from "~/lib/auth-client";

export function RoleManagement() {
  const availableRoles = getAvailableRoles();

  // Mock data for role statistics - in a real implementation, this would come from an API
  const roleStats = {
    administrator: 2,
    researcher: 15,
    wizard: 8,
    observer: 12,
  };

  return (
    <div className="space-y-4">
      <div>
        <h3 className="mb-2 text-sm font-medium text-slate-900">
          System Roles Overview
        </h3>
        <p className="text-xs text-slate-600">
          Roles define user permissions and access levels within HRIStudio
        </p>
      </div>

      <div className="space-y-3">
        {availableRoles.map((role) => (
          <Card key={role.value} className="border-l-4 border-l-transparent">
            <CardHeader className="pb-2">
              <div className="flex items-center justify-between">
                <CardTitle className="text-sm">
                  <Badge
                    variant="outline"
                    className={`${getRoleColor(role.value)} text-xs`}
                  >
                    {role.label}
                  </Badge>
                </CardTitle>
                <span className="text-xs text-slate-500">
                  {roleStats[role.value as keyof typeof roleStats] || 0} users
                </span>
              </div>
            </CardHeader>
            <CardContent className="space-y-3">
              <p className="text-xs text-slate-600">{role.description}</p>

              <Separator />

              <div>
                <h4 className="mb-2 text-xs font-medium text-slate-700">
                  Key Permissions:
                </h4>
                <div className="space-y-1">
                  {getRolePermissions(role.value)
                    ?.slice(0, 3)
                    .map((permission, index) => (
                      <div key={index} className="flex items-center gap-2">
                        <div className="h-1.5 w-1.5 rounded-full bg-slate-400"></div>
                        <span className="text-xs text-slate-600">
                          {permission}
                        </span>
                      </div>
                    ))}
                  {getRolePermissions(role.value)?.length > 3 && (
                    <div className="flex items-center gap-2">
                      <div className="h-1.5 w-1.5 rounded-full bg-slate-300"></div>
                      <span className="text-xs text-slate-500">
                        +{getRolePermissions(role.value).length - 3} more
                      </span>
                    </div>
                  )}
                </div>
              </div>
            </CardContent>
          </Card>
        ))}
      </div>

      <Separator />

      <div className="rounded-md bg-blue-50 p-3">
        <div className="flex items-start gap-2">
          <div className="flex h-5 w-5 items-center justify-center rounded-full bg-blue-100">
            <svg
              className="h-3 w-3 text-blue-600"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
              />
            </svg>
          </div>
          <div>
            <h4 className="text-xs font-medium text-blue-900">
              Role Hierarchy
            </h4>
            <p className="mt-1 text-xs text-blue-800">
              Administrator has access to all features. Users can have multiple
              roles for different access levels.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}
