"use client";

import { formatDistanceToNow } from "date-fns";
import Link from "next/link";
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

interface Study {
  id: string;
  name: string;
  description: string;
  status: "draft" | "active" | "completed" | "archived";
  institution: string;
  irbProtocolNumber?: string;
  createdAt: Date;
  updatedAt: Date;
  ownerId: string;
  _count?: {
    experiments: number;
    trials: number;
    studyMembers: number;
    participants: number;
  };
  owner: {
    name: string | null;
    email: string;
  };
}

interface StudyCardProps {
  study: Study;
  userRole?: "owner" | "researcher" | "wizard" | "observer";
  isOwner?: boolean;
}

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800 hover:bg-gray-200",
    icon: "📝",
  },
  active: {
    label: "Active",
    className: "bg-green-100 text-green-800 hover:bg-green-200",
    icon: "🟢",
  },
  completed: {
    label: "Completed",
    className: "bg-blue-100 text-blue-800 hover:bg-blue-200",
    icon: "✅",
  },
  archived: {
    label: "Archived",
    className: "bg-orange-100 text-orange-800 hover:bg-orange-200",
    icon: "📦",
  },
};

export function StudyCard({ study, userRole, isOwner }: StudyCardProps) {
  const statusInfo = statusConfig[study.status];
  const canEdit =
    isOwner ?? (userRole === "owner" || userRole === "researcher");

  return (
    <Card className="group transition-all duration-200 hover:border-slate-300 hover:shadow-md">
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div className="min-w-0 flex-1">
            <CardTitle className="truncate text-lg font-semibold text-slate-900 transition-colors group-hover:text-blue-600">
              <Link href={`/studies/${study.id}`} className="hover:underline">
                {study.name}
              </Link>
            </CardTitle>
            <CardDescription className="mt-1 line-clamp-2 text-sm text-slate-600">
              {study.description}
            </CardDescription>
          </div>
          <Badge className={statusInfo.className} variant="secondary">
            <span className="mr-1">{statusInfo.icon}</span>
            {statusInfo.label}
          </Badge>
        </div>
      </CardHeader>

      <CardContent className="space-y-4">
        {/* Institution and IRB */}
        <div className="space-y-1">
          <div className="flex items-center text-sm text-slate-600">
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
                d="M19 21V5a2 2 0 00-2-2H7a2 2 0 00-2 2v16m14 0h2m-2 0h-4m-5 0H3m2 0h4M9 7h6m-6 4h6m-6 4h6"
              />
            </svg>
            {study.institution}
          </div>
          {study.irbProtocolNumber && (
            <div className="flex items-center text-sm text-slate-500">
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
                  d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z"
                />
              </svg>
              IRB: {study.irbProtocolNumber}
            </div>
          )}
        </div>

        {/* Statistics */}
        {study._count && (
          <>
            <Separator />
            <div className="grid grid-cols-2 gap-4 text-sm">
              <div className="space-y-1">
                <div className="flex justify-between">
                  <span className="text-slate-600">Experiments:</span>
                  <span className="font-medium">
                    {study._count.experiments}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-600">Trials:</span>
                  <span className="font-medium">{study._count.trials}</span>
                </div>
              </div>
              <div className="space-y-1">
                <div className="flex justify-between">
                  <span className="text-slate-600">Team:</span>
                  <span className="font-medium">
                    {study._count.studyMembers}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-slate-600">Participants:</span>
                  <span className="font-medium">
                    {study._count.participants}
                  </span>
                </div>
              </div>
            </div>
          </>
        )}

        {/* Metadata */}
        <Separator />
        <div className="space-y-1 text-xs text-slate-500">
          <div className="flex justify-between">
            <span>Created:</span>
            <span>
              {formatDistanceToNow(study.createdAt, { addSuffix: true })}
            </span>
          </div>
          <div className="flex justify-between">
            <span>Owner:</span>
            <span className="ml-2 truncate">
              {study.owner.name ?? study.owner.email}
            </span>
          </div>
          {study.updatedAt !== study.createdAt && (
            <div className="flex justify-between">
              <span>Updated:</span>
              <span>
                {formatDistanceToNow(study.updatedAt, { addSuffix: true })}
              </span>
            </div>
          )}
        </div>

        {/* Actions */}
        <div className="flex gap-2 pt-2">
          <Button asChild size="sm" className="flex-1">
            <Link href={`/studies/${study.id}`}>View Details</Link>
          </Button>
          {canEdit && (
            <Button asChild size="sm" variant="outline" className="flex-1">
              <Link href={`/studies/${study.id}/edit`}>Edit</Link>
            </Button>
          )}
        </div>

        {/* Role indicator */}
        {userRole && (
          <div className="flex items-center justify-center pt-1">
            <span className="text-xs text-slate-500 capitalize">
              Your role: <span className="font-medium">{userRole}</span>
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  );
}
