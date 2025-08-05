import { formatDistanceToNow } from "date-fns";
import {
  BarChart3,
  Building,
  Calendar,
  FlaskConical,
  Plus,
  Settings,
  Shield,
  Users,
} from "lucide-react";
import Link from "next/link";
import { notFound } from "next/navigation";
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
import { api } from "~/trpc/server";

interface StudyDetailPageProps {
  params: Promise<{
    id: string;
  }>;
}

const statusConfig = {
  draft: {
    label: "Draft",
    className: "bg-gray-100 text-gray-800",
    icon: "📝",
  },
  active: {
    label: "Active",
    className: "bg-green-100 text-green-800",
    icon: "🟢",
  },
  completed: {
    label: "Completed",
    className: "bg-blue-100 text-blue-800",
    icon: "✅",
  },
  archived: {
    label: "Archived",
    className: "bg-orange-100 text-orange-800",
    icon: "📦",
  },
};

export default async function StudyDetailPage({
  params,
}: StudyDetailPageProps) {
  try {
    const resolvedParams = await params;
    const study = await api.studies.get({ id: resolvedParams.id });
    const members = await api.studies.getMembers({
      studyId: resolvedParams.id,
    });

    if (!study) {
      notFound();
    }

    const statusInfo = statusConfig[study.status];

    return (
      <div className="p-8">
        {/* Header */}
        <div className="mb-8">
          <div className="mb-4 flex items-center space-x-2 text-sm text-slate-600">
            <Link href="/studies" className="hover:text-slate-900">
              Studies
            </Link>
            <span>/</span>
            <span className="text-slate-900">{study.name}</span>
          </div>

          <div className="flex items-start justify-between">
            <div className="min-w-0 flex-1">
              <div className="mb-2 flex items-center space-x-3">
                <h1 className="truncate text-3xl font-bold text-slate-900">
                  {study.name}
                </h1>
                <Badge className={statusInfo.className} variant="secondary">
                  <span className="mr-1">{statusInfo.icon}</span>
                  {statusInfo.label}
                </Badge>
              </div>
              <p className="text-lg text-slate-600">{study.description}</p>
            </div>

            <div className="ml-4 flex items-center space-x-2">
              <Button asChild variant="outline">
                <Link href={`/studies/${study.id}/edit`}>
                  <Settings className="mr-2 h-4 w-4" />
                  Edit Study
                </Link>
              </Button>
              <Button asChild>
                <Link href={`/studies/${study.id}/experiments/new`}>
                  <Plus className="mr-2 h-4 w-4" />
                  New Experiment
                </Link>
              </Button>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-1 gap-8 lg:grid-cols-3">
          {/* Main Content */}
          <div className="space-y-8 lg:col-span-2">
            {/* Study Information */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Building className="h-5 w-5" />
                  <span>Study Information</span>
                </CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid grid-cols-1 gap-4 md:grid-cols-2">
                  <div>
                    <label className="text-sm font-medium text-slate-700">
                      Institution
                    </label>
                    <p className="text-slate-900">{study.institution}</p>
                  </div>
                  {study.irbProtocol && (
                    <div>
                      <label className="text-sm font-medium text-slate-700">
                        IRB Protocol
                      </label>
                      <p className="text-slate-900">{study.irbProtocol}</p>
                    </div>
                  )}
                  <div>
                    <label className="text-sm font-medium text-slate-700">
                      Created
                    </label>
                    <p className="text-slate-900">
                      {formatDistanceToNow(study.createdAt, {
                        addSuffix: true,
                      })}
                    </p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-slate-700">
                      Last Updated
                    </label>
                    <p className="text-slate-900">
                      {formatDistanceToNow(study.updatedAt, {
                        addSuffix: true,
                      })}
                    </p>
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Experiments */}
            <Card>
              <CardHeader>
                <div className="flex items-center justify-between">
                  <CardTitle className="flex items-center space-x-2">
                    <FlaskConical className="h-5 w-5" />
                    <span>Experiments</span>
                  </CardTitle>
                  <Button asChild variant="outline" size="sm">
                    <Link href={`/studies/${study.id}/experiments/new`}>
                      <Plus className="mr-2 h-4 w-4" />
                      Add Experiment
                    </Link>
                  </Button>
                </div>
                <CardDescription>
                  Design and manage experimental protocols for this study
                </CardDescription>
              </CardHeader>
              <CardContent>
                {/* Placeholder for experiments list */}
                <div className="py-8 text-center">
                  <FlaskConical className="mx-auto mb-4 h-12 w-12 text-slate-400" />
                  <h3 className="mb-2 text-lg font-semibold text-slate-900">
                    No Experiments Yet
                  </h3>
                  <p className="mb-4 text-slate-600">
                    Create your first experiment to start designing research
                    protocols
                  </p>
                  <Button asChild>
                    <Link href={`/studies/${study.id}/experiments/new`}>
                      Create First Experiment
                    </Link>
                  </Button>
                </div>
              </CardContent>
            </Card>

            {/* Recent Activity */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <BarChart3 className="h-5 w-5" />
                  <span>Recent Activity</span>
                </CardTitle>
              </CardHeader>
              <CardContent>
                <div className="py-8 text-center">
                  <Calendar className="mx-auto mb-4 h-12 w-12 text-slate-400" />
                  <h3 className="mb-2 text-lg font-semibold text-slate-900">
                    No Recent Activity
                  </h3>
                  <p className="text-slate-600">
                    Activity will appear here once you start working on this
                    study
                  </p>
                </div>
              </CardContent>
            </Card>
          </div>

          {/* Sidebar */}
          <div className="space-y-6">
            {/* Team Members */}
            <Card>
              <CardHeader>
                <div className="flex items-center justify-between">
                  <CardTitle className="flex items-center space-x-2">
                    <Users className="h-5 w-5" />
                    <span>Team</span>
                  </CardTitle>
                  <Button variant="outline" size="sm">
                    <Plus className="mr-2 h-4 w-4" />
                    Invite
                  </Button>
                </div>
                <CardDescription>
                  {members.length} team member{members.length !== 1 ? "s" : ""}
                </CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  {members.map((member) => (
                    <div
                      key={member.user.id}
                      className="flex items-center space-x-3"
                    >
                      <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                        <span className="text-sm font-medium text-blue-600">
                          {(member.user.name || member.user.email)
                            .charAt(0)
                            .toUpperCase()}
                        </span>
                      </div>
                      <div className="min-w-0 flex-1">
                        <p className="truncate text-sm font-medium text-slate-900">
                          {member.user.name || member.user.email}
                        </p>
                        <p className="text-xs text-slate-500 capitalize">
                          {member.role}
                        </p>
                      </div>
                      {member.role === "owner" && (
                        <Shield className="h-4 w-4 text-amber-600" />
                      )}
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>

            {/* Quick Stats */}
            <Card>
              <CardHeader>
                <CardTitle>Quick Stats</CardTitle>
              </CardHeader>
              <CardContent>
                <div className="space-y-3">
                  <div className="flex justify-between">
                    <span className="text-sm text-slate-600">Experiments:</span>
                    <span className="font-medium">0</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-sm text-slate-600">
                      Total Trials:
                    </span>
                    <span className="font-medium">0</span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-sm text-slate-600">
                      Participants:
                    </span>
                    <span className="font-medium">0</span>
                  </div>
                  <Separator />
                  <div className="flex justify-between">
                    <span className="text-sm text-slate-600">
                      Completion Rate:
                    </span>
                    <span className="font-medium text-green-600">—</span>
                  </div>
                </div>
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
                  <Link href={`/studies/${study.id}/participants`}>
                    <Users className="mr-2 h-4 w-4" />
                    Manage Participants
                  </Link>
                </Button>
                <Button
                  asChild
                  variant="outline"
                  className="w-full justify-start"
                >
                  <Link href={`/studies/${study.id}/trials`}>
                    <Calendar className="mr-2 h-4 w-4" />
                    Schedule Trials
                  </Link>
                </Button>
                <Button
                  asChild
                  variant="outline"
                  className="w-full justify-start"
                >
                  <Link href={`/studies/${study.id}/analytics`}>
                    <BarChart3 className="mr-2 h-4 w-4" />
                    View Analytics
                  </Link>
                </Button>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    );
  } catch (error) {
    console.error("Error loading study:", error);
    notFound();
  }
}
