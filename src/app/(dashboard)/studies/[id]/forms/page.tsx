"use client";

import { useEffect, useState } from "react";
import { useParams, useRouter } from "next/navigation";
import { useSession } from "~/lib/auth-client";
import { notFound } from "next/navigation";
import Link from "next/link";
import {
  FileText,
  Plus,
  Search,
  ClipboardList,
  FileQuestion,
  FileSignature,
  MoreHorizontal,
  Pencil,
  Trash2,
  Eye,
  Copy,
  CheckCircle,
  XCircle,
  Clock,
} from "lucide-react";
import {
  EntityView,
  EntityViewSection,
  EmptyState,
} from "~/components/ui/entity-view";
import { useBreadcrumbsEffect } from "~/components/ui/breadcrumb-provider";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import { Card, CardContent, CardHeader, CardTitle } from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuTrigger,
} from "~/components/ui/dropdown-menu";
import { api } from "~/trpc/react";
import { toast } from "sonner";
import { PageHeader } from "~/components/ui/page-header";

const formTypeIcons = {
  consent: FileSignature,
  survey: ClipboardList,
  questionnaire: FileQuestion,
};

const formTypeColors = {
  consent: "bg-purple-100 text-purple-700 dark:bg-purple-900/30 dark:text-purple-400",
  survey: "bg-blue-100 text-blue-700 dark:bg-blue-900/30 dark:text-blue-400",
  questionnaire: "bg-orange-100 text-orange-700 dark:bg-orange-900/30 dark:text-orange-400",
};

const statusConfig = {
  active: {
    label: "Active",
    variant: "default" as const,
    icon: CheckCircle,
  },
  draft: {
    label: "Draft",
    variant: "secondary" as const,
    icon: Clock,
  },
  deprecated: {
    label: "Deprecated",
    variant: "destructive" as const,
    icon: XCircle,
  },
};

interface StudyFormsPageProps {
  params: Promise<{
    id: string;
  }>;
}

export default function StudyFormsPage({ params }: StudyFormsPageProps) {
  const { data: session } = useSession();
  const router = useRouter();
  const utils = api.useUtils();
  const [resolvedParams, setResolvedParams] = useState<{ id: string } | null>(null);
  const [search, setSearch] = useState("");

  useEffect(() => {
    const resolveParams = async () => {
      const resolved = await params;
      setResolvedParams(resolved);
    };
    void resolveParams();
  }, [params]);

  const { data: study } = api.studies.get.useQuery(
    { id: resolvedParams?.id ?? "" },
    { enabled: !!resolvedParams?.id },
  );

  const { data: formsData, isLoading } = api.forms.list.useQuery(
    { studyId: resolvedParams?.id ?? "", search: search || undefined },
    { enabled: !!resolvedParams?.id },
  );

  const userRole = (study as any)?.userRole;
  const canManage = userRole === "owner" || userRole === "researcher";

  const deleteMutation = api.forms.delete.useMutation({
    onSuccess: () => {
      toast.success("Form deleted successfully");
      void utils.forms.list.invalidate({ studyId: resolvedParams?.id });
    },
    onError: (error) => {
      toast.error("Failed to delete form", { description: error.message });
    },
  });

  const setActiveMutation = api.forms.setActive.useMutation({
    onSuccess: () => {
      toast.success("Form set as active");
      void utils.forms.list.invalidate({ studyId: resolvedParams?.id });
    },
    onError: (error) => {
      toast.error("Failed to set active", { description: error.message });
    },
  });

  useBreadcrumbsEffect([
    { label: "Dashboard", href: "/dashboard" },
    { label: "Studies", href: "/studies" },
    { label: study?.name ?? "Study", href: `/studies/${resolvedParams?.id}` },
    { label: "Forms" },
  ]);

  if (!session?.user) {
    return notFound();
  }

  if (!study) return <div>Loading...</div>;

  const forms = formsData?.forms ?? [];

  return (
    <EntityView>
      <PageHeader
        title="Forms"
        description="Manage consent forms, surveys, and questionnaires for this study"
        icon={FileText}
        actions={
          canManage && (
            <Button asChild>
              <Link href={`/studies/${resolvedParams?.id}/forms/new`}>
                <Plus className="mr-2 h-4 w-4" />
                Create Form
              </Link>
            </Button>
          )
        }
      />

      {forms.length === 0 && !isLoading ? (
        <EmptyState
          icon="FileText"
          title="No Forms Yet"
          description="Create consent forms, surveys, or questionnaires to collect data from participants"
          action={
            canManage ? (
              <Button asChild>
                <Link href={`/studies/${resolvedParams?.id}/forms/new`}>
                  <Plus className="mr-2 h-4 w-4" />
                  Create Your First Form
                </Link>
              </Button>
            ) : null
          }
        />
      ) : (
        <div className="space-y-6">
          <div className="flex items-center gap-4">
            <div className="relative flex-1 max-w-sm">
              <Search className="absolute left-3 top-1/2 h-4 w-4 -translate-y-1/2 text-muted-foreground" />
              <Input
                placeholder="Search forms..."
                value={search}
                onChange={(e) => setSearch(e.target.value)}
                className="pl-10"
              />
            </div>
          </div>

          <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
            {forms.map((form) => {
              const TypeIcon = formTypeIcons[form.type as keyof typeof formTypeIcons] || FileText;
              const typeColor = formTypeColors[form.type as keyof typeof formTypeColors] || "bg-gray-100";
              const isActive = form.active;

              return (
                <Card key={form.id} className="overflow-hidden">
                  <CardHeader className="pb-3">
                    <div className="flex items-start justify-between">
                      <div className="flex items-center gap-2">
                        <div className={`rounded-md p-2 ${typeColor}`}>
                          <TypeIcon className="h-4 w-4" />
                        </div>
                        <div>
                          <CardTitle className="text-base">{form.title}</CardTitle>
                          <p className="text-muted-foreground text-xs capitalize">
                            {form.type}
                          </p>
                        </div>
                      </div>
                      {isActive && (
                        <Badge variant="default" className="text-xs">
                          Active
                        </Badge>
                      )}
                    </div>
                  </CardHeader>
                  <CardContent className="pb-3">
                    {form.description && (
                      <p className="text-muted-foreground text-sm line-clamp-2 mb-3">
                        {form.description}
                      </p>
                    )}
                    <div className="flex items-center justify-between text-xs text-muted-foreground">
                      <span>v{form.version}</span>
                      <span>{(form as any)._count?.responses ?? 0} responses</span>
                    </div>
                  </CardContent>
                  <div className="flex items-center justify-between border-t bg-muted/30 px-4 py-2">
                    <Button asChild variant="ghost" size="sm">
                      <Link href={`/studies/${resolvedParams?.id}/forms/${form.id}`}>
                        <Eye className="mr-1 h-3 w-3" />
                        View
                      </Link>
                    </Button>
                    {canManage && (
                      <DropdownMenu>
                        <DropdownMenuTrigger asChild>
                          <Button variant="ghost" size="sm">
                            <MoreHorizontal className="h-4 w-4" />
                          </Button>
                        </DropdownMenuTrigger>
                        <DropdownMenuContent align="end">
                          <DropdownMenuItem asChild>
                            <Link href={`/studies/${resolvedParams?.id}/forms/${form.id}/edit`}>
                              <Pencil className="mr-2 h-4 w-4" />
                              Edit
                            </Link>
                          </DropdownMenuItem>
                          {!isActive && (
                            <DropdownMenuItem
                              onClick={() => setActiveMutation.mutate({ id: form.id })}
                            >
                              <CheckCircle className="mr-2 h-4 w-4" />
                              Set Active
                            </DropdownMenuItem>
                          )}
                          <DropdownMenuItem
                            onClick={() => {
                              if (confirm("Are you sure you want to delete this form?")) {
                                deleteMutation.mutate({ id: form.id });
                              }
                            }}
                            className="text-destructive"
                          >
                            <Trash2 className="mr-2 h-4 w-4" />
                            Delete
                          </DropdownMenuItem>
                        </DropdownMenuContent>
                      </DropdownMenu>
                    )}
                  </div>
                </Card>
              );
            })}
          </div>
        </div>
      )}
    </EntityView>
  );
}