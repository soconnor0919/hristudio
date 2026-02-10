"use client";

import { ArrowLeft, type LucideIcon } from "lucide-react";
import Link from "next/link";
import { useRouter } from "next/navigation";
import { type ReactNode } from "react";
import { type FieldValues, type UseFormReturn } from "react-hook-form";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { PageHeader } from "~/components/ui/page-header";
import { Separator } from "~/components/ui/separator";
import { cn } from "~/lib/utils";

interface EntityFormProps<T extends FieldValues = FieldValues> {
  // Mode
  mode: "create" | "edit";

  // Entity info
  entityName: string; // "Study", "Experiment", etc.
  entityNamePlural: string; // "Studies", "Experiments", etc.

  // Navigation
  backUrl: string;
  listUrl: string;

  // Header
  title: string;
  description: string;
  icon?: LucideIcon;

  // Form
  form: UseFormReturn<T>;
  onSubmit: (data: T) => Promise<void> | void;
  children: ReactNode; // Form fields

  // State
  isSubmitting?: boolean;
  error?: string | null;

  // Actions
  onDelete?: () => Promise<void> | void;
  isDeleting?: boolean;

  // Sidebar content
  sidebar?: ReactNode;

  // Custom submit button text
  submitText?: string;
  submitButtonId?: string;

  // Additional header actions
  extraActions?: ReactNode;

  // Layout
  layout?: "default" | "full-width";
  className?: string;
}

export function EntityForm<T extends FieldValues = FieldValues>({
  mode,
  entityName,
  entityNamePlural,
  backUrl,
  listUrl: _listUrl,
  title,
  description,
  icon: Icon,
  form,
  onSubmit,
  children,
  isSubmitting = false,
  error,
  onDelete,
  isDeleting = false,
  sidebar,
  submitText,
  submitButtonId,
  extraActions,
  layout = "default",
  className,
}: EntityFormProps<T>) {
  const router = useRouter();

  const handleSubmit = form.handleSubmit(async (data) => {
    await onSubmit(data);
  });

  const defaultSubmitText =
    mode === "create" ? `Create ${entityName}` : `Save Changes`;

  return (
    <div className={cn("space-y-6", className)}>
      {/* Header */}
      <PageHeader
        title={title}
        description={description}
        icon={Icon}
        actions={
          <div className="flex items-center space-x-2">
            {extraActions}
            <Button variant="outline" asChild>
              <Link href={backUrl}>
                <ArrowLeft className="mr-2 h-4 w-4" />
                Back to {entityNamePlural}
              </Link>
            </Button>
            {mode === "edit" && onDelete && (
              <Button
                variant="destructive"
                onClick={onDelete}
                disabled={isDeleting || isSubmitting}
              >
                {isDeleting ? "Deleting..." : "Delete"}
              </Button>
            )}
          </div>
        }
      />

      {/* Form Layout */}
      <div
        className={cn(
          "grid gap-8 w-full",
          // If sidebar exists, use 2-column layout. If not, use full width (max-w-7xl centered).
          sidebar && layout === "default"
            ? "grid-cols-1 lg:grid-cols-3"
            : "grid-cols-1 max-w-7xl mx-auto",
        )}
      >
        {/* Main Form */}
        <div className={sidebar && layout === "default" ? "lg:col-span-2" : "col-span-1"}>
          <Card>
            <CardHeader>
              <CardTitle>
                {mode === "create" ? `New ${entityName}` : `Edit ${entityName}`}
              </CardTitle>
              <CardDescription>
                {mode === "create"
                  ? `Fill in the details to create a new ${entityName.toLowerCase()}.`
                  : `Update the details for this ${entityName.toLowerCase()}.`}
              </CardDescription>
            </CardHeader>
            <CardContent>
              <form onSubmit={handleSubmit} className="space-y-6">
                {/* Form Fields */}
                {children}

                {/* Error Message */}
                {error && (
                  <div className="rounded-md bg-red-50 p-3">
                    <p className="text-sm text-red-800">{error}</p>
                  </div>
                )}

                {/* Form Actions */}
                <Separator />
                <div className="flex justify-end space-x-3">
                  <Button
                    type="button"
                    variant="outline"
                    onClick={() => router.back()}
                    disabled={isSubmitting || isDeleting}
                  >
                    Cancel
                  </Button>
                  <Button
                    id={submitButtonId}
                    type="submit"
                    disabled={
                      isSubmitting ||
                      isDeleting ||
                      (mode === "edit" && !form.formState.isDirty)
                    }
                    className="min-w-[140px]"
                  >
                    {isSubmitting ? (
                      <div className="flex items-center space-x-2">
                        <svg
                          className="h-4 w-4 animate-spin"
                          fill="none"
                          viewBox="0 0 24 24"
                        >
                          <circle
                            className="opacity-25"
                            cx="12"
                            cy="12"
                            r="10"
                            stroke="currentColor"
                            strokeWidth="4"
                          />
                          <path
                            className="opacity-75"
                            fill="currentColor"
                            d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                          />
                        </svg>
                        <span>
                          {mode === "create" ? "Creating..." : "Saving..."}
                        </span>
                      </div>
                    ) : (
                      (submitText ?? defaultSubmitText)
                    )}
                  </Button>
                </div>
              </form>
            </CardContent>
          </Card>
        </div>

        {/* Sidebar */}
        {sidebar && layout === "default" && (
          <div className="space-y-6">{sidebar}</div>
        )}
      </div>
    </div>
  );
}

// Form field components for consistency
interface FormFieldProps {
  children: ReactNode;
  className?: string;
}

export function FormField({ children, className }: FormFieldProps) {
  return <div className={cn("space-y-2", className)}>{children}</div>;
}

interface FormSectionProps {
  title: string;
  description?: string;
  children: ReactNode;
  className?: string;
}

export function FormSection({
  title,
  description,
  children,
  className,
}: FormSectionProps) {
  return (
    <div className={cn("space-y-4", className)}>
      <div>
        <h3 className="text-lg font-medium">{title}</h3>
        {description && (
          <p className="text-muted-foreground text-sm">{description}</p>
        )}
      </div>
      <div className="space-y-4">{children}</div>
    </div>
  );
}

// Sidebar components
interface SidebarCardProps {
  title: string;
  icon?: LucideIcon;
  children: ReactNode;
  className?: string;
}

export function SidebarCard({
  title,
  icon: Icon,
  children,
  className,
}: SidebarCardProps) {
  return (
    <Card className={className}>
      <CardHeader>
        <CardTitle className="flex items-center space-x-2">
          {Icon && <Icon className="h-5 w-5" />}
          <span>{title}</span>
        </CardTitle>
      </CardHeader>
      <CardContent>{children}</CardContent>
    </Card>
  );
}

interface NextStepsProps {
  steps: Array<{
    title: string;
    description: string;
    completed?: boolean;
  }>;
}

export function NextSteps({ steps }: NextStepsProps) {
  return (
    <SidebarCard title="What's Next?">
      <div className="space-y-3 text-sm">
        {steps.map((step, index) => (
          <div key={index} className="flex items-start space-x-3">
            <div
              className={cn(
                "mt-1 h-2 w-2 rounded-full",
                step.completed
                  ? "bg-green-600"
                  : index === 0
                    ? "bg-blue-600"
                    : "bg-slate-300",
              )}
            />
            <div>
              <p className="font-medium">{step.title}</p>
              <p className="text-muted-foreground">{step.description}</p>
            </div>
          </div>
        ))}
      </div>
    </SidebarCard>
  );
}

interface TipsProps {
  tips: string[];
}

export function Tips({ tips }: TipsProps) {
  return (
    <SidebarCard title="💡 Tips">
      <div className="text-muted-foreground space-y-3 text-sm">
        {tips.map((tip, index) => (
          <p key={index}>{tip}</p>
        ))}
      </div>
    </SidebarCard>
  );
}
