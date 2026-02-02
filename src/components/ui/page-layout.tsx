import { type ReactNode } from "react";
import { type LucideIcon } from "lucide-react";
import { cn } from "~/lib/utils";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";
import type { BreadcrumbItem } from "~/components/ui/breadcrumb";

interface BreadcrumbItem {
  label: string;
  href?: string;
}

interface QuickAction {
  title: string;
  description: string;
  icon: LucideIcon;
  href: string;
  variant?:
    | "default"
    | "primary"
    | "secondary"
    | "outline"
    | "destructive"
    | "ghost"
    | "link";
}

interface CreateButton {
  label: string;
  href: string;
}

interface Stat {
  title: string;
  value: string | number;
  description?: string;
  icon?: LucideIcon;
  trend?: {
    value: number;
    isPositive: boolean;
  };
}

interface Alert {
  type: "info" | "warning" | "error" | "success";
  message: string;
}

interface Activity {
  id: string;
  title: string;
  description: string;
  time: string;
  type: string;
}

interface PageLayoutProps {
  children?: ReactNode;
  className?: string;
  title?: string;
  description?: string;
  userName?: string;
  userRole?: string;
  breadcrumb?: BreadcrumbItem[];
  createButton?: CreateButton;
  quickActions?: QuickAction[];
  stats?: Stat[];
  alerts?: Alert[];
  recentActivity?: Activity[] | ReactNode | null;
}

export function PageLayout({
  children,
  className,
  title,
  description,
  userName: _userName,
  userRole: _userRole,
  breadcrumb: _breadcrumb,
  createButton,
  quickActions,
  stats,
  alerts,
  recentActivity,
}: PageLayoutProps) {
  return (
    <div className={cn("space-y-6", className)}>
      {/* Header */}
      {title && (
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold tracking-tight">{title}</h1>
            {description && (
              <p className="text-muted-foreground">{description}</p>
            )}
          </div>
          {createButton && (
            <Button asChild>
              <a href={createButton.href}>{createButton.label}</a>
            </Button>
          )}
        </div>
      )}

      {/* Alerts */}
      {alerts && alerts.length > 0 && (
        <div className="space-y-2">
          {alerts.map((alert, index) => (
            <div
              key={index}
              className={cn(
                "rounded-lg border p-4",
                alert.type === "error" &&
                  "border-red-200 bg-red-50 text-red-800",
                alert.type === "warning" &&
                  "border-yellow-200 bg-yellow-50 text-yellow-800",
                alert.type === "info" &&
                  "border-blue-200 bg-blue-50 text-blue-800",
                alert.type === "success" &&
                  "border-green-200 bg-green-50 text-green-800",
              )}
            >
              {alert.message}
            </div>
          ))}
        </div>
      )}

      {/* Stats */}
      {stats && stats.length > 0 && (
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
          {stats.map((stat, index) => (
            <div key={index} className="bg-card rounded-lg border p-6">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-muted-foreground text-sm font-medium">
                    {stat.title}
                  </p>
                  <p className="text-3xl font-bold">{stat.value}</p>
                  {stat.description && (
                    <p className="text-muted-foreground text-sm">
                      {stat.description}
                    </p>
                  )}
                </div>
                {stat.icon && (
                  <stat.icon className="text-muted-foreground h-8 w-8" />
                )}
              </div>
              {stat.trend && (
                <div className="mt-2">
                  <Badge
                    variant={stat.trend.isPositive ? "default" : "destructive"}
                  >
                    {stat.trend.isPositive ? "+" : ""}
                    {stat.trend.value}%
                  </Badge>
                </div>
              )}
            </div>
          ))}
        </div>
      )}

      {/* Quick Actions */}
      {quickActions && quickActions.length > 0 && (
        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-4">
          {quickActions.map((action, index) => (
            <Button
              key={index}
              asChild
              variant={
                action.variant === "primary"
                  ? "default"
                  : (action.variant ?? "default")
              }
              className="h-auto flex-col gap-2 p-4"
            >
              <a href={action.href}>
                <action.icon className="h-6 w-6" />
                <div className="text-center">
                  <div className="font-medium">{action.title}</div>
                  <div className="text-muted-foreground text-sm">
                    {action.description}
                  </div>
                </div>
              </a>
            </Button>
          ))}
        </div>
      )}

      {/* Recent Activity */}
      {recentActivity && (
        <div>
          {Array.isArray(recentActivity) && recentActivity.length > 0 ? (
            <div className="bg-card rounded-lg border p-6">
              <h3 className="mb-4 text-lg font-medium">Recent Activity</h3>
              <div className="space-y-4">
                {recentActivity.map((activity) => (
                  <div key={activity.id} className="flex items-start space-x-4">
                    <div className="flex-1">
                      <p className="text-sm font-medium">{activity.title}</p>
                      <p className="text-muted-foreground text-sm">
                        {activity.description}
                      </p>
                    </div>
                    <p className="text-muted-foreground text-sm">
                      {activity.time}
                    </p>
                  </div>
                ))}
              </div>
            </div>
          ) : (
            (recentActivity as ReactNode)
          )}
        </div>
      )}

      {/* Main Content */}
      {children && <div className="space-y-4">{children}</div>}
    </div>
  );
}

// Legacy component names for compatibility
export const ManagementPageLayout = PageLayout;
export const DashboardOverviewLayout = PageLayout;
export const DetailPageLayout = PageLayout;
export const FormPageLayout = PageLayout;

// Simple components for basic usage
interface PageHeaderProps {
  title: string;
  description?: string;
  icon?: LucideIcon;
  actions?: ReactNode;
  className?: string;
}

export function PageHeader({
  title,
  description,
  icon: Icon,
  actions,
  className,
}: PageHeaderProps) {
  return (
    <div className={cn("flex items-center justify-between", className)}>
      <div className="flex items-center gap-3">
        {Icon && (
          <div className="bg-primary/10 flex h-12 w-12 items-center justify-center rounded-lg">
            <Icon className="text-primary h-6 w-6" />
          </div>
        )}
        <div>
          <h1 className="text-3xl font-bold tracking-tight">{title}</h1>
          {description && (
            <p className="text-muted-foreground">{description}</p>
          )}
        </div>
      </div>
      {actions && <div className="flex items-center gap-2">{actions}</div>}
    </div>
  );
}

interface PageContentProps {
  children: ReactNode;
  className?: string;
}

export function PageContent({ children, className }: PageContentProps) {
  return <div className={cn("space-y-4", className)}>{children}</div>;
}
