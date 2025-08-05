"use client";

import * as LucideIcons from "lucide-react";
import { type ReactNode } from "react";
import { Badge } from "~/components/ui/badge";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

type IconName = keyof typeof LucideIcons;

function getIcon(iconName: IconName) {
  const Icon = LucideIcons[iconName] as React.ComponentType<{
    className?: string;
  }>;
  return Icon;
}

interface EntityViewHeaderProps {
  title: string;
  subtitle?: string;
  icon: IconName;
  status?: {
    label: string;
    variant: "default" | "secondary" | "destructive" | "outline";
    icon?: IconName;
  };
  actions?: ReactNode;
}

interface EntityViewSectionProps {
  title: string;
  icon: IconName;
  description?: string;
  actions?: ReactNode;
  children: ReactNode;
}

interface EntityViewSidebarProps {
  children: ReactNode;
}

interface EntityViewProps {
  children: ReactNode;
}

export function EntityViewHeader({
  title,
  subtitle,
  icon,
  status,
  actions,
}: EntityViewHeaderProps) {
  const Icon = getIcon(icon);
  const StatusIcon = status?.icon ? getIcon(status.icon) : null;

  return (
    <div className="flex items-center justify-between">
      <div className="flex items-center gap-4">
        <div className="bg-primary text-primary-foreground flex h-16 w-16 items-center justify-center rounded-lg">
          <Icon className="h-8 w-8" />
        </div>
        <div>
          <div className="flex items-center gap-3">
            <h1 className="text-3xl font-bold">{title}</h1>
            {status && (
              <Badge variant={status.variant}>
                {StatusIcon && <StatusIcon className="mr-1 h-3 w-3" />}
                {status.label}
              </Badge>
            )}
          </div>
          {subtitle && (
            <p className="text-muted-foreground text-lg">{subtitle}</p>
          )}
        </div>
      </div>
      {actions && <div className="flex items-center gap-2">{actions}</div>}
    </div>
  );
}

export function EntityViewSection({
  title,
  icon,
  description,
  actions,
  children,
}: EntityViewSectionProps) {
  const Icon = getIcon(icon);

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="flex items-center gap-2">
            <Icon className="h-5 w-5" />
            <span>{title}</span>
          </CardTitle>
          {actions && actions}
        </div>
        {description && <CardDescription>{description}</CardDescription>}
      </CardHeader>
      <CardContent>{children}</CardContent>
    </Card>
  );
}

export function EntityViewSidebar({ children }: EntityViewSidebarProps) {
  return <div className="space-y-6">{children}</div>;
}

export function EntityView({ children }: EntityViewProps) {
  return <div className="space-y-6">{children}</div>;
}

// Utility component for empty states
interface EmptyStateProps {
  icon: IconName;
  title: string;
  description: string;
  action?: ReactNode;
}

export function EmptyState({
  icon,
  title,
  description,
  action,
}: EmptyStateProps) {
  const Icon = getIcon(icon);

  return (
    <div className="py-8 text-center">
      <Icon className="text-muted-foreground mx-auto mb-4 h-12 w-12" />
      <h3 className="mb-2 font-medium">{title}</h3>
      <p className="text-muted-foreground mb-4 text-sm">{description}</p>
      {action && action}
    </div>
  );
}

// Utility component for key-value information display
interface InfoGridProps {
  items: Array<{
    label: string;
    value: ReactNode;
    fullWidth?: boolean;
  }>;
  columns?: 1 | 2 | 3;
}

export function InfoGrid({ items, columns = 2 }: InfoGridProps) {
  return (
    <div
      className={`grid gap-4 ${
        columns === 1
          ? "grid-cols-1"
          : columns === 2
            ? "md:grid-cols-2"
            : "md:grid-cols-2 lg:grid-cols-3"
      }`}
    >
      {items.map((item, index) => (
        <div
          key={index}
          className={item.fullWidth ? "md:col-span-full" : undefined}
        >
          <h4 className="text-muted-foreground text-sm font-medium">
            {item.label}
          </h4>
          <div className="text-sm">{item.value}</div>
        </div>
      ))}
    </div>
  );
}

// Utility component for statistics display
interface StatsGridProps {
  stats: Array<{
    label: string;
    value: string | number;
    color?: "default" | "success" | "warning" | "error";
  }>;
}

export function StatsGrid({ stats }: StatsGridProps) {
  const getValueColor = (color?: string) => {
    switch (color) {
      case "success":
        return "text-green-600";
      case "warning":
        return "text-amber-600";
      case "error":
        return "text-red-600";
      default:
        return "font-medium";
    }
  };

  return (
    <div className="space-y-3">
      {stats.map((stat, index) => (
        <div key={index} className="flex justify-between">
          <span className="text-muted-foreground text-sm">{stat.label}:</span>
          <span className={getValueColor(stat.color)}>{stat.value}</span>
        </div>
      ))}
    </div>
  );
}

// Utility component for quick actions
interface QuickActionsProps {
  actions: Array<{
    label: string;
    icon: IconName;
    href?: string;
    onClick?: () => void;
    variant?: "default" | "outline" | "secondary" | "destructive";
  }>;
}

export function QuickActions({ actions }: QuickActionsProps) {
  return (
    <div className="space-y-2">
      {actions.map((action, index) => {
        const ActionIcon = getIcon(action.icon);

        return (
          <Button
            key={index}
            variant={action.variant ?? "outline"}
            className="w-full justify-start"
            asChild={!!action.href}
            onClick={action.onClick}
          >
            {action.href ? (
              <a href={action.href}>
                <ActionIcon className="mr-2 h-4 w-4" />
                {action.label}
              </a>
            ) : (
              <>
                <ActionIcon className="mr-2 h-4 w-4" />
                {action.label}
              </>
            )}
          </Button>
        );
      })}
    </div>
  );
}
