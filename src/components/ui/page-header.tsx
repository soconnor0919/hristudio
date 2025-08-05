import { type ReactNode } from "react";
import { type LucideIcon } from "lucide-react";
import { cn } from "~/lib/utils";
import { Button } from "~/components/ui/button";
import { Badge } from "~/components/ui/badge";

interface PageHeaderProps {
  title: string;
  description?: string;
  icon?: LucideIcon;
  iconClassName?: string;
  badges?: Array<{
    label: string;
    variant?: "default" | "secondary" | "destructive" | "outline";
    className?: string;
  }>;
  actions?: ReactNode;
  className?: string;
}

export function PageHeader({
  title,
  description,
  icon: Icon,
  iconClassName,
  badges,
  actions,
  className,
}: PageHeaderProps) {
  return (
    <div className={cn("flex items-start justify-between", className)}>
      <div className="flex items-start space-x-4">
        {/* Icon */}
        {Icon && (
          <div
            className={cn(
              "bg-primary/10 flex h-12 w-12 items-center justify-center rounded-lg",
              iconClassName,
            )}
          >
            <Icon className="text-primary h-6 w-6" />
          </div>
        )}

        {/* Title and description */}
        <div className="min-w-0 flex-1">
          <div className="flex items-center space-x-3">
            <h1 className="text-foreground text-3xl font-bold tracking-tight">
              {title}
            </h1>
            {/* Badges */}
            {badges && badges.length > 0 && (
              <div className="flex space-x-2">
                {badges.map((badge, index) => (
                  <Badge
                    key={index}
                    variant={badge.variant}
                    className={badge.className}
                  >
                    {badge.label}
                  </Badge>
                ))}
              </div>
            )}
          </div>
          {description && (
            <p className="text-muted-foreground mt-2 text-base">
              {description}
            </p>
          )}
        </div>
      </div>

      {/* Actions */}
      {actions && <div className="flex-shrink-0">{actions}</div>}
    </div>
  );
}

// Quick action button helper
interface ActionButtonProps {
  children: ReactNode;
  href?: string;
  onClick?: () => void;
  variant?: "default" | "secondary" | "outline" | "destructive" | "ghost" | "link";
  size?: "default" | "sm" | "lg" | "icon";
  disabled?: boolean;
  className?: string;
}

export function ActionButton({
  children,
  href,
  onClick,
  variant = "default",
  size = "default",
  disabled,
  className,
}: ActionButtonProps) {
  if (href) {
    return (
      <Button asChild variant={variant} size={size} className={className}>
        <a href={href}>{children}</a>
      </Button>
    );
  }

  return (
    <Button
      onClick={onClick}
      variant={variant}
      size={size}
      disabled={disabled}
      className={className}
    >
      {children}
    </Button>
  );
}
