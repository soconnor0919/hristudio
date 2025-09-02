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
  breadcrumbs?: ReactNode;
  actions?: ReactNode;
  className?: string;
}

export function PageHeader({
  title,
  description,
  icon: Icon,
  iconClassName,
  badges,
  breadcrumbs,
  actions,
  className,
}: PageHeaderProps) {
  return (
    <div
      className={cn(
        "flex min-w-0 items-start justify-between gap-2 md:gap-4",
        className,
      )}
    >
      <div className="flex min-w-0 items-start gap-3 md:gap-4">
        {/* Icon */}
        {Icon && (
          <div
            className={cn(
              "bg-primary/10 flex h-10 w-10 items-center justify-center rounded-lg md:h-12 md:w-12",
              iconClassName,
            )}
          >
            <Icon className="text-primary h-5 w-5 md:h-6 md:w-6" />
          </div>
        )}

        {/* Title and description */}
        <div className="min-w-0 flex-1">
          {breadcrumbs && (
            <div className="text-muted-foreground/80 mb-1 truncate text-xs md:text-sm">
              {breadcrumbs}
            </div>
          )}
          <div className="flex min-w-0 items-center gap-2 md:gap-3">
            <h1 className="text-foreground truncate text-2xl font-bold tracking-tight md:text-3xl">
              {title}
            </h1>
            {/* Badges */}
            {badges && badges.length > 0 && (
              <div className="hidden flex-shrink-0 items-center gap-2 sm:flex">
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
            <p className="text-muted-foreground mt-1.5 line-clamp-2 text-sm md:mt-2 md:text-base">
              {description}
            </p>
          )}
        </div>
      </div>

      {/* Actions */}
      {actions && (
        <div className="flex flex-shrink-0 items-center gap-2">{actions}</div>
      )}
    </div>
  );
}

// Quick action button helper
interface ActionButtonProps {
  children: ReactNode;
  href?: string;
  onClick?: () => void;
  variant?:
    | "default"
    | "secondary"
    | "outline"
    | "destructive"
    | "ghost"
    | "link";
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
