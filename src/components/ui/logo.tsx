import { Bot } from "lucide-react";
import { cn } from "~/lib/utils";

interface LogoProps {
  className?: string;
  iconSize?: "sm" | "md" | "lg";
  showText?: boolean;
}

const iconSizes = {
  sm: "h-4 w-4",
  md: "h-6 w-6",
  lg: "h-8 w-8",
};

export function Logo({
  className,
  iconSize = "md",
  showText = true,
}: LogoProps) {
  return (
    <div className={cn("flex items-center gap-2", className)}>
      <div className="bg-sidebar-primary text-sidebar-primary-foreground flex aspect-square items-center justify-center rounded-lg p-1">
        <Bot className={iconSizes[iconSize]} />
      </div>
      {showText && (
        <div className="grid flex-1 text-left text-sm leading-tight">
          <div className="flex items-baseline gap-0">
            <span className="text-base font-extrabold tracking-tight">HRI</span>
            <span className="text-base font-normal tracking-tight">Studio</span>
          </div>
          <span className="text-muted-foreground truncate text-xs">
            Research Platform
          </span>
        </div>
      )}
    </div>
  );
}
