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

const textSizes = {
  sm: "text-sm",
  md: "text-base",
  lg: "text-3xl",
};

export function Logo({
  className,
  iconSize = "md",
  showText = true,
}: LogoProps) {
  return (
    <div className={cn("flex items-center gap-2", className)}>
      <div className="bg-primary text-primary-foreground flex aspect-square items-center justify-center rounded-lg p-1">
        <Bot className={iconSizes[iconSize]} />
      </div>
      {showText && (
        <div className="grid flex-1 text-left leading-none">
          <div className="flex items-baseline gap-0">
            <span className={cn(textSizes[iconSize], "font-extrabold tracking-tight")}>HRI</span>
            <span className={cn(textSizes[iconSize], "font-normal tracking-tight")}>Studio</span>
          </div>
        </div>
      )}
    </div>
  );
}
