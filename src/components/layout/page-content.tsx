import * as React from "react"
import { cn } from "~/lib/utils"

interface PageContentProps
  extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode
}

export function PageContent({
  children,
  className,
  ...props
}: PageContentProps) {
  return (
    <div
      className={cn(
        "grid gap-6",
        className,
      )}
      {...props}
    >
      {children}
    </div>
  )
} 