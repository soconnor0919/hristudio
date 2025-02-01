import { BotIcon } from "lucide-react"
import Link from "next/link"
import { cn } from "~/lib/utils"

interface LogoProps {
  href?: string
  className?: string
  iconClassName?: string
  textClassName?: string
}

export function Logo({ 
  href = "/", 
  className,
  iconClassName,
  textClassName
}: LogoProps) {
  return (
    <Link 
      href={href} 
      className={cn(
        "flex items-center font-sans text-xl",
        className
      )}
    >
      <BotIcon className={cn(
        "h-6 w-6 mr-1 text-muted-foreground",
        iconClassName
      )} />
      <span className={cn(textClassName)}>
        <span className="font-extrabold">HRI</span>
        <span className="font-normal">Studio</span>
      </span>
    </Link>
  )
} 