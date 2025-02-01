"use client"

import { usePathname } from "next/navigation"
import Link from "next/link"

export function BreadcrumbNav() {
  const pathname = usePathname()

  // Get breadcrumb items based on pathname
  const getBreadcrumbs = () => {
    const paths = pathname.split("/").filter(Boolean)
    const items = []

    if (paths[0] === "dashboard") {
      items.push({
        label: "Dashboard",
        href: "/dashboard",
        current: paths.length === 1,
      })
    }

    if (paths[1] === "studies") {
      items.push({
        label: "Studies",
        href: "/dashboard/studies",
        current: paths.length === 2,
      })

      if (paths[2] === "new") {
        items.push({
          label: "Create Study",
          current: true,
        })
      }
    }

    return items
  }

  const breadcrumbs = getBreadcrumbs()

  return (
    <nav aria-label="breadcrumb">
      <ol className="flex items-center gap-2">
        {breadcrumbs.map((item, index) => (
          <li key={item.label} className="flex items-center">
            {index > 0 && (
              <span role="presentation" aria-hidden="true" className="mx-2 text-muted-foreground">
                /
              </span>
            )}
            {item.current ? (
              <span className="font-medium">{item.label}</span>
            ) : (
              <Link href={item.href} className="text-muted-foreground hover:text-foreground">
                {item.label}
              </Link>
            )}
          </li>
        ))}
      </ol>
    </nav>
  )
} 