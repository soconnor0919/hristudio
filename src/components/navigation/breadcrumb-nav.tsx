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
      } else if (paths[2]) {
        items.push({
          label: "Study Details",
          href: `/dashboard/studies/${paths[2]}`,
          current: paths.length === 3,
        })

        if (paths[3] === "participants") {
          items.push({
            label: "Participants",
            href: `/dashboard/studies/${paths[2]}/participants`,
            current: paths.length === 4,
          })

          if (paths[4] === "new") {
            items.push({
              label: "Add Participant",
              current: true,
            })
          } else if (paths[4]) {
            items.push({
              label: "Participant Details",
              current: paths.length === 5,
            })

            if (paths[5] === "edit") {
              items.push({
                label: "Edit",
                current: true,
              })
            }
          }
        }
      }
    }

    return items
  }

  const breadcrumbs = getBreadcrumbs()

  return (
    <nav aria-label="breadcrumb">
      <ol className="flex items-center">
        {breadcrumbs.map((item, index) => (
          <li key={item.label} className="flex items-center">
            {index > 0 && (
              <span role="presentation" aria-hidden="true" className="mx-1 text-muted-foreground">
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