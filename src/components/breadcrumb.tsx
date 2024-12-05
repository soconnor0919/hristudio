'use client';

import { ChevronRight } from "lucide-react";
import Link from "next/link";
import { usePathname } from "next/navigation";
import { useActiveStudy } from "~/context/active-study";

interface BreadcrumbItem {
  label: string;
  href?: string;
}

export function Breadcrumb() {
  const pathname = usePathname();
  const { activeStudy } = useActiveStudy();

  const getBreadcrumbs = (): BreadcrumbItem[] => {
    const items: BreadcrumbItem[] = [{ label: 'Dashboard', href: '/dashboard' }];
    const path = pathname.split('/').filter(Boolean);

    // Handle root dashboard
    if (path.length === 1 && path[0] === 'dashboard') {
      return items;
    }

    // Handle studies list page
    if (path[1] === 'studies') {
      items.push({ label: 'Studies', href: '/dashboard/studies' });
      
      if (path[2] === 'new') {
        items.push({ label: 'New Study' });
        return items;
      }

      if (!activeStudy) {
        return items;
      }

      // Handle active study pages
      items.push({ 
        label: activeStudy.title,
        href: `/dashboard/studies/${activeStudy.id}`
      });

      // Add section based on URL
      if (path.length > 3) {
        const section = path[3];
        const sectionLabel = section.charAt(0).toUpperCase() + section.slice(1);
        
        if (section === 'new') {
          items.push({ 
            label: `New ${path[2].slice(0, -1)}`,
            href: `/dashboard/studies/${activeStudy.id}/${path[2]}/new`
          });
        } else {
          items.push({ 
            label: sectionLabel,
            href: `/dashboard/studies/${activeStudy.id}/${section}`
          });
        }
      }
      return items;
    }

    // Handle participants page
    if (path[1] === 'participants') {
      items.push({ label: 'Participants', href: '/dashboard/participants' });
      return items;
    }

    // Handle settings page
    if (path[1] === 'settings') {
      items.push({ label: 'Settings', href: '/dashboard/settings' });
      return items;
    }

    // Handle profile page
    if (path[1] === 'profile') {
      items.push({ label: 'Profile', href: '/dashboard/profile' });
      return items;
    }

    return items;
  };

  const breadcrumbs = getBreadcrumbs();

  // Always show breadcrumbs on dashboard pages
  if (breadcrumbs.length <= 1 && !pathname.startsWith('/dashboard')) return null;

  return (
    <div className="flex items-center space-x-2 text-sm text-muted-foreground mb-6">
      {breadcrumbs.map((item, index) => {
        const isLast = index === breadcrumbs.length - 1;

        return (
          <div key={item.label} className="flex items-center">
            {index > 0 && <ChevronRight className="h-4 w-4 mx-2" />}
            {item.href && !isLast ? (
              <Link
                href={item.href}
                className="hover:text-foreground transition-colors"
              >
                {item.label}
              </Link>
            ) : (
              <span className={isLast ? "text-foreground font-medium" : ""}>
                {item.label}
              </span>
            )}
          </div>
        );
      })}
    </div>
  );
} 