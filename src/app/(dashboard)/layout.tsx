import { auth } from "~/server/auth";
import { redirect } from "next/navigation";
import Link from "next/link";
import { Button } from "~/components/ui/button";
import { Separator } from "~/components/ui/separator";
import {
  Users,
  FlaskConical,
  Play,
  BarChart3,
  Settings,
  User,
  LogOut,
  Home,
  UserCog,
} from "lucide-react";

interface DashboardLayoutProps {
  children: React.ReactNode;
}

const navigationItems = [
  {
    label: "Studies",
    href: "/studies",
    icon: FlaskConical,
    roles: ["administrator", "researcher", "wizard", "observer"],
  },
  {
    label: "Experiments",
    href: "/experiments",
    icon: Settings,
    roles: ["administrator", "researcher"],
  },
  {
    label: "Trials",
    href: "/trials",
    icon: Play,
    roles: ["administrator", "researcher", "wizard"],
  },
  {
    label: "Analytics",
    href: "/analytics",
    icon: BarChart3,
    roles: ["administrator", "researcher"],
  },
  {
    label: "Participants",
    href: "/participants",
    icon: Users,
    roles: ["administrator", "researcher"],
  },
];

const adminItems = [
  {
    label: "Administration",
    href: "/admin",
    icon: UserCog,
    roles: ["administrator"],
  },
];

export default async function DashboardLayout({
  children,
}: DashboardLayoutProps) {
  const session = await auth();

  if (!session?.user) {
    redirect("/auth/signin");
  }

  const userRole = session.user.roles[0]?.role || "observer";
  const userName = session.user.name || session.user.email;

  // Filter navigation items based on user role
  const allowedNavItems = navigationItems.filter((item) =>
    item.roles.includes(userRole),
  );
  const allowedAdminItems = adminItems.filter((item) =>
    item.roles.includes(userRole),
  );

  return (
    <div className="min-h-screen bg-slate-50">
      {/* Sidebar */}
      <div className="fixed inset-y-0 left-0 z-50 w-64 border-r border-slate-200 bg-white">
        {/* Header */}
        <div className="flex h-16 items-center border-b border-slate-200 px-6">
          <Link href="/" className="flex items-center space-x-2">
            <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-blue-600">
              <FlaskConical className="h-5 w-5 text-white" />
            </div>
            <span className="text-xl font-bold text-slate-900">HRIStudio</span>
          </Link>
        </div>

        {/* Navigation */}
        <div className="flex h-full flex-col">
          <nav className="flex-1 space-y-2 px-4 py-6">
            {/* Main Navigation */}
            <div className="space-y-1">
              {allowedNavItems.map((item) => (
                <Link
                  key={item.href}
                  href={item.href}
                  className="flex items-center space-x-3 rounded-lg px-3 py-2 text-sm font-medium text-slate-700 transition-colors hover:bg-slate-100 hover:text-slate-900"
                >
                  <item.icon className="h-5 w-5" />
                  <span>{item.label}</span>
                </Link>
              ))}
            </div>

            {/* Admin Section */}
            {allowedAdminItems.length > 0 && (
              <>
                <Separator className="my-4" />
                <div className="space-y-1">
                  <h3 className="px-3 text-xs font-semibold tracking-wider text-slate-500 uppercase">
                    Administration
                  </h3>
                  {allowedAdminItems.map((item) => (
                    <Link
                      key={item.href}
                      href={item.href}
                      className="flex items-center space-x-3 rounded-lg px-3 py-2 text-sm font-medium text-slate-700 transition-colors hover:bg-slate-100 hover:text-slate-900"
                    >
                      <item.icon className="h-5 w-5" />
                      <span>{item.label}</span>
                    </Link>
                  ))}
                </div>
              </>
            )}
          </nav>

          {/* User Section */}
          <div className="border-t border-slate-200 p-4">
            <div className="mb-3 flex items-center space-x-3">
              <div className="flex h-8 w-8 items-center justify-center rounded-full bg-blue-100">
                <User className="h-4 w-4 text-blue-600" />
              </div>
              <div className="min-w-0 flex-1">
                <p className="truncate text-sm font-medium text-slate-900">
                  {userName}
                </p>
                <p className="text-xs text-slate-500 capitalize">{userRole}</p>
              </div>
            </div>

            <div className="space-y-1">
              <Link
                href="/profile"
                className="flex w-full items-center space-x-3 rounded-lg px-3 py-2 text-sm font-medium text-slate-700 transition-colors hover:bg-slate-100 hover:text-slate-900"
              >
                <Settings className="h-4 w-4" />
                <span>Profile</span>
              </Link>

              <Link
                href="/"
                className="flex w-full items-center space-x-3 rounded-lg px-3 py-2 text-sm font-medium text-slate-700 transition-colors hover:bg-slate-100 hover:text-slate-900"
              >
                <Home className="h-4 w-4" />
                <span>Home</span>
              </Link>

              <Link
                href="/auth/signout"
                className="flex w-full items-center space-x-3 rounded-lg px-3 py-2 text-sm font-medium text-red-600 transition-colors hover:bg-red-50 hover:text-red-700"
              >
                <LogOut className="h-4 w-4" />
                <span>Sign Out</span>
              </Link>
            </div>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="pl-64">
        <main className="min-h-screen">{children}</main>
      </div>
    </div>
  );
}
