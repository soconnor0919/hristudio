import { redirect } from "next/navigation";
import { cookies } from "next/headers";
import {
  SidebarInset,
  SidebarProvider,
  SidebarTrigger,
} from "~/components/ui/sidebar";
import { Separator } from "~/components/ui/separator";
import { AppSidebar } from "~/components/dashboard/app-sidebar";
import { auth } from "~/server/auth";
import {
  BreadcrumbProvider,
  BreadcrumbDisplay,
} from "~/components/ui/breadcrumb-provider";
import { StudyProvider } from "~/lib/study-context";
import { TourProvider } from "~/components/onboarding/TourProvider";

interface DashboardLayoutProps {
  children: React.ReactNode;
}

export default async function DashboardLayout({
  children,
}: DashboardLayoutProps) {
  const session = await auth();

  if (!session?.user) {
    redirect("/auth/signin");
  }

  const userRole =
    typeof session.user.roles?.[0] === "string"
      ? session.user.roles[0]
      : (session.user.roles?.[0]?.role ?? "observer");

  const cookieStore = await cookies();
  const defaultOpen = cookieStore.get("sidebar_state")?.value === "true";

  // Pre-seed selected study from cookie (SSR) to avoid client flash
  const selectedStudyCookie =
    cookieStore.get("hristudio_selected_study")?.value ?? null;

  return (
    <StudyProvider initialStudyId={selectedStudyCookie}>
      <TourProvider>
        <BreadcrumbProvider>
          <SidebarProvider defaultOpen={defaultOpen}>
            <AppSidebar userRole={userRole} />
            <SidebarInset>
              <header className="flex h-16 shrink-0 items-center gap-2 transition-[width,height] ease-linear group-has-[[data-collapsible=icon]]/sidebar-wrapper:h-12">
                <div className="flex items-center gap-2 px-4">
                  <SidebarTrigger className="-ml-1" />
                  <Separator orientation="vertical" className="mr-2 h-4" />
                  <BreadcrumbDisplay />
                </div>
              </header>
              <div className="flex min-h-0 min-w-0 flex-1 flex-col gap-4 overflow-hidden p-4 pt-0">
                {children}
              </div>
            </SidebarInset>
          </SidebarProvider>
        </BreadcrumbProvider>
      </TourProvider>
    </StudyProvider>
  );
}
