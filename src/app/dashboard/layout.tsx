import { Sidebar } from "~/components/sidebar";
import { Breadcrumb } from "~/components/breadcrumb";
import { ActiveStudyProvider } from "~/context/active-study";
import { StudyProvider } from "~/context/StudyContext";

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <ActiveStudyProvider>
      <StudyProvider>
        <div className="flex h-screen">
          <Sidebar />
          <div className="flex-1 flex flex-col min-h-0">
            <main className="flex-1 overflow-y-auto p-6">
              <Breadcrumb />
              {children}
            </main>
          </div>
        </div>
      </StudyProvider>
    </ActiveStudyProvider>
  );
}
