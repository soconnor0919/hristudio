import { Sidebar } from "~/components/sidebar";
import { cn } from "~/lib/utils";
import { StudyProvider } from "~/context/StudyContext";

export default function DashboardLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <StudyProvider>
      <div className="flex h-screen overflow-hidden">
        <Sidebar />
        <main className={cn(
          "flex-1 overflow-y-auto",
          "lg:pt-8 p-8",
          "pt-[calc(3.5rem+2rem)]"
        )}>
          {children}
        </main>
      </div>
    </StudyProvider>
  );
}
