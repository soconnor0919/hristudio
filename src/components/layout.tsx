import { PropsWithChildren } from "react";
import { Sidebar } from "~/components/sidebar";
import { StudyHeader } from "~/components/study/StudyHeader";
import { Toaster } from "~/components/ui/toaster";

interface LayoutProps {
  pageTitle: string;
}

const Layout = ({ children, pageTitle }: PropsWithChildren<LayoutProps>) => {
  return (
    <div className="flex h-screen">
      <Sidebar />
      <main className="flex-1 overflow-y-auto bg-gradient-to-b from-[hsl(var(--gradient-start))] to-[hsl(var(--gradient-end))]">
        <div className="container mx-auto space-y-4 p-4 pt-16 lg:pt-4">
          <StudyHeader pageTitle={pageTitle} />
          {children}
          <Toaster />
        </div>
      </main>
    </div>
  );
};

export default Layout;