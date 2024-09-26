import { PropsWithChildren } from "react";
import { Sidebar } from "~/components/sidebar";
import { StudyHeader } from "~/components/study/StudyHeader";

const Layout = ({ children }: PropsWithChildren) => {
  return (
    <div className="flex h-screen bg-gradient-to-b from-blue-100 to-white">
      <Sidebar />
      <main className="flex-1 overflow-y-auto p-4 pt-16 lg:pt-4">
        <div className="container mx-auto space-y-4">
          <StudyHeader />
          {children}
        </div>
      </main>
    </div>
  );
};

export default Layout;