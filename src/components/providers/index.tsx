"use client";

import { ThemeProvider } from "next-themes";
import { StudyProvider } from "./study-provider";
import { PluginStoreProvider } from "./plugin-store-provider";
import { Toaster } from "~/components/ui/toaster";

export function Providers({ children }: { children: React.ReactNode }) {
  return (
    <ThemeProvider
      attribute="class"
      defaultTheme="system"
      enableSystem
      disableTransitionOnChange
    >
      <PluginStoreProvider>
        <StudyProvider>
          {children}
          <Toaster />
        </StudyProvider>
      </PluginStoreProvider>
    </ThemeProvider>
  );
} 