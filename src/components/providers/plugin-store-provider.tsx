"use client";

import { createContext, useContext, useState } from "react";
import { type RobotPlugin } from "~/lib/plugin-store/types";

interface PluginStoreContextType {
  plugins: RobotPlugin[];
  selectedPlugin?: RobotPlugin;
  selectPlugin: (robotId: string) => void;
  setPlugins: (plugins: RobotPlugin[]) => void;
}

const PluginStoreContext = createContext<PluginStoreContextType | undefined>(undefined);

export function PluginStoreProvider({ 
  children,
  initialPlugins = [],
}: { 
  children: React.ReactNode;
  initialPlugins?: RobotPlugin[];
}) {
  const [plugins, setPlugins] = useState<RobotPlugin[]>(initialPlugins);
  const [selectedPlugin, setSelectedPlugin] = useState<RobotPlugin>();

  const selectPlugin = (robotId: string) => {
    const plugin = plugins.find(p => p.robotId === robotId);
    setSelectedPlugin(plugin);
  };

  return (
    <PluginStoreContext.Provider
      value={{
        plugins,
        selectedPlugin,
        selectPlugin,
        setPlugins,
      }}
    >
      {children}
    </PluginStoreContext.Provider>
  );
}

export function usePluginStore() {
  const context = useContext(PluginStoreContext);
  if (!context) {
    throw new Error("usePluginStore must be used within a PluginStoreProvider");
  }
  return context;
} 