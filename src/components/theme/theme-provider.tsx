"use client";

import * as React from "react";

type Theme = "dark" | "light" | "system";

type ThemeProviderProps = {
  children: React.ReactNode;
  defaultTheme?: Theme;
  storageKey?: string;
  attribute?: string;
  enableSystem?: boolean;
  disableTransitionOnChange?: boolean;
};

type ThemeProviderState = {
  theme: Theme;
  setTheme: (theme: Theme) => void;
  resolvedTheme?: "dark" | "light";
};

const initialState: ThemeProviderState = {
  theme: "system",
  setTheme: () => null,
  resolvedTheme: "light",
};

const ThemeProviderContext =
  React.createContext<ThemeProviderState>(initialState);

export function ThemeProvider({
  children,
  defaultTheme = "system",
  storageKey = "hristudio-theme",
  attribute: _attribute = "class",
  enableSystem = true,
  disableTransitionOnChange = false,
  ...props
}: ThemeProviderProps) {
  const [theme, setThemeState] = React.useState<Theme>(defaultTheme);
  const [resolvedTheme, setResolvedTheme] = React.useState<"dark" | "light">(
    "light",
  );

  React.useEffect(() => {
    const root = window.document.documentElement;

    // Add theme-changing class to disable transitions
    root.classList.add("theme-changing");

    root.classList.remove("light", "dark");

    if (theme === "system" && enableSystem) {
      const systemTheme = window.matchMedia("(prefers-color-scheme: dark)")
        .matches
        ? "dark"
        : "light";

      root.classList.add(systemTheme);
      setResolvedTheme(systemTheme);
    } else {
      root.classList.add(theme);
      setResolvedTheme(theme as "dark" | "light");
    }

    // Remove theme-changing class after transition
    setTimeout(() => {
      root.classList.remove("theme-changing");
    }, 10);
  }, [theme, enableSystem]);

  // Listen for system theme changes
  React.useEffect(() => {
    if (theme !== "system" || !enableSystem) return;

    const mediaQuery = window.matchMedia("(prefers-color-scheme: dark)");

    const handleChange = (e: MediaQueryListEvent) => {
      const systemTheme = e.matches ? "dark" : "light";
      const root = window.document.documentElement;

      // Add theme-changing class to disable transitions
      root.classList.add("theme-changing");

      root.classList.remove("light", "dark");
      root.classList.add(systemTheme);
      setResolvedTheme(systemTheme);

      // Remove theme-changing class after transition
      setTimeout(() => {
        root.classList.remove("theme-changing");
      }, 10);
    };

    mediaQuery.addEventListener("change", handleChange);
    return () => mediaQuery.removeEventListener("change", handleChange);
  }, [theme, enableSystem]);

  // Load theme from localStorage on mount
  React.useEffect(() => {
    try {
      const storedTheme = localStorage.getItem(storageKey) as Theme;
      if (storedTheme && ["dark", "light", "system"].includes(storedTheme)) {
        setThemeState(storedTheme);
      }
    } catch (_error) {
      // localStorage is not available
      console.warn("Failed to load theme from localStorage:", _error);
    }
  }, [storageKey]);

  const setTheme = React.useCallback(
    (newTheme: Theme) => {
      if (disableTransitionOnChange) {
        // Use theme-changing class instead of inline styles
        document.documentElement.classList.add("theme-changing");
        setTimeout(() => {
          document.documentElement.classList.remove("theme-changing");
        }, 10);
      }

      try {
        localStorage.setItem(storageKey, newTheme);
      } catch (_error) {
        // localStorage is not available
        console.warn("Failed to save theme to localStorage:", _error);
      }

      setThemeState(newTheme);
    },
    [storageKey, disableTransitionOnChange],
  );

  const value = React.useMemo(
    () => ({
      theme,
      setTheme,
      resolvedTheme,
    }),
    [theme, setTheme, resolvedTheme],
  );

  return (
    <ThemeProviderContext.Provider {...props} value={value}>
      {children}
    </ThemeProviderContext.Provider>
  );
}

export const useTheme = () => {
  const context = React.useContext(ThemeProviderContext);

  if (context === undefined)
    throw new Error("useTheme must be used within a ThemeProvider");

  return context;
};
