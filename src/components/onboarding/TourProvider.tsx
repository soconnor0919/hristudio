"use client";

import React, { createContext, useContext, useEffect, useRef } from "react";
import { driver, type Driver } from "driver.js";
import "driver.js/dist/driver.css";
import { useTheme } from "next-themes";
import { usePathname } from "next/navigation";
import Cookies from "js-cookie";

type TourType = "dashboard" | "study_creation" | "participant_creation" | "designer" | "wizard" | "full_platform";

interface TourContextType {
    startTour: (tour: TourType) => void;
}

const TourContext = createContext<TourContextType | undefined>(undefined);

export function useTour() {
    const context = useContext(TourContext);
    if (!context) {
        throw new Error("useTour must be used within a TourProvider");
    }
    return context;
}

export function TourProvider({ children }: { children: React.ReactNode }) {
    const driverObj = useRef<Driver | null>(null);
    const { theme } = useTheme();
    const pathname = usePathname();

    // --- Multi-page Tour Logic ---
    useEffect(() => {
        // Check if we are in "Full Platform" mode (Local Storage OR Cookie)
        const localMode = localStorage.getItem("hristudio_tour_mode");
        const cookieMode = Cookies.get("hristudio_tour_mode");

        const tourMode = localMode === "full" || cookieMode === "full" ? "full" : null;

        if (tourMode === "full") {
            // Re-sync local storage if missing but cookie present
            if (localMode !== "full") localStorage.setItem("hristudio_tour_mode", "full");

            // Small delay to ensure DOM is ready
            const timer = setTimeout(() => {
                if (pathname === "/dashboard") {
                    runTourSegment("dashboard");
                } else if (pathname.includes("/studies/new")) {
                    runTourSegment("study_creation");
                } else if (pathname.includes("/participants/new")) {
                    runTourSegment("participant_creation");
                } else if (pathname.includes("/designer")) {
                    runTourSegment("designer");
                } else if (pathname.includes("/wizard")) {
                    runTourSegment("wizard");
                }
            }, 500); // Reduced delay for snappier feel, but still safe for render
            return () => clearTimeout(timer);
        }
    }, [pathname]);

    const runTourSegment = (segment: "dashboard" | "study_creation" | "participant_creation" | "designer" | "wizard") => {
        const isDark = theme === "dark";
        // We add a specific class to handle dark/light overrides reliably
        const themeClass = isDark ? "driverjs-theme-dark" : "driverjs-theme-light";

        let steps: any[] = [];

        if (segment === "dashboard") {
            steps = [
                {
                    element: "#dashboard-header",
                    popover: {
                        title: "Overview",
                        description: "Welcome to HRIStudio. This dashboard gives you a high-level view of your research activities, active studies, and data collection progress.",
                        side: "bottom",
                        align: "start",
                    },
                },
                {
                    element: "#tour-sidebar-overview",
                    popover: {
                        title: "Navigation: Overview",
                        description: "Quickly return to this main dashboard from anywhere in the application.",
                        side: "right",
                    },
                },
                {
                    element: "#tour-sidebar-studies",
                    popover: {
                        title: "Navigation: Studies",
                        description: "Manage all your research studies, IRBs, and team permissions in one place.",
                        side: "right",
                    },
                },
                {
                    element: "#tour-sidebar-study-selector",
                    popover: {
                        title: "Active Study Selector",
                        description: "Switch between different studies here. Selecting a study unlocks study-specific tools like the Experiment Designer and Data Analytics.",
                        side: "right",
                    },
                },
                {
                    element: "#tour-new-study",
                    popover: {
                        title: "Create a New Study",
                        description: "Ready to start? Click here to initialize a new research project and define your protocol.",
                        side: "right",
                    },
                },
            ];
        } else if (segment === "study_creation") {
            steps = [
                {
                    element: "#tour-study-name",
                    popover: {
                        title: "Naming Your Study",
                        description: "Choose a concise, descriptive name. This will properly namespace your data, logs, and robot configurations.",
                        side: "right",
                    }
                },
                {
                    element: "#tour-study-description",
                    popover: {
                        title: "Research Protocol",
                        description: "Add a short description of your methodology or research questions. This helps team members understand the context.",
                        side: "right",
                    }
                },
                {
                    element: "#tour-study-submit",
                    popover: {
                        title: "Initialize Project",
                        description: "Create the study to access the full suite of tools: Experiment Designer, Wizard Interface, and Analytics.",
                        side: "top",
                    }
                }
            ];
        } else if (segment === "participant_creation") {
            steps = [
                {
                    element: "#tour-participant-code",
                    popover: {
                        title: "Participant ID",
                        description: "Assign a unique code (e.g., P001) to identify this participant while maintaining anonymity.",
                        side: "right",
                    }
                },
                {
                    element: "#tour-participant-name",
                    popover: {
                        title: "Name (Optional)",
                        description: "You store their name for internal reference; analytics will use the ID.",
                        side: "right",
                    }
                },
                {
                    element: "#tour-participant-study-container",
                    popover: {
                        title: "Study Association",
                        description: "Link this participant to a specific research study to enable data collection.",
                        side: "right",
                    }
                },
                {
                    element: "#tour-participant-consent",
                    popover: {
                        title: "Informed Consent",
                        description: "Mandatory check to confirm you have obtained necessary ethical approvals and consent.",
                        side: "top",
                    }
                },
                {
                    element: "#tour-participant-submit",
                    popover: {
                        title: "Register",
                        description: "Create the participant record to begin scheduling trials.",
                        side: "top",
                    }
                }
            ];
        } else if (segment === "designer") {
            steps = [
                {
                    element: "#tour-designer-blocks",
                    popover: {
                        title: "Action Library",
                        description: "Drag and drop robot behaviors (Speech, Gestures, Movement) onto the canvas. Includes both core actions and those from installed plugins.",
                        side: "right",
                    },
                },
                {
                    element: "#tour-designer-canvas",
                    popover: {
                        title: "Visual Flow Canvas",
                        description: "Design your experiment logic here. Connect blocks to create sequences, branches, and loops for the robot to execute.",
                        side: "top",
                    },
                },
                {
                    element: "#tour-designer-properties",
                    popover: {
                        title: "Properties Panel",
                        description: "Select any block to configure its parameters—like speech text, speed, volume, or timeout durations.",
                        side: "left",
                    },
                },
            ];
        } else if (segment === "wizard") {
            steps = [
                {
                    element: "#tour-wizard-controls",
                    popover: {
                        title: "Wizard Dashboard",
                        description: "The command center for running trials. Manually trigger robot actions or override autonomous behaviors in real-time.",
                        side: "right",
                    },
                },
                {
                    element: "#tour-wizard-timeline",
                    popover: {
                        title: "Live Timeline",
                        description: "See exactly what the robot is doing, what's coming next, and a history of all events in the current session.",
                        side: "top",
                    },
                },
                {
                    element: "#tour-wizard-robot-status",
                    popover: {
                        title: "System Health",
                        description: "Monitor critical telemetry: battery levels, joint temperatures, and network latency to ensure safety.",
                        side: "left",
                    },
                },
            ];
        }

        driverObj.current = driver({
            showProgress: true,
            animate: true,
            allowClose: true,
            steps: steps.map((step) => ({
                ...step,
                popover: {
                    ...step.popover,
                    popoverClass: `driver-popover-override ${themeClass}`,
                },
            })),
            onDestroyed: () => {
                // Persistence handled by localStorage state
            }
        });

        driverObj.current.drive();
    };

    const startTour = (tour: TourType) => {
        if (tour === "full_platform") {
            localStorage.setItem("hristudio_tour_mode", "full");
            Cookies.set("hristudio_tour_mode", "full", { expires: 7 }); // 7 days persistence

            // Trigger current page immediately
            if (pathname === "/dashboard") runTourSegment("dashboard");
            else if (pathname.includes("/studies/new")) runTourSegment("study_creation");
            else if (pathname.includes("/participants/new")) runTourSegment("participant_creation");
            else if (pathname.includes("/designer")) runTourSegment("designer");
            else if (pathname.includes("/wizard")) runTourSegment("wizard");
            else runTourSegment("dashboard"); // Fallback
        } else {
            localStorage.setItem("hristudio_tour_mode", "manual");
            Cookies.remove("hristudio_tour_mode");

            if (tour === "dashboard") runTourSegment("dashboard");
            if (tour === "study_creation") runTourSegment("study_creation");
            if (tour === "participant_creation") runTourSegment("participant_creation");
            if (tour === "designer") runTourSegment("designer");
            if (tour === "wizard") runTourSegment("wizard");
        }
    };

    return (
        <TourContext.Provider value={{ startTour }}>
            {children}
            <style jsx global>{`
        /* 
           SHADCN/UI THEMING OVERRIDES
           CRITICAL: The global variables in globals.css use OKLCH/HSL values directly or with units.
           DO NOT wrap variables in hsl() if they are already defined as colors.
           Use direct assignment.
        */
        
        .driver-popover-override {
            padding: 1.25rem !important;
            border-radius: var(--radius) !important;
            box-shadow: var(--shadow-xl) !important;
            max-width: 420px !important;
            
            /* Background & Text - Match Card Aesthetic */
            background-color: var(--card) !important;
            color: var(--card-foreground) !important;
            border: 1px solid var(--border) !important;

            /* Typography */
            font-family: var(--font-sans) !important;
        }

        /* Arrow Styling - Critical for transparent/card matching */
        .driver-popover-override .driver-popover-arrow {
            border-width: 8px !important;
        }

        /* 
         Since driver.js uses borders for arrows, we need to match the specific side.
         Using CSS variables requires a bit of trickery because border-color expects distinct values.
         We'll target the side classes driver.js adds.
        */
        .driver-popover-override.driverjs-theme-dark .driver-popover-arrow-side-left.driver-popover-arrow {
            border-left-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-dark .driver-popover-arrow-side-right.driver-popover-arrow {
            border-right-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-dark .driver-popover-arrow-side-top.driver-popover-arrow {
            border-top-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-dark .driver-popover-arrow-side-bottom.driver-popover-arrow {
            border-bottom-color: var(--card) !important;
        }

        /* Light mode fallbacks (using border color for definition, though card bg is usually sufficient) */
        .driver-popover-override.driverjs-theme-light .driver-popover-arrow-side-left.driver-popover-arrow {
            border-left-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-light .driver-popover-arrow-side-right.driver-popover-arrow {
            border-right-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-light .driver-popover-arrow-side-top.driver-popover-arrow {
            border-top-color: var(--card) !important;
        }
        .driver-popover-override.driverjs-theme-light .driver-popover-arrow-side-bottom.driver-popover-arrow {
            border-bottom-color: var(--card) !important;
        }

        /* Title Styling */
        .driver-popover-override .driver-popover-title {
            color: var(--foreground) !important;
            font-size: 1.125rem !important; /* 18px */
            font-weight: 600 !important;
            margin-bottom: 0.5rem !important;
            letter-spacing: -0.015em !important;
            font-family: var(--font-sans) !important;
        }
        
        /* Description Styling */
        .driver-popover-override .driver-popover-description {
            color: var(--muted-foreground) !important;
            font-size: 0.875rem !important; /* 14px */
            line-height: 1.6 !important;
            font-family: var(--font-sans) !important;
        }
        
        /* Buttons */
        .driver-popover-override .driver-popover-footer button {
            background-color: var(--primary) !important;
            color: var(--primary-foreground) !important;
            border-radius: calc(var(--radius) - 2px) !important;
            padding: 0.5rem 1rem !important;
            font-size: 0.875rem !important;
            font-weight: 500 !important;
            border: none !important;
            text-shadow: none !important;
            transition-all: 0.2s !important;
            font-family: var(--font-sans) !important;
        }
        
        .driver-popover-override .driver-popover-footer button:hover {
             opacity: 0.9 !important;
             transform: translateY(-1px);
        }

        /* Navigation Buttons (Previous/Next) specifically */
        .driver-popover-override .driver-popover-footer .driver-popover-prev-btn {
             background-color: transparent !important;
             color: var(--muted-foreground) !important;
             border: 1px solid var(--border) !important;
        }
        .driver-popover-override .driver-popover-footer .driver-popover-prev-btn:hover {
             background-color: var(--accent) !important;
             color: var(--accent-foreground) !important;
        }
        
        /* Close Button */
        .driver-popover-override .driver-popover-close-btn {
            color: var(--muted-foreground) !important;
            opacity: 0.7 !important;
            transition: opacity 0.2s !important;
        }
        
        .driver-popover-override .driver-popover-close-btn:hover {
            color: var(--foreground) !important;
            opacity: 1 !important;
        }
      `}</style>
        </TourContext.Provider>
    );
}
