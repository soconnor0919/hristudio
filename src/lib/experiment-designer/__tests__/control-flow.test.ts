
import { describe, it, expect } from "vitest";
import { convertStepsToDatabase, convertDatabaseToSteps } from "../block-converter";
import type { ExperimentStep, ExperimentAction } from "../types";

// Mock Action
const branchAction: ExperimentAction = {
    id: "act-branch-1",
    name: "Decision",
    type: "branch",
    category: "control",
    parameters: {},
    source: { kind: "core", baseActionId: "branch" },
    execution: { transport: "internal" }
};

describe("Control Flow Persistence", () => {
    it("should persist conditional branching options", () => {
        const originalSteps: ExperimentStep[] = [
            {
                id: "step-1",
                name: "Question",
                type: "conditional",
                order: 0,
                trigger: {
                    type: "trial_start",
                    conditions: {
                        options: [
                            { label: "Yes", nextStepIndex: 1, variant: "default" },
                            { label: "No", nextStepIndex: 2, variant: "destructive" }
                        ]
                    }
                },
                actions: [branchAction],
                expanded: true
            },
            {
                id: "step-2",
                name: "Path A",
                type: "sequential",
                order: 1,
                trigger: { type: "previous_step", conditions: {} },
                actions: [],
                expanded: true
            },
            {
                id: "step-3",
                name: "Path B",
                type: "sequential",
                order: 2,
                trigger: { type: "previous_step", conditions: {} },
                actions: [],
                expanded: true
            }
        ];

        // Simulate Save
        const dbRows = convertStepsToDatabase(originalSteps);

        // START DEBUG
        // console.log("DB Rows Conditions:", JSON.stringify(dbRows[0].conditions, null, 2));
        // END DEBUG

        expect(dbRows[0].type).toBe("conditional");
        expect((dbRows[0].conditions as any).options).toHaveLength(2);

        // Simulate Load
        const hydratedSteps = convertDatabaseToSteps(dbRows);

        expect(hydratedSteps[0].type).toBe("conditional");
        expect((hydratedSteps[0].trigger.conditions as any).options).toHaveLength(2);
        expect((hydratedSteps[0].trigger.conditions as any).options[0].label).toBe("Yes");
    });

    it("should persist loop configuration", () => {
        const originalSteps: ExperimentStep[] = [
            {
                id: "step-loop-1",
                name: "Repeat Task",
                type: "loop",
                order: 0,
                trigger: {
                    type: "trial_start",
                    conditions: {
                        loop: {
                            iterations: 5,
                            requireApproval: false
                        }
                    }
                },
                actions: [],
                expanded: true
            }
        ];

        // Simulate Save
        const dbRows = convertStepsToDatabase(originalSteps);

        // Note: 'loop' type is mapped to 'conditional' in DB, but detailed conditions should survive
        expect(dbRows[0].type).toBe("conditional");
        expect((dbRows[0].conditions as any).loop.iterations).toBe(5);

        // Simulate Load
        const hydratedSteps = convertDatabaseToSteps(dbRows);

        // Checking data integrity
        expect((hydratedSteps[0].trigger.conditions as any).loop).toBeDefined();
        expect((hydratedSteps[0].trigger.conditions as any).loop.iterations).toBe(5);
    });
});
