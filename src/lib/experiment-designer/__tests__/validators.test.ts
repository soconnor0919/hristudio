
import { describe, it, expect } from "vitest";
import { validateExperimentDesign, validateStructural, validateParameters } from "../../../components/experiments/designer/state/validators";
import type { ExperimentStep, ExperimentAction, ActionDefinition } from "../../experiment-designer/types";

// Mock Data
const mockActionDef: ActionDefinition = {
    id: "core.log",
    name: "Log Info",
    type: "log",
    category: "utility",
    parameters: [
        { id: "message", name: "Message", type: "text", required: true },
        { id: "level", name: "Level", type: "select", options: ["info", "warn", "error"], default: "info" }
    ],
    source: { kind: "core", baseActionId: "log" }
};

const validStep: ExperimentStep = {
    id: "step-1",
    name: "Step 1",
    type: "sequential",
    order: 0,
    trigger: { type: "trial_start", conditions: {} },
    actions: [],
    expanded: true,
    description: "A valid step"
};

describe("Experiment Validators", () => {

    describe("Structural Validation", () => {
        it("should fail if experiment has no steps", () => {
            const result = validateExperimentDesign([], { steps: [], actionDefinitions: [] });
            expect(result.valid).toBe(false);
            expect(result.issues[0].message).toContain("at least one step");
        });

        it("should fail if step name is empty", () => {
            const step = { ...validStep, name: "" };
            const issues = validateStructural([step], { steps: [step], actionDefinitions: [] });
            expect(issues.some(i => i.field === "name" && i.severity === "error")).toBe(true);
        });

        it("should fail if step type is invalid", () => {
            const step = { ...validStep, type: "magic_step" as any };
            const issues = validateStructural([step], { steps: [step], actionDefinitions: [] });
            expect(issues.some(i => i.field === "type" && i.severity === "error")).toBe(true);
        });
    });

    describe("Parameter Validation", () => {
        it("should fail if required parameter is missing", () => {
            const action: ExperimentAction = {
                id: "act-1",
                type: "log",
                name: "Log",
                order: 0,
                parameters: {}, // Missing 'message'
                source: { kind: "core", baseActionId: "log" },
                execution: { transport: "internal" }
            };
            const step: ExperimentStep = { ...validStep, actions: [action] };

            const issues = validateParameters([step], {
                steps: [step],
                actionDefinitions: [mockActionDef]
            });

            expect(issues.some(i => i.field === "parameters.message" && i.severity === "error")).toBe(true);
        });

        it("should pass if required parameter is present", () => {
            const action: ExperimentAction = {
                id: "act-1",
                type: "log",
                name: "Log",
                order: 0,
                parameters: { message: "Hello" },
                source: { kind: "core", baseActionId: "log" },
                execution: { transport: "internal" }
            };
            const step: ExperimentStep = { ...validStep, actions: [action] };

            const issues = validateParameters([step], {
                steps: [step],
                actionDefinitions: [mockActionDef]
            });

            // Should have 0 errors, maybe warnings but no parameter errors
            const paramErrors = issues.filter(i => i.category === "parameter" && i.severity === "error");
            expect(paramErrors).toHaveLength(0);
        });

        it("should validate number ranges", () => {
            const rangeActionDef: ActionDefinition = {
                ...mockActionDef,
                id: "math",
                type: "math",
                parameters: [{ id: "val", name: "Value", type: "number", min: 0, max: 10 }]
            };

            const action: ExperimentAction = {
                id: "act-1",
                type: "math",
                name: "Math",
                order: 0,
                parameters: { val: 15 }, // Too high
                source: { kind: "core", baseActionId: "math" },
                execution: { transport: "internal" }
            };
            const step: ExperimentStep = { ...validStep, actions: [action] };

            const issues = validateParameters([step], {
                steps: [step],
                actionDefinitions: [rangeActionDef]
            });

            expect(issues[0].message).toContain("must be at most 10");
        });
    });
});
