
import { convertDatabaseToSteps } from "../../src/lib/experiment-designer/block-converter";
import { type ExperimentStep } from "../../src/lib/experiment-designer/types";

// Mock DB Steps (simulating what experimentsRouter returns before conversion)
const mockDbSteps = [
    {
        id: "step-1",
        name: "Step 1",
        type: "wizard",
        orderIndex: 0,
        actions: [
            {
                id: "seq-1",
                name: "Test Sequence",
                type: "sequence",
                parameters: {
                    children: [
                        { id: "child-1", name: "Child 1", type: "wait", parameters: { duration: 1 } },
                        { id: "child-2", name: "Child 2", type: "wait", parameters: { duration: 2 } }
                    ]
                }
            }
        ]
    }
];

// Mock Store Logic (simulating store.ts)
function cloneActions(actions: any[]): any[] {
    return actions.map((a) => ({
        ...a,
        children: a.children ? cloneActions(a.children) : undefined,
    }));
}

function cloneSteps(steps: any[]): any[] {
    return steps.map((s) => ({
        ...s,
        actions: cloneActions(s.actions),
    }));
}

console.log("🔹 Testing Hydration & Cloning...");

// 1. Convert DB -> Runtime
const runtimeSteps = convertDatabaseToSteps(mockDbSteps);
const seq = runtimeSteps[0]?.actions[0];

if (!seq) {
    console.error("❌ Conversion Failed: Sequence action not found.");
    process.exit(1);
}

console.log(`Runtime Children Count: ${seq.children?.length ?? "undefined"}`);

if (!seq.children || seq.children.length === 0) {
    console.error("❌ Conversion Failed: Children not hydrated from parameters.");
    process.exit(1);
}

// 2. Store Cloning
const clonedSteps = cloneSteps(runtimeSteps);
const clonedSeq = clonedSteps[0]?.actions[0];

if (!clonedSeq) {
    console.error("❌ Cloning Failed: Sequence action lost.");
    process.exit(1);
}

console.log(`Cloned Children Count: ${clonedSeq.children?.length ?? "undefined"}`);

if (clonedSeq.children?.length === 2) {
    console.log("✅ SUCCESS: Data hydrated and cloned correctly.");
} else {
    console.error("❌ CLONING FAILED: Children lost during clone.");
}
