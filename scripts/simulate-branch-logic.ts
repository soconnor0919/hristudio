
// Mock of the logic in WizardInterface.tsx handleNextStep
const steps = [
    {
        id: "b9d43f8c-c40c-4f1c-9fdc-9076338d3c85",
        name: "Step 3 (Conditional)",
        order: 2
    },
    {
        id: "3a2dc0b7-a43e-4236-9b9e-f957abafc1e5",
        name: "Step 4 (Branch A)",
        order: 3,
        conditions: {
            "nextStepId": "cc3fbc7f-29e5-45e0-8d46-e80813c54292"
        }
    },
    {
        id: "3ae2fe8a-fc5d-4a04-baa5-699a21f19e30",
        name: "Step 5 (Branch B)",
        order: 4,
        conditions: {
            "nextStepId": "cc3fbc7f-29e5-45e0-8d46-e80813c54292"
        }
    },
    {
        id: "cc3fbc7f-29e5-45e0-8d46-e80813c54292",
        name: "Step 6 (Conclusion)",
        order: 5
    }
];

function simulateNextStep(currentStepIndex: number) {
    const currentStep = steps[currentStepIndex];
    console.log(`\n--- Simulating Next Step from: ${currentStep.name} ---`);
    console.log("Current Step Data:", JSON.stringify(currentStep, null, 2));

    // Logic from WizardInterface.tsx
    console.log("[WizardInterface] Checking for nextStepId condition:", currentStep?.conditions);

    if (currentStep?.conditions?.nextStepId) {
        const nextId = String(currentStep.conditions.nextStepId);
        const targetIndex = steps.findIndex(s => s.id === nextId);

        console.log(`Target ID: ${nextId}`);
        console.log(`Target Index Found: ${targetIndex}`);

        if (targetIndex !== -1) {
            console.log(`[WizardInterface] Condition-based jump to step ${targetIndex} (${nextId})`);
            return targetIndex;
        } else {
            console.warn(`[WizardInterface] Targeted nextStepId ${nextId} not found in steps list.`);
        }
    } else {
        console.log("[WizardInterface] No nextStepId found in conditions, proceeding linearly.");
    }

    // Default: Linear progression
    const nextIndex = currentStepIndex + 1;
    console.log(`Proceeding linearly to index ${nextIndex}`);
    return nextIndex;
}

// Simulate Branch A (Index 1 in this array, but 3 in real experiment?)
// In real exp, Step 3 is index 2. Step 4 (Branch A) is index 3.
console.log("Real experiment indices:");
// 0: Hook, 1: Narrative, 2: Conditional, 3: Branch A, 4: Branch B, 5: Conclusion
const indexStep4 = 1; // logical index in my mock array
const indexStep5 = 2; // logical index

console.log("Testing Branch A Logic:");
const resultA = simulateNextStep(indexStep4);
if (resultA === 3) console.log("SUCCESS: Branch A jumped to Conclusion");
else console.log("FAILURE: Branch A fell through");

console.log("\nTesting Branch B Logic:");
const resultB = simulateNextStep(indexStep5);
if (resultB === 3) console.log("SUCCESS: Branch B jumped to Conclusion");
else console.log("FAILURE: Branch B fell through");
