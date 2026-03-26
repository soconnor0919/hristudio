import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function DesigningExperimentsTutorial() {
  return (
    <TutorialPage
      title="Designing Experiments"
      description="Build experiment protocols with the visual designer"
      duration="25 min"
      level="Intermediate"
      steps={[
        { title: "Understand the experiment structure", description: "" },
        { title: "Navigate the visual designer", description: "" },
        { title: "Use core blocks", description: "" },
        { title: "Build branching protocols", description: "" },
        { title: "Test your experiment", description: "" },
      ]}
      prevTutorial={{
        title: "Your First Study",
        href: "/help/tutorials/your-first-study",
      }}
      nextTutorial={{
        title: "Running Trials",
        href: "/help/tutorials/running-trials",
      }}
    >
      <h2>What is an Experiment?</h2>
      <p>An <strong>Experiment</strong> defines the protocol for your study:</p>
      <pre><code>Experiment
├── Steps (ordered sequence)
│   ├── Actions (robot behaviors)
│   ├── Wizard Blocks (human decisions)
│   └── Control Flow (loops, branches)
├── Robot Actions (from plugins)
└── Parameters (configurable values)</code></pre>

      <h2>Step 1: Create an Experiment</h2>
      <ol>
        <li>Open your study</li>
        <li>Go to <strong>Experiments</strong> tab</li>
        <li>Click <strong>New Experiment</strong></li>
      </ol>

      <h2>Step 2: The Visual Designer</h2>
      <p>The designer has three main areas:</p>
      <ul>
        <li><strong>Block Library</strong> (left) - Drag blocks from here</li>
        <li><strong>Canvas</strong> (center) - Design your protocol visually</li>
        <li><strong>Properties Panel</strong> (right) - Configure selected elements</li>
      </ul>

      <h2>Step 3: Block Categories</h2>

      <h3>Events (Triggers)</h3>
      <p>Start your experiment with these blocks:</p>
      <table>
        <thead>
          <tr><th>Block</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>Trial Start</td><td>Triggers when trial begins</td></tr>
          <tr><td>Wizard Button</td><td>Waits for wizard to press a button</td></tr>
          <tr><td>Timer</td><td>Waits for a specified duration</td></tr>
          <tr><td>Participant Response</td><td>Waits for participant input</td></tr>
        </tbody>
      </table>

      <h3>Wizard Actions</h3>
      <p>Blocks the wizard can control:</p>
      <table>
        <thead>
          <tr><th>Block</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>Say Text</td><td>Robot speaks text</td></tr>
          <tr><td>Play Animation</td><td>Play a predefined animation</td></tr>
          <tr><td>Show Image</td><td>Display image on robot screen</td></tr>
          <tr><td>Move Robot</td><td>Move robot to position</td></tr>
        </tbody>
      </table>

      <h3>Control Flow</h3>
      <p>Control experiment progression:</p>
      <table>
        <thead>
          <tr><th>Block</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>Branch</td><td>Split into multiple paths</td></tr>
          <tr><td>Loop</td><td>Repeat a sequence</td></tr>
          <tr><td>Wait</td><td>Pause for duration</td></tr>
          <tr><td>Converge</td><td>Merge multiple paths back</td></tr>
        </tbody>
      </table>

      <h2>Step 4: Building a Branching Protocol</h2>
      <p>Let&apos;s build &quot;The Interactive Storyteller&quot; - a simple storytelling experiment:</p>

      <h3>Step 1: The Hook (Start)</h3>
      <ol>
        <li>Click <strong>+ Add Step</strong></li>
        <li>Name it &quot;The Hook&quot;</li>
        <li>Set type to <strong>Robot</strong></li>
        <li>Drag <strong>Say Text</strong> block</li>
        <li>Configure: <code>{`{ text: "Hello! I have a story to tell you." }`}</code></li>
      </ol>

      <h3>Step 2: Comprehension Check (Branching)</h3>
      <ol>
        <li>Add new step &quot;Comprehension Check&quot;</li>
        <li>Set type to <strong>Conditional</strong></li>
        <li>Add <strong>Ask Question</strong> block</li>
        <li>Configure options:
          <pre><code>{`{
  question: "What color was the rock?",
  options: [
    { label: "Correct", value: "red" },
    { label: "Incorrect", value: "other" }
  ]
}`}</code></pre>
        </li>
        <li>This creates two paths automatically</li>
      </ol>

      <h3>Step 3: Converge Paths</h3>
      <ol>
        <li>Add new step &quot;Story Continues&quot;</li>
        <li>Set type to <strong>Converge</strong></li>
        <li>Connect both branches to this step</li>
      </ol>

      <h2>Step 5: Testing Your Experiment</h2>

      <h3>Preview Mode</h3>
      <p>Test your experiment without running a real trial:</p>
      <ol>
        <li>Click <strong>Preview</strong> button</li>
        <li>Step through each block</li>
        <li>See timing and flow</li>
        <li>Test branching decisions</li>
      </ol>

      <h3>Simulation Mode</h3>
      <p>Run with a simulated robot:</p>
      <ol>
        <li>Enable <code>NEXT_PUBLIC_SIMULATION_MODE=true</code></li>
        <li>Start a trial</li>
        <li>Robot actions are logged but not executed</li>
      </ol>

      <h2>Common Patterns</h2>

      <h3>Linear Protocol</h3>
      <pre><code>Start → Step 1 → Step 2 → Step 3 → End</code></pre>

      <h3>Branching Protocol</h3>
      <pre><code>Start → Step 1
          ├── Condition A → Step 2a
          └── Condition B → Step 2b</code></pre>

      <h3>Loop Protocol</h3>
      <pre><code>Start → Step 1 → Loop (3x) → Step 2 → End
             ↑
             └── (back to Step 1)</code></pre>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/your-first-study">
            Previous: Your First Study
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/running-trials">
            Next: Running Trials
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
