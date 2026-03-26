import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function SimulationModeTutorial() {
  return (
    <TutorialPage
      title="Simulation Mode"
      description="Test experiments without a physical robot"
      duration="10 min"
      level="Beginner"
      steps={[
        { title: "Enable simulation mode", description: "" },
        { title: "Test robot actions", description: "" },
        { title: "Run test trials", description: "" },
        { title: "Practice wizard controls", description: "" },
        { title: "Transition to real robot", description: "" },
      ]}
      prevTutorial={{
        title: "Data & Analysis",
        href: "/help/tutorials/data-and-analysis",
      }}
    >
      <h2>Why Simulation Mode?</h2>
      <p>Simulation mode allows you to:</p>
      <ul>
        <li><strong>Test protocols</strong> without a robot</li>
        <li><strong>Train wizards</strong> before live sessions</li>
        <li><strong>Debug experiments</strong> in development</li>
        <li><strong>Run pilots</strong> without robot access</li>
        <li><strong>Develop</strong> on any computer</li>
      </ul>

      <h2>Simulation Options</h2>
      <p>HRIStudio offers two simulation approaches:</p>
      <table>
        <thead>
          <tr><th>Approach</th><th>Pros</th><th>Cons</th></tr>
        </thead>
        <tbody>
          <tr>
            <td>Client-side</td>
            <td>No server needed, instant</td>
            <td>Limited robot simulation</td>
          </tr>
          <tr>
            <td>Mock Server</td>
            <td>Full rosbridge protocol</td>
            <td>Requires running server</td>
          </tr>
        </tbody>
      </table>

      <h2>Step 1: Enable Client-Side Simulation</h2>

      <h3>Quick Start</h3>
      <ol>
        <li>Create or edit <code>hristudio/.env.local</code></li>
        <li>Add: <code>NEXT_PUBLIC_SIMULATION_MODE=true</code></li>
        <li>Restart the dev server:
          <pre><code>bun dev</code></pre>
        </li>
      </ol>

      <h3>Verify Enabled</h3>
      <p>Look for the simulation indicator in the UI:</p>
      <pre><code>Wizard Interface [🔵 SIMULATION MODE]</code></pre>

      <h2>Step 2: Start Mock Server (Optional)</h2>
      <p>For more complete testing, use the mock server:</p>

      <h3>Standalone Server</h3>
      <pre><code>cd hristudio/scripts/mock-robot
bun install
bun dev</code></pre>

      <h3>Docker</h3>
      <pre><code>cd nao6-hristudio-integration
docker compose -f docker-compose.yml -f docker-compose.mock.yml --profile mock up -d</code></pre>

      <h2>Step 3: Test Robot Actions</h2>

      <h3>From NAO Test Page</h3>
      <ol>
        <li>Navigate to: <code>/nao-test</code></li>
        <li>Click <strong>Connect</strong></li>
        <li>Test actions:
          <ul>
            <li><strong>Speech</strong> - Enter text, click Say</li>
            <li><strong>Movement</strong> - Set speed, click Walk</li>
            <li><strong>Head</strong> - Set angles, click Move</li>
          </ul>
        </li>
      </ol>

      <h3>Simulated Actions</h3>
      <table>
        <thead>
          <tr><th>Action</th><th>Simulation Behavior</th></tr>
        </thead>
        <tbody>
          <tr><td>say_text</td><td>Duration = 1.5s + 300ms × word_count</td></tr>
          <tr><td>walk_forward</td><td>Position updates over 500ms</td></tr>
          <tr><td>turn_left/right</td><td>Angle changes over 500ms</td></tr>
        </tbody>
      </table>

      <h2>Step 4: Run Test Trials</h2>
      <ol>
        <li>Enable simulation mode</li>
        <li>Create or open experiment</li>
        <li>Schedule trial</li>
        <li>Start trial in wizard interface</li>
        <li>Execute through all steps</li>
        <li>Verify timing and flow</li>
      </ol>

      <h3>Test Checklist</h3>
      <ul>
        <li>All steps execute in order</li>
        <li>Branching decisions work</li>
        <li>Timing estimates are accurate</li>
        <li>Event log captures everything</li>
        <li>No errors or warnings</li>
        <li>Trial completes successfully</li>
      </ul>

      <h2>Step 5: Training Wizards</h2>
      <p>Simulation mode is perfect for training:</p>

      <h3>Training Scenarios</h3>
      <ol>
        <li><strong>Basic Operation</strong> - Start/pause trials, execute actions</li>
        <li><strong>Decision Making</strong> - Select appropriate branches</li>
        <li><strong>Handling Issues</strong> - Pause, respond to alerts, stop early</li>
      </ol>

      <h2>Transitioning to Real Robot</h2>
      <ol>
        <li><strong>Disable Simulation</strong>
          <pre><code>NEXT_PUBLIC_SIMULATION_MODE=false</code></pre>
        </li>
        <li><strong>Connect Robot</strong>
          <ul>
            <li>Start Docker services</li>
            <li>Verify robot connection</li>
            <li>Test with NAO Test Page</li>
          </ul>
        </li>
        <li><strong>Run Comparison Trial</strong>
          <ul>
            <li>Run same experiment on real robot</li>
            <li>Compare timing and behavior</li>
            <li>Adjust parameters as needed</li>
          </ul>
        </li>
      </ol>

      <h2>Comparison: Simulation vs Real</h2>
      <table>
        <thead>
          <tr><th>Aspect</th><th>Simulation</th><th>Real Robot</th></tr>
        </thead>
        <tbody>
          <tr><td>Setup time</td><td>1 min</td><td>30+ min</td></tr>
          <tr><td>Availability</td><td>Always</td><td>Requires robot</td></tr>
          <tr><td>Cost</td><td>Free</td><td>Robot access needed</td></tr>
          <tr><td>Timing accuracy</td><td>Estimated</td><td>Actual</td></tr>
          <tr><td>Physical interaction</td><td>✗</td><td>✓</td></tr>
          <tr><td>Sensor accuracy</td><td>Fake</td><td>Real</td></tr>
        </tbody>
      </table>

      <h2>Best Practices</h2>

      <h3>When to Use Simulation</h3>
      <ul>
        <li>During experiment design</li>
        <li>While robot unavailable</li>
        <li>For wizard training</li>
        <li>For debugging protocols</li>
        <li>For quick iteration</li>
      </ul>

      <h3>When to Use Real Robot</h3>
      <ul>
        <li>Final protocol validation</li>
        <li>Timing accuracy critical</li>
        <li>Physical interaction matters</li>
        <li>Sensor data needed</li>
        <li>Pre-study pilot</li>
      </ul>

      <div className="mt-8 flex justify-start">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/data-and-analysis">
            Previous: Data & Analysis
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
