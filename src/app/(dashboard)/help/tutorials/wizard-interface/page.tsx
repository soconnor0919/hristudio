import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function WizardInterfaceTutorial() {
  return (
    <TutorialPage
      title="Wizard Interface"
      description="Real-time trial control and monitoring"
      duration="15 min"
      level="Intermediate"
      steps={[
        { title: "Access the wizard interface", description: "" },
        { title: "Understand the layout", description: "" },
        { title: "Control robot actions", description: "" },
        { title: "Make branching decisions", description: "" },
        { title: "Handle interruptions", description: "" },
      ]}
      prevTutorial={{
        title: "Running Trials",
        href: "/help/tutorials/running-trials",
      }}
      nextTutorial={{
        title: "Robot Integration",
        href: "/help/tutorials/robot-integration",
      }}
    >
      <h2>What is the Wizard Interface?</h2>
      <p>The <strong>Wizard Interface</strong> is your control center during trials. It provides:</p>
      <ul>
        <li>Real-time trial monitoring</li>
        <li>Robot action controls</li>
        <li>Decision-making tools</li>
        <li>Intervention capabilities</li>
        <li>Event logging</li>
      </ul>

      <h2>Step 1: Accessing the Interface</h2>

      <h3>Method 1: From Trials List</h3>
      <ol>
        <li>Go to <strong>Trials</strong> in sidebar</li>
        <li>Find your scheduled trial</li>
        <li>Click <strong>Open Wizard</strong></li>
      </ol>

      <h3>Method 2: Direct URL</h3>
      <pre><code>{`/trials/{trialId}/wizard`}</code></pre>

      <h3>Method 3: Trial Queue</h3>
      <ol>
        <li>Go to <strong>Wizard Queue</strong></li>
        <li>See all pending trials</li>
        <li>Click <strong>Start</strong> on any trial</li>
      </ol>

      <h2>Step 2: Understanding the Layout</h2>

      <h3>Left Panel: Trial Controls</h3>
      <table>
        <thead>
          <tr><th>Control</th><th>Function</th></tr>
        </thead>
        <tbody>
          <tr><td>Play/Pause</td><td>Start or pause trial</td></tr>
          <tr><td>Stop</td><td>End trial early</td></tr>
          <tr><td>Notes</td><td>Add timestamped observations</td></tr>
          <tr><td>Alert</td><td>Send alert to researchers</td></tr>
        </tbody>
      </table>

      <h3>Center Panel: Timeline</h3>
      <ul>
        <li><strong>Visual Progress</strong> - See step progression</li>
        <li><strong>Current Position</strong> - Highlighted current step</li>
        <li><strong>Time Display</strong> - Elapsed and estimated remaining</li>
      </ul>

      <h3>Right Panel: Robot Control</h3>
      <ul>
        <li><strong>Status Section</strong> - Connection, battery, position</li>
        <li><strong>Action Section</strong> - Quick action buttons</li>
      </ul>

      <h2>Step 3: Controlling the Robot</h2>

      <h3>Quick Actions</h3>
      <p>Pre-configured robot actions:</p>
      <table>
        <thead>
          <tr><th>Action</th><th>Description</th></tr>
        </thead>
        <tbody>
          <tr><td>Say Text</td><td>Make robot speak</td></tr>
          <tr><td>Wave</td><td>Wave gesture</td></tr>
          <tr><td>Look at Me</td><td>Turn head toward participant</td></tr>
          <tr><td>Nod</td><td>Confirmation nod</td></tr>
        </tbody>
      </table>

      <h3>Custom Say Text</h3>
      <ol>
        <li>Click <strong>Say Text</strong></li>
        <li>Enter text in popup</li>
        <li>Select options (speed, emotion)</li>
        <li>Click <strong>Execute</strong></li>
      </ol>

      <h2>Step 4: Making Decisions</h2>
      <p>When the experiment reaches a branching point:</p>
      <ol>
        <li><strong>Observe</strong> participant&apos;s actual response</li>
        <li><strong>Consider</strong> protocol criteria</li>
        <li><strong>Select</strong> appropriate branch</li>
        <li><strong>Confirm</strong> selection</li>
      </ol>
      <p>Decision is logged with timestamp and trial continues.</p>

      <h2>Step 5: Handling Interruptions</h2>

      <h3>Pause Trial</h3>
      <ol>
        <li>Click <strong>Pause</strong> button</li>
        <li>Add reason (optional)</li>
        <li>Trial pauses, robot holds position</li>
      </ol>

      <h3>Resume Trial</h3>
      <ol>
        <li>Click <strong>Play</strong> button</li>
        <li>Trial resumes from pause point</li>
        <li>Pause duration is logged</li>
      </ol>

      <h3>Stop Trial</h3>
      <ol>
        <li>Click <strong>Stop</strong> button</li>
        <li>Select reason</li>
        <li>Confirm stop</li>
        <li>Partial data is saved</li>
      </ol>

      <h2>Keyboard Shortcuts</h2>
      <table>
        <thead>
          <tr><th>Key</th><th>Action</th></tr>
        </thead>
        <tbody>
          <tr><td>Space</td><td>Play/Pause toggle</td></tr>
          <tr><td>Escape</td><td>Stop trial</td></tr>
          <tr><td>N</td><td>Add note</td></tr>
          <tr><td>A</td><td>Send alert</td></tr>
        </tbody>
      </table>

      <h2>Event Logging</h2>
      <p>All actions are logged automatically:</p>
      <pre><code>[14:32:05] Trial started
[14:32:08] Step 1: The Hook
[14:32:10] Action: Say Text &quot;Hello!&quot;
[14:33:28] Wizard Note: &quot;Participant engaged&quot;
[14:33:30] Branch: Correct selected
[14:34:05] Trial completed</code></pre>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/running-trials">
            Previous: Running Trials
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/robot-integration">
            Next: Robot Integration
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
