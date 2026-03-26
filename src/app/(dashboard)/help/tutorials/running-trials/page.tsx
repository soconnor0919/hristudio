import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function RunningTrialsTutorial() {
  return (
    <TutorialPage
      title="Running Trials"
      description="Execute experiments and manage participant trials"
      duration="20 min"
      level="Intermediate"
      steps={[
        { title: "Schedule a trial", description: "" },
        { title: "Prepare for trial execution", description: "" },
        { title: "Start and monitor the trial", description: "" },
        { title: "Handle interventions", description: "" },
        { title: "Complete and review the trial", description: "" },
      ]}
      prevTutorial={{
        title: "Designing Experiments",
        href: "/help/tutorials/designing-experiments",
      }}
      nextTutorial={{
        title: "Wizard Interface",
        href: "/help/tutorials/wizard-interface",
      }}
    >
      <h2>What is a Trial?</h2>
      <p>A <strong>Trial</strong> is a single execution of an experiment with one participant:</p>
      <pre><code>Trial
├── Participant (who took part)
├── Experiment (which protocol)
├── Status (scheduled, in_progress, completed)
├── Events (timestamped actions)
└── Data (collected responses)</code></pre>

      <h2>Trial Lifecycle</h2>
      <pre><code>Scheduled → In Progress → Completed
    │            │            │
    │            ▼            │
    │        Aborted ◄────────┤
    │            │            │
    └────────► Failed ◄───────┘</code></pre>

      <h2>Step 1: Schedule a Trial</h2>
      <ol>
        <li>Go to your <strong>Study</strong></li>
        <li>Open <strong>Trials</strong> tab</li>
        <li>Click <strong>Schedule Trial</strong></li>
        <li>Select:
          <ul>
            <li><strong>Participant</strong>: P001</li>
            <li><strong>Experiment</strong>: The Interactive Storyteller</li>
            <li><strong>Scheduled Time</strong>: Today, 2:00 PM</li>
          </ul>
        </li>
      </ol>

      <h2>Step 2: Prepare for Trial</h2>
      <p>Before starting:</p>
      <ol>
        <li><strong>Verify Robot Connection</strong>
          <ul>
            <li>Check robot is powered on</li>
            <li>Verify network connection</li>
            <li>Test WebSocket connection</li>
          </ul>
        </li>
        <li><strong>Review Experiment</strong>
          <ul>
            <li>Ensure experiment is &quot;Ready&quot; status</li>
            <li>Check step count and timing</li>
            <li>Verify all actions are configured</li>
          </ul>
        </li>
        <li><strong>Prepare Environment</strong>
          <ul>
            <li>Ensure participant consent is obtained</li>
            <li>Set up recording equipment (if needed)</li>
            <li>Remove distractions</li>
          </ul>
        </li>
      </ol>

      <h2>Step 3: Start a Trial</h2>
      <p>From Trials List:</p>
      <ol>
        <li>Find the scheduled trial</li>
        <li>Click <strong>Start Trial</strong></li>
        <li>Confirm participant is ready</li>
        <li>Click <strong>Begin</strong></li>
      </ol>

      <h2>Step 4: During the Trial</h2>
      <p>The wizard interface provides:</p>
      <ul>
        <li><strong>Timeline View</strong> - Visual step progression</li>
        <li><strong>Current Step</strong> - Highlighted current step</li>
        <li><strong>Progress</strong> - Estimated time remaining</li>
        <li><strong>Event Log</strong> - Timestamped events</li>
      </ul>

      <h2>Step 5: Wizard Interventions</h2>
      <p>During Wizard-of-Oz studies, wizards can intervene:</p>

      <h3>Add Intervention</h3>
      <ol>
        <li>Click <strong>+ Intervention</strong></li>
        <li>Select type:
          <ul>
            <li><strong>Pause</strong>: Temporarily stop trial</li>
            <li><strong>Resume</strong>: Continue after pause</li>
            <li><strong>Note</strong>: Add observation</li>
            <li><strong>Alert</strong>: Send alert notification</li>
          </ul>
        </li>
      </ol>

      <h3>Branch Selection</h3>
      <p>When reaching a conditional step:</p>
      <ol>
        <li>Observe participant response</li>
        <li>Select appropriate branch</li>
        <li>Selection is logged for analysis</li>
      </ol>

      <h2>Step 6: Trial Completion</h2>

      <h3>Automatic Completion</h3>
      <p>When all steps complete:</p>
      <ol>
        <li>Final step executes</li>
        <li>Trial status → &quot;Completed&quot;</li>
        <li>Data is saved automatically</li>
        <li>Summary shown</li>
      </ol>

      <h3>Manual Completion</h3>
      <p>To end early:</p>
      <ol>
        <li>Click <strong>Stop Trial</strong></li>
        <li>Confirm completion</li>
        <li>Select reason</li>
        <li>Save partial data</li>
      </ol>

      <h2>Best Practices</h2>

      <h3>Before Trials</h3>
      <ul className="list-disc pl-6">
        <li>Robot connected and tested</li>
        <li>Experiment verified</li>
        <li>Participant consent obtained</li>
        <li>Recording equipment ready</li>
        <li>Wizard briefed on protocol</li>
      </ul>

      <h3>During Trials</h3>
      <ul className="list-disc pl-6">
        <li>Monitor timeline progress</li>
        <li>Take timestamped notes</li>
        <li>Document interventions</li>
        <li>Watch for issues</li>
      </ul>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/designing-experiments">
            Previous: Designing Experiments
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/wizard-interface">
            Next: Wizard Interface
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
