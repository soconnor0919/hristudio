import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function DataAndAnalysisTutorial() {
  return (
    <TutorialPage
      title="Data & Analysis"
      description="Collect and export trial data"
      duration="15 min"
      level="Intermediate"
      steps={[
        { title: "Understand data collection", description: "" },
        { title: "Access trial data", description: "" },
        { title: "Export data formats", description: "" },
        { title: "Use the analytics dashboard", description: "" },
        { title: "Generate reports", description: "" },
      ]}
      prevTutorial={{
        title: "Forms & Surveys",
        href: "/help/tutorials/forms-and-surveys",
      }}
      nextTutorial={{
        title: "Simulation Mode",
        href: "/help/tutorials/simulation-mode",
      }}
    >
      <h2>Data Collection Overview</h2>
      <p>HRIStudio automatically captures comprehensive data during trials:</p>
      <pre><code>Trial Data
├── Trial Metadata
│   ├── Start/End times
│   ├── Duration
│   ├── Participant info
│   └── Experiment version
├── Event Log (Timestamped)
│   ├── Step changes
│   ├── Action executions
│   ├── Robot responses
│   └── Wizard interventions
├── Form Responses
│   ├── Consent forms
│   ├── Surveys
│   └── Questionnaires
└── Sensor Data
    ├── Joint positions
    ├── Touch events
    └── Audio/video (if enabled)</code></pre>

      <h2>Event Types</h2>
      <table>
        <thead>
          <tr><th>Event Type</th><th>Description</th><th>Data Captured</th></tr>
        </thead>
        <tbody>
          <tr><td>trial_started</td><td>Trial began</td><td>Timestamp</td></tr>
          <tr><td>step_changed</td><td>New step began</td><td>Step ID, name</td></tr>
          <tr><td>action_executed</td><td>Robot action</td><td>Action details, duration</td></tr>
          <tr><td>wizard_response</td><td>Wizard decision</td><td>Selected option</td></tr>
          <tr><td>intervention</td><td>Wizard intervention</td><td>Type, note</td></tr>
          <tr><td>trial_completed</td><td>Trial finished</td><td>Summary</td></tr>
        </tbody>
      </table>

      <h2>Step 1: Accessing Trial Data</h2>

      <h3>From Trial List</h3>
      <ol>
        <li>Go to <strong>Trials</strong> tab</li>
        <li>Find completed trial</li>
        <li>Click <strong>View Details</strong></li>
      </ol>

      <h3>From Study Dashboard</h3>
      <ol>
        <li>Open your study</li>
        <li>Go to <strong>Data</strong> tab</li>
        <li>Select trial or view aggregate</li>
      </ol>

      <h2>Step 2: Exporting Data</h2>

      <h3>Export Single Trial</h3>
      <ol>
        <li>Open trial details</li>
        <li>Click <strong>Export</strong></li>
        <li>Select format</li>
      </ol>

      <h3>Export Study Data</h3>
      <ol>
        <li>Open study</li>
        <li>Go to <strong>Data</strong> tab</li>
        <li>Click <strong>Export All</strong></li>
        <li>Select options:
          <ul>
            <li>Date range</li>
            <li>Trial status</li>
            <li>Include forms</li>
          </ul>
        </li>
      </ol>

      <h3>Export Formats</h3>
      <table>
        <thead>
          <tr><th>Format</th><th>Contents</th></tr>
        </thead>
        <tbody>
          <tr><td>CSV</td><td>Tabular data for spreadsheets</td></tr>
          <tr><td>JSON</td><td>Full event log with metadata</td></tr>
          <tr><td>Video</td><td>Screen recording (if enabled)</td></tr>
        </tbody>
      </table>

      <h2>Step 3: Analytics Dashboard</h2>
      <p>View aggregate statistics:</p>
      <ul>
        <li><strong>Total Trials</strong> - Number of scheduled trials</li>
        <li><strong>Completed</strong> - Successfully completed trials</li>
        <li><strong>Average Duration</strong> - Mean trial time</li>
        <li><strong>Completion Rate</strong> - % of trials completed</li>
        <li><strong>Failed</strong> - Trials that failed</li>
      </ul>

      <h2>Step 4: Analyzing Event Data</h2>

      <h3>Timing Analysis</h3>
      <p>Calculate action durations from event log:</p>
      <pre><code>{`for event in events:
    if event.type == 'action_executed':
        duration = event.get('duration', 0)
        print(f"{event.actionName}: {duration/1000:.1f}s")`}</code></pre>

      <h3>Intervention Analysis</h3>
      <p>Track wizard interventions:</p>
      <pre><code>{`interventions = [e for e in events if e.type == 'intervention']

by_type = {}
for i in interventions:
    itype = i.data.get('type', 'unknown')
    by_type[itype] = by_type.get(itype, 0) + 1`}</code></pre>

      <h2>Step 5: Generating Reports</h2>

      <h3>Trial Summary Report</h3>
      <p>Generate PDF summary with:</p>
      <ul>
        <li>Executive summary</li>
        <li>Timeline of events</li>
        <li>Metrics and statistics</li>
        <li>Intervention summary</li>
      </ul>

      <h3>Study Report</h3>
      <p>Aggregate across participants:</p>
      <ul>
        <li>Participation rates</li>
        <li>Timing statistics</li>
        <li>Intervention totals</li>
        <li>Branch selection distribution</li>
      </ul>

      <h2>Data Privacy</h2>

      <h3>Anonymization</h3>
      <p>Remove identifying information:</p>
      <pre><code>{`participant_map = {
    'P001': 'S001',
    'P002': 'S002',
    'P003': 'S003',
}`}</code></pre>

      <h2>Best Practices</h2>
      <ul>
        <li>Export data regularly (daily/weekly)</li>
        <li>Store in secure location</li>
        <li>Follow IRB data retention</li>
        <li>Backup critical data</li>
      </ul>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/forms-and-surveys">
            Previous: Forms & Surveys
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/simulation-mode">
            Next: Simulation Mode
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
