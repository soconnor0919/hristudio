import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function YourFirstStudyTutorial() {
  return (
    <TutorialPage
      title="Your First Study"
      description="Create a research study and manage team members"
      duration="15 min"
      level="Beginner"
      steps={[
        { title: "Understand the Study structure", description: "" },
        { title: "Create a new study", description: "" },
        { title: "Add team members", description: "" },
        { title: "Install robot plugins", description: "" },
        { title: "Add participants", description: "" },
      ]}
      prevTutorial={{
        title: "Getting Started",
        href: "/help/tutorials/getting-started",
      }}
      nextTutorial={{
        title: "Designing Experiments",
        href: "/help/tutorials/designing-experiments",
      }}
    >
      <h2>What is a Study?</h2>
      <p>In HRIStudio, a <strong>Study</strong> is the top-level container for your research:</p>
      <pre><code>Study
├── Experiments (multiple protocols)
├── Participants (study participants)
├── Team Members (collaborators)
├── Forms & Surveys (consent, questionnaires)
└── Trials (individual experiment runs)</code></pre>

      <h2>Step 1: Create a New Study</h2>
      <ol>
        <li>Log in as <strong>Researcher</strong> or <strong>Administrator</strong></li>
        <li>Click <strong>Studies</strong> in the sidebar</li>
        <li>Click <strong>Create Study</strong></li>
      </ol>

      <h3>Study Settings</h3>
      <table>
        <thead>
          <tr>
            <th>Field</th>
            <th>Description</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Name</td>
            <td>Study title</td>
          </tr>
          <tr>
            <td>Description</td>
            <td>Brief overview of research goals</td>
          </tr>
          <tr>
            <td>Institution</td>
            <td>University or organization</td>
          </tr>
          <tr>
            <td>IRB Protocol</td>
            <td>Protocol number (e.g., 2024-HRI-001)</td>
          </tr>
          <tr>
            <td>Status</td>
            <td>Draft, Active, Completed, Archived</td>
          </tr>
        </tbody>
      </table>

      <h2>Step 2: Add Team Members</h2>
      <p>Studies can have multiple collaborators with different roles:</p>
      <table>
        <thead>
          <tr>
            <th>Role</th>
            <th>Permissions</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td>Owner</td>
            <td>Full access, can delete study</td>
          </tr>
          <tr>
            <td>Researcher</td>
            <td>Create/edit experiments, manage participants</td>
          </tr>
          <tr>
            <td>Wizard</td>
            <td>Execute trials, control robot during trials</td>
          </tr>
          <tr>
            <td>Observer</td>
            <td>View-only access, add annotations</td>
          </tr>
        </tbody>
      </table>

      <h3>Adding a Wizard</h3>
      <ol>
        <li>Open your study</li>
        <li>Go to <strong>Team</strong> tab</li>
        <li>Click <strong>Add Member</strong></li>
        <li>Enter the wizard&apos;s email</li>
        <li>Select <strong>Wizard</strong> role</li>
        <li>Click <strong>Invite</strong></li>
      </ol>

      <h2>Step 3: Install Robot Plugins</h2>
      <p>For studies involving robots, you need to install the appropriate plugin:</p>
      <ol>
        <li>Go to <strong>Plugins</strong> in the sidebar</li>
        <li>Select your study from the dropdown</li>
        <li>Click <strong>Browse Plugins</strong></li>
        <li>Find your robot (e.g., &quot;NAO6 Robot&quot;)</li>
        <li>Click <strong>Install</strong></li>
        <li>Configure robot settings (IP address, etc.)</li>
      </ol>

      <h2>Step 4: Add Participants</h2>
      <ol>
        <li>Go to <strong>Participants</strong> tab</li>
        <li>Click <strong>Add Participant</strong></li>
        <li>Enter participant code (e.g., &quot;P001&quot;)</li>
        <li>Fill in optional details</li>
      </ol>

      <h3>Batch Import</h3>
      <p>For large studies, import from CSV:</p>
      <pre><code>participantCode,name,email,notes
P001,John Smith,john@email.com,Condition A
P002,Jane Doe,jane@email.com,Condition B</code></pre>

      <h2>Study Workflow</h2>
      <pre><code>Draft → Active → Recruiting → In Progress → Completed
  │        │          │            │           │
  │        │          │            │           └── All trials done
  │        │          │            └── Trials running
  │        │          └── Recruiting participants
  │        └── Ready to collect data
  └── Setting up study</code></pre>

      <h2>Common Tasks</h2>
      
      <h3>Clone a Study</h3>
      <ol>
        <li>Open the study</li>
        <li>Click <strong>Settings</strong> (gear icon)</li>
        <li>Select <strong>Duplicate Study</strong></li>
        <li>Enter new study name</li>
      </ol>

      <h3>Archive a Study</h3>
      <p>When a study is complete:</p>
      <ol>
        <li>Go to study settings</li>
        <li>Change status to <strong>Archived</strong></li>
        <li>Data is preserved but study is read-only</li>
      </ol>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/getting-started">
            Previous: Getting Started
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/designing-experiments">
            Next: Designing Experiments
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
