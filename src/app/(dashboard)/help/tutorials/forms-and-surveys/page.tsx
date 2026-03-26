import { TutorialPage } from "~/components/ui/tutorial-page";
import { Button } from "~/components/ui/button";
import Link from "next/link";

export default function FormsAndSurveysTutorial() {
  return (
    <TutorialPage
      title="Forms & Surveys"
      description="Create consent forms and questionnaires"
      duration="15 min"
      level="Intermediate"
      steps={[
        { title: "Understand form types", description: "" },
        { title: "Create a new form", description: "" },
        { title: "Add form fields", description: "" },
        { title: "Use form templates", description: "" },
        { title: "Collect responses", description: "" },
      ]}
      prevTutorial={{
        title: "Robot Integration",
        href: "/help/tutorials/robot-integration",
      }}
      nextTutorial={{
        title: "Data & Analysis",
        href: "/help/tutorials/data-and-analysis",
      }}
    >
      <h2>Form Types</h2>
      <p>HRIStudio supports three form types:</p>
      <table>
        <thead>
          <tr><th>Type</th><th>Purpose</th><th>When</th></tr>
        </thead>
        <tbody>
          <tr><td>Consent</td><td>Informed consent for participation</td><td>Before trial</td></tr>
          <tr><td>Survey</td><td>Collect feedback and observations</td><td>After trial</td></tr>
          <tr><td>Questionnaire</td><td>Demographic data collection</td><td>Any time</td></tr>
        </tbody>
      </table>

      <h2>Step 1: Access Forms</h2>
      <ol>
        <li>Go to your <strong>Study</strong></li>
        <li>Click <strong>Forms</strong> tab</li>
        <li>View existing forms and templates</li>
      </ol>

      <h2>Step 2: Create a Form</h2>

      <h3>Using a Template</h3>
      <ol>
        <li>Click <strong>Create Form</strong></li>
        <li>Select <strong>Use Template</strong></li>
        <li>Choose template:
          <ul>
            <li>Informed Consent</li>
            <li>Post-Session Survey</li>
            <li>Demographics</li>
          </ul>
        </li>
        <li>Customize as needed</li>
      </ol>

      <h3>From Scratch</h3>
      <ol>
        <li>Click <strong>Create Form</strong></li>
        <li>Select <strong>Blank Form</strong></li>
        <li>Choose form type</li>
        <li>Build fields manually</li>
      </ol>

      <h2>Step 3: Form Field Types</h2>

      <table>
        <thead>
          <tr><th>Field Type</th><th>Description</th><th>Example</th></tr>
        </thead>
        <tbody>
          <tr><td>Text</td><td>Single line text input</td><td>Participant name</td></tr>
          <tr><td>Text Area</td><td>Multi-line text</td><td>Open-ended feedback</td></tr>
          <tr><td>Rating</td><td>Scale rating</td><td>Rate 1-5</td></tr>
          <tr><td>Multiple Choice</td><td>Select one option</td><td>Gender selection</td></tr>
          <tr><td>Yes/No</td><td>Binary choice</td><td>Consent checkbox</td></tr>
          <tr><td>Date</td><td>Date picker</td><td>Session date</td></tr>
          <tr><td>Signature</td><td>Digital signature</td><td>Consent signature</td></tr>
        </tbody>
      </table>

      <h2>Step 4: Consent Forms</h2>
      <p>For IRB compliance, consent forms must include:</p>
      <ul>
        <li>Study title and purpose</li>
        <li>Principal investigator</li>
        <li>Procedures description</li>
        <li>Risks and benefits</li>
        <li>Confidentiality statement</li>
        <li>Voluntary participation note</li>
        <li>Signature and date fields</li>
      </ul>

      <h2>Step 5: Distributing Forms</h2>

      <h3>Automatic Distribution</h3>
      <ol>
        <li>Open form settings</li>
        <li>Enable <strong>Auto-distribute</strong></li>
        <li>Set trigger:
          <ul>
            <li>Before trial (consent)</li>
            <li>After trial (survey)</li>
          </ul>
        </li>
        <li>Select participants</li>
      </ol>

      <h3>Manual Distribution</h3>
      <ol>
        <li>Open form</li>
        <li>Click <strong>Distribute</strong></li>
        <li>Select participants</li>
      </ol>

      <h2>Step 6: Collecting Responses</h2>

      <h3>View Responses</h3>
      <ol>
        <li>Open form</li>
        <li>Click <strong>Responses</strong> tab</li>
        <li>View individual submissions</li>
      </ol>

      <h3>Export Responses</h3>
      <p>Download collected data:</p>
      <table>
        <thead>
          <tr><th>Format</th><th>Contents</th></tr>
        </thead>
        <tbody>
          <tr><td>CSV</td><td>Tabular data</td></tr>
          <tr><td>JSON</td><td>Full response objects</td></tr>
          <tr><td>PDF</td><td>Printed consent forms</td></tr>
        </tbody>
      </table>

      <h2>Form Templates</h2>
      <p>Pre-built templates available:</p>
      <table>
        <thead>
          <tr><th>Template</th><th>Use Case</th></tr>
        </thead>
        <tbody>
          <tr><td>Standard Consent</td><td>Generic research consent</td></tr>
          <tr><td>Post-Session Survey</td><td>Post-session feedback</td></tr>
          <tr><td>Demographics</td><td>Participant information</td></tr>
        </tbody>
      </table>

      <div className="mt-8 flex justify-between">
        <Button variant="outline" asChild>
          <Link href="/help/tutorials/robot-integration">
            Previous: Robot Integration
          </Link>
        </Button>
        <Button asChild>
          <Link href="/help/tutorials/data-and-analysis">
            Next: Data & Analysis
          </Link>
        </Button>
      </div>
    </TutorialPage>
  );
}
