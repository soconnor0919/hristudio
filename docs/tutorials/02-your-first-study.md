# Tutorial 2: Your First Study

Learn how to create a research study and configure team access.

## Objectives

- Create a new research study
- Configure study settings (IRB, institution)
- Add team members with appropriate roles

## What is a Study?

In HRIStudio, a **Study** is the top-level container for your research:

```
Study
├── Experiments (multiple protocols)
├── Participants (study participants)
├── Team Members (collaborators)
├── Forms & Surveys (consent, questionnaires)
└── Trials (individual experiment runs)
```

## Step 1: Create a New Study

1. Log in as **Researcher** or **Administrator**
2. Click **Studies** in the sidebar
3. Click **Create Study**

### Study Settings

| Field | Description | Required |
|-------|-------------|----------|
| Name | Study title | Yes |
| Description | Brief overview of research goals | Yes |
| Institution | University or organization | No |
| IRB Protocol | Protocol number (e.g., 2024-HRI-001) | No |
| Status | Draft, Active, Completed, Archived | Yes |

### Example: Creating "Robot Trust Study"

```
Name: Robot Trust Study
Description: Investigating how robot appearance affects human trust in collaborative tasks.
Institution: Bucknell University
IRB Protocol: 2024-HRI-TRUST
Status: Draft
```

## Step 2: Add Team Members

Studies can have multiple collaborators with different roles:

| Role | Permissions |
|------|-------------|
| Owner | Full access, can delete study |
| Researcher | Create/edit experiments, manage participants |
| Wizard | Execute trials, control robot |
| Observer | View-only access, add annotations |

### Adding a Wizard

1. Open your study
2. Go to **Team** tab
3. Click **Add Member**
4. Enter the wizard's email
5. Select **Wizard** role
6. Click **Invite**

The wizard will receive access to:
- View the study and experiments
- Execute trials
- Control the robot during trials
- Add notes to trials

## Step 3: Install Robot Plugins

For studies involving robots, you need to install the appropriate plugin:

1. Go to **Plugins** in the sidebar
2. Select your study from the dropdown
3. Click **Browse Plugins**
4. Find your robot (e.g., "NAO6 Robot (ROS2 Integration)")
5. Click **Install**
6. Configure robot settings (IP address, etc.)

### Plugin Configuration

For NAO6 robots:

```
Robot IP: 192.168.1.100
Connection Type: ROS2 Bridge
WebSocket URL: ws://localhost:9090
```

## Step 4: Create Forms

Before running trials, you need consent forms:

1. Go to **Forms** tab in your study
2. Click **Create Form**
3. Select form type:
   - **Consent** - Informed consent documents
   - **Survey** - Post-session questionnaires
   - **Questionnaire** - Demographic forms

### Form Templates

HRIStudio provides templates to get started:

| Template | Use Case |
|----------|----------|
| Informed Consent | Required for all participants |
| Post-Session Survey | Collect feedback after trials |
| Demographics | Collect participant information |

## Step 5: Add Participants

1. Go to **Participants** tab
2. Click **Add Participant**
3. Enter participant code (e.g., "P001")
4. Fill in optional details

### Batch Import

For large studies, import from CSV:

```csv
participantCode,name,email,notes
P001,John Smith,john@email.com,Condition A
P002,Jane Doe,jane@email.com,Condition B
```

## Study Workflow

```
Draft → Active → Recruiting → In Progress → Completed
  │        │          │            │           │
  │        │          │            │           └── All trials done
  │        │          │            └── Trials running
  │        │          └── Recruiting participants
  │        └── Ready to collect data
  └── Setting up study
```

## Study Settings Deep Dive

### IRB Compliance

Store your IRB information:
- Protocol number
- Approval date
- Expiration date
- Consent form versions

### Data Management

Configure data retention:
- Anonymization settings
- Export formats (CSV, JSON)
- Backup frequency

### Notification Settings

Configure alerts for:
- Trial completion
- Participant issues
- Robot disconnection

## Common Tasks

### Clone a Study

Create a copy of an existing study:

1. Open the study
2. Click **Settings** (gear icon)
3. Select **Duplicate Study**
4. Enter new study name

### Archive a Study

When a study is complete:

1. Go to study settings
2. Change status to **Archived**
3. Data is preserved but study is read-only

### Transfer Ownership

Change the study owner:

1. Go to **Team** tab
2. Find the new owner
3. Click **Make Owner**

## Troubleshooting

### Can't Add Team Member

- Check email is correct
- User must have an HRIStudio account
- You must be an owner or admin

### Plugin Installation Failed

- Check robot is on the network
- Verify WebSocket URL is correct
- Check Docker services are running

## Next Steps

Now that your study is set up:

1. **[Designing Experiments](03-designing-experiments.md)** - Create your first experiment protocol
2. **[Forms & Surveys](07-forms-and-surveys.md)** - Customize your consent forms
3. **[Running Trials](04-running-trials.md)** - Learn about trial management

---

**Previous**: [Getting Started](01-getting-started.md) | **Next**: [Designing Experiments](03-designing-experiments.md)
