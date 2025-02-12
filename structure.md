# HRIStudio Structure and Requirements

## Structure

A *study* is a general term for a research project.

An *experiment* is a specific set of steps and actions that will be conducted with a participant and robot. Experiments are designed and configured via a dedicated drag and drop experiment designer. This interactive designer features a dotted background—similar to Unreal Engine's IDE drag and drop area—that clearly indicates drop zones. Users can add, reorder, and connect individual steps and actions visually.

An *trial* is a specific instance of an experiment. It is a single run of the experiment with a specific participant and robot.

A *step* is a general term for something that is being done in the experiment. It is represented as a collection of actions that are being done in a specific order.

An *action* is a specific operation that is being done (like "move to position", "press button", "say something", etc.) These are the smallest atomic units of the platform.

A *participant* is a person that has been added to a study. This person does not have an account.

A *user* is a person that has an account, which is a person that has been added to a study. Anyone can sign up for an account, but they must be added to a study or create their own. A user can have different roles in different studies.

## Experiment Design and Implementation

Experiments are central to HRIStudio and are managed with full CRUD operations. The Experiment Design feature includes:

- **Drag and Drop Designer:** An interactive design area with a dotted background, reminiscent of Unreal Engine's IDE, which allows users to visually add, reposition, and connect steps and actions. The designer includes:
  - A dotted grid background that provides visual cues for alignment and spacing
  - Highlighted drop zones that activate when dragging components
  - Visual feedback for valid/invalid drop targets
  - Smooth animations for reordering and nesting
  - Connection lines showing relationships between steps
  - A side panel of available actions that can be dragged into steps
- **Experiment Templates:** The ability to save and reuse experiment configurations.
- **CRUD Operations:** Procedures to create, retrieve, update, and delete experiments associated with a study.
- **Dynamic Interaction:** Support for adding and reordering steps, and nesting actions within steps.

## Roles and Permissions

### Core Roles

1. **Owner**
   - Single owner per study
   - Full control over all aspects of the study
   - Can delete study or transfer ownership
   - Can manage all other roles
   - Usually the study creator or designated successor
   - Cannot be removed except through ownership transfer

2. **Admin**
   - Multiple admins allowed
   - Can manage participants, experiments, and study settings
   - Can invite and manage other users (except Owner)
   - Cannot delete study or transfer ownership
   - Appointed by Owner

3. **Principal Investigator (PI)**
   - Scientific oversight role
   - Full access to participant data and experiment design
   - Can manage experiment protocols
   - Can analyze and export all data
   - Cannot modify core study settings or manage user roles
   - Typically one PI per study

4. **Wizard**
   - Operates the robot during experiment trials
   - Can control live experiment sessions
   - Can view anonymized participant data
   - Can annotate experiments in real-time
   - Cannot modify study design or access sensitive participant data
   - Multiple wizards allowed

5. **Researcher**
   - Can view and analyze experiment data
   - Can access anonymized participant information
   - Can export and analyze results
   - Cannot modify study design or participant data
   - Cannot run experiment trials
   - Multiple researchers allowed

6. **Observer**
   - Can view live experiments
   - Can view anonymized participant data
   - Can add annotations
   - Cannot modify any study aspects
   - Cannot access sensitive data
   - Multiple observers allowed

### Permission Categories

1. **Study Management**
   - Create/Delete Study (Owner only)
   - Edit Study Settings
   - Transfer Ownership (Owner only)
   - Manage Study Metadata

2. **Participant Management**
   - Add/Remove Participants
   - View Participant Details (identifiable vs. anonymized)
   - Edit Participant Information
   - Manage Participant Consent Forms

3. **Experiment Design**
   - Create/Edit Experiment Templates
   - Define Steps and Actions
   - Set Robot Behaviors
   - Configure Data Collection

4. **Experiment Execution**
   - Run Experiment Trials
   - Control Robot Actions
   - Monitor Live Sessions
   - Add Real-time Annotations

5. **Data Access**
   - View Raw Data
   - View Anonymized Data
   - Export Data
   - Access Participant Identifiable Information

6. **User Management**
   - Invite Users
   - Assign Roles
   - Remove Users
   - Manage Permissions

### Role-Permission Matrix

| Permission Category   | Owner | Admin | PI  | Wizard | Researcher | Observer |
|-----------------------|-------|-------|-----|--------|------------|----------|
| Study Management      | Full  | Most  | No  | No     | No         | No       |
| Participant Management| Full  | Full  | Full| Limited| Limited    | View Only|
| Experiment Design     | Full  | Full  | Full| No     | No         | No       |
| Experiment Execution  | Full  | Full  | Full| Full   | View Only  | View Only|
| Data Access           | Full  | Full  | Full| Limited| Limited    | Limited  |
| User Management       | Full  | Most  | No  | No     | No         | No       |

### Special Considerations

1. **Data Privacy**
   - Identifiable participant information is only accessible to Owner, Admin, and PI roles
   - All other roles see anonymized data
   - Audit logs track all data access

2. **Role Hierarchy**
   - Owner > Admin > PI > Wizard/Researcher > Observer
   - Higher roles inherit permissions from lower roles
   - Certain permissions (like study deletion) are restricted to specific roles

3. **Role Assignment**
   - Users can have different roles in different studies
   - One user cannot hold multiple roles in the same study
   - Role changes are logged and require appropriate permissions

Participant Management: can create, update, delete participants, as well as view their personal information
- Admin: can do everything
- Principal Investigator: can do everything
- Wizard: can view participants, but cannot view their personal information
- Researcher: can view participants, but cannot view their personal information

Experiment Management: can create, update, delete experiments, as well as view their data and results.

- Admin: Can do everything
- Principal Investigator: Can do everything 
- Wizard: Runs experiment trials, can view results
- Researcher: Can view results

Experiment Design: can create, update, delete steps and actions, as well as specify general parameters for the experiment.

- Admin: Can do everything
- Principal Investigator: Can do everything
- Wizard: Can create, update, delete steps and actions, as well as specify general parameters for the experiment
- Researcher: Can view steps and actions.