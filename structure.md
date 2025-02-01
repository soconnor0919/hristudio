# HRIStudio Structure and Requirements

## Structure

A *study* is a general term for a research project.

An *experiment* is a specific set of steps and actions that will be conducted with a participant and robot.

An *trial* is a specific instance of an experiment. It is a single run of the experiment with a specific participant and robot.

A *step* is a general term for something that is being done in the experiment. It is represented as a collection of actions that are being done in a specific order.

An *action* is a specific operation that is being done (like "move to position", "press button", "say something", etc.) These are the smallest atomic units of the platform.

A *participant* is a person that has been added to a study. This person does not have an account.

A *user* is a person that has an account, which is a person that has been added to a study. Anyone can sign up for an account, but they must be added to a study or create their own. A user can have one of many roles, but can only have one role at a time. They can be in one or more studies, allowing them to have different roles in different studies.

A *role* is a set of permissions that a user has in a study. A user can have one or more roles, but can only have one role at a time per study.

A *permission* is a specific action that a user can perform in a study. Permissions are grouped into categories, and each category has a set of permissions.

Roles and permissions:

An admin is a user with all permissions in a study. This is usually the creator of the study, but may not be the principal investigator.
The principal investigator is the PI of the project- may not be the creator of the study on the platform
A wizard is a user that can run experiment trials, and can view the results of the trials.
A researcher is a user that can view the results of the trials, and interpret the data.

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