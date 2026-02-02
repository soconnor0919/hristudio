# A Web-Based Wizard-of-Oz Platform for Collaborative and Reproducible Human-Robot Interaction Research

## 1) Introduction
- HRI needs rigorous methods for studying robot communication, collaboration, and coexistence with people.
- WoZ: a wizard remotely operates a robot to simulate autonomous behavior, enabling rapid prototyping and iterative refinement.
- Challenges with WoZ:
  - Wizard must execute scripted sequences consistently across participants.
  - Deviations and technical barriers reduce methodological rigor and reproducibility.
  - Many available tools require specialized technical expertise.
- Goal: a platform that lowers barriers to entry, supports rigorous, reproducible WoZ experiments, and provides integrated capabilities.

## 2) Assessment of the State-of-the-Art
- Technical infrastructure and architectures:
  - Polonius: ROS-based, finite-state machine scripting, integrated logging for real-time event recording; designed for non-programming collaborators.
  - OpenWoZ: runtime-configurable, multi-client, supports distributed operation and dynamic evaluator interventions (requires programming for behavior creation).
- Interface design and user experience:
  - NottReal: interface for voice UI studies; tabbed pre-scripted messages, customization slots, message queuing, comprehensive logging, familiar listening/processing feedback.
  - WoZ4U: GUI designed for non-programmers; specialized to Aldebaran Pepper (limited generalizability).
- Domain specialization vs. generalizability:
  - System longevity is often short (2–3 years for general-purpose tools).
  - Ozlab’s longevity due to: general-purpose design, curricular integration, flexible wizard UI that adapts to experiments.
- Standardization and methodological approaches:
  - Interaction Specification Language (ISL) and ADEs (Porfirio et al.): hierarchical modularity, formal representations, platform independence for reproducibility.
  - Riek: methodological transparency deficiencies in WoZ literature (insufficient reporting of protocols/training/constraints).
  - Steinfeld et al.: “Oz of Wizard” complements WoZ; structured permutations of real vs. simulated components; both approaches serve valid objectives.
  - Belhassein et al.: recurring HRI study challenges (limited participants, inadequate protocol reporting, weak replication); need for validated measures and comprehensive documentation.
  - Fraune et al.: practical guidance (pilot testing, ensuring intended perception of robot behaviors, managing novelty effects, cross-field collaboration).
- Remaining challenges:
  - Accessibility for interdisciplinary teams.
  - Methodological standardization and comprehensive data capture/sharing.
  - Balance of structure (for reproducibility) and flexibility (for diverse research questions).

## 3) Reproducibility Challenges in WoZ Studies
- Inconsistent wizard behavior across trials undermines reproducibility.
- Publications often omit critical procedural details, making replication difficult.
- Custom, ad-hoc setups are hard to recreate; unrecorded changes hinder transparency.
- HRIStudio’s reproducibility requirements (five areas):
  - Standardized terminology and structure.
  - Wizard behavior formalization (clear, consistent execution with controlled flexibility).
  - Comprehensive, time-synchronized data capture.
  - Experiment specification sharing (package and distribute complete designs).
  - Procedural documentation (automatic logging of parameters and methodological details).

## 4) The Design and Architecture of HRIStudio
- Guiding design principles:
  - Accessibility for researchers without deep robot programming expertise.
  - Abstraction to focus on experimental design over platform details.
  - Comprehensive data management (logs, audio, video, study materials).
  - Collaboration through multi-user accounts, role-based access control, and data sharing.
  - Embedded methodological guidance to encourage scientifically sound practices.
- Conceptual separation aligned to research needs:
  - User-facing tools for design, execution, and analysis; stewarded data and access control; and standardized interfaces to connect experiments with robots and sensors.
- Three-layer architecture [Screenshot Placeholder: Architecture Overview]:
  - User Interface Layer:
    - Experiment Designer (visual programming for specifying experiments).
    - Wizard Interface (real-time control for trials).
    - Playback & Analysis (data exploration and visualization).
  - Data Management Layer:
    - Structured storage of experiment definitions, metadata, and media.
    - Role-based access aligned with study responsibilities.
    - Collaboration with secure, compartmentalized access for teams.
  - Robot Integration Layer:
    - Translates standardized abstractions to robot behaviors through plugins.
    - Standardized plugin interfaces support diverse platforms without changing study designs.
    - Integrates with external systems (robot hardware, sensors, tools).
- Sustained reproducibility and sharing:
  - Study definitions and execution environments can be packaged and shared to support faithful reproduction by independent teams.

## 5) Experimental Workflow Support
- Directly addresses reproducibility requirements with standardized structures, wizard guidance, and comprehensive capture.

### 5.1 Hierarchical Structure for WoZ Studies
- Standard terminology and elements:
  - Study: top-level container with one or more experiments.
  - Experiment: parameterized protocol template composed of steps.
  - Trial: concrete, executable instance of an experiment for a specific participant; all trial data recorded.
  - Step: type-bound container (wizard or robot) comprising a sequence of actions.
  - Action: atomic task for wizard or robot (e.g., input gathering, speech, movement), parameterized per trial.
- [Screenshot Placeholder: Experiment Hierarchy Diagram].
- [Screenshot Placeholder: Study Details View]:
  - Overview of execution summaries, trials, participant info and documents (e.g., consent), members, metadata, and audit activity.

### 5.2 Collaboration and Knowledge Sharing
- Dashboard for project overview, collaborators, trial schedules, pending tasks.
- Role-based access control (pre-defined roles; flexible extensions):
  - Administrator: system configuration/management.
  - Researcher: create/configure studies and experiments.
  - Observer: read-only access and real-time monitoring.
  - Wizard: execute experiments.
- Packaging and dissemination of complete materials for replication and meta-analyses.

### 5.3 Visual Experiment Design (EDE)
- Visual programming canvas for sequencing steps and actions (drag-and-drop).
- Abstract robot actions translated by plugins into platform-specific commands.
- Contextual help and documentation in the interface.
- [Screenshot Placeholder: Experiment Designer].
- Inspiration: Choregraphe’s flow-based, no-code composition for steps/actions.

### 5.4 Wizard Interface and Experiment Execution
- Adaptable, experiment-specific wizard UI (avoids one-size-fits-all trap).
- Incremental instructions, “View More” for full script, video feed, timestamped event log, and “quick actions.”
- Observer view mirrors wizard interface without execution controls.
- Action execution process:
  1) Translate abstract action into robot-specific calls via plugin.
  2) Route calls through appropriate communication channels.
  3) Process robot feedback, log details, update experiment state.
- [Screenshot Placeholder: Wizard Interface].

### 5.5 Robot Platform Integration (Plugin Store)
- Two-tier abstraction/translation of actions:
  - High-level action components (movement, speech, sensors) with parameter schemas and validation rules.
  - Robot plugins implement concrete mappings appropriate to each platform.
- [Screenshot Placeholder: Plugin Store]:
  - Trust levels: Official, Verified, Community.
  - Source repositories for precise version tracking and reproducibility.

### 5.6 Comprehensive Data Capture and Analysis
- Timestamped logs of all executed actions and events.
- Robot sensor data (position, orientation, sensor readings).
- Audio/video recordings of interactions.
- Wizard decisions/interventions (including unplanned deviations).
- Observer notes and annotations.
- Structured storage for long-term preservation and analysis integration.
- Sensitive participant data encrypted at the database level.
- Playback for step-by-step trial review and annotation.

## 6) Conclusion and Future Directions
- HRIStudio supports rigorous, reproducible WoZ experimentation via:
  - Standardized hierarchy and terminology.
  - Visual designer for protocol specification.
  - Configurable wizard interface for consistent execution.
  - Plugin-based, robot-agnostic integration.
  - Comprehensive capture and structured storage of multimodal data.
- Future directions:
  - Interface-integrated documentation for installation and operation.
  - Enhanced execution and analysis (advanced guidance, dynamic adaptation, real-time feedback).
  - Playback for synchronized streams and expanded hardware integration.
  - Continued community engagement to refine integration with existing research infrastructures and workflows.
  - Preparation for an open beta release.
