# HRIStudio Project Backlog - Honors Thesis Research

## Project Overview

**Student**: Sean O'Connor  
**Thesis Title**: A Web-Based Wizard-of-Oz Platform for Collaborative and Reproducible Human-Robot Interaction Research  
**Timeline**: Fall 2025 - Spring 2026  
**Current Date**: September 23, 2025

## Current Status Assessment

### Platform Strengths
- **Core Platform Complete**: Production-ready backend with 12 tRPC routers, 31 database tables
- **Visual Designer**: Repository-based plugin system with 26+ core blocks  
- **Type Safety**: Clean TypeScript throughout, passes `bun typecheck`
- **Authentication**: Role-based access control (Admin, Researcher, Wizard, Observer)
- **Development Environment**: Comprehensive seed data and documentation

### Critical Gaps
- **Wizard Interface**: ✅ COMPLETE - Role-based views implemented (Wizard, Observer, Participant)
- **Robot Control**: Not working yet - core functionality missing (NEXT PRIORITY)
- **NAO6 Integration**: Cannot test without working robot control
- **Trial Execution**: WebSocket implementation needed for real-time functionality

### Platform Constraints
- **Device Target**: Laptop-only (no mobile/tablet optimization needed)
- **Robot Platform**: NAO6 humanoid robot
- **Study Focus**: Comparative usability study, not full platform development

## Academic Timeline

### Fall 2025 Semester
- **Current Date**: September 23, 2025
- **Goal**: Functional platform + IRB submission by end of December

### Winter Break
- **December - January 2026**: IRB approval process and final preparations

### Spring 2026 Semester  
- **January - February 2026**: User study execution (10-12 participants)
- **March 2026**: Data analysis and results drafting
- **April 2026**: Thesis defense preparation and execution
- **May 2026**: Final thesis submission

## Research Study Design

### Comparison Study
- **Control Group**: Choregraphe (manufacturer software for NAO6)
- **Experimental Group**: HRIStudio platform
- **Task**: Recreate well-documented HRI experiment from literature
- **Participants**: 10-12 non-engineering researchers (Psychology, Education, etc.)
- **Metrics**: Methodological consistency, user experience, completion times, error rates

### Success Criteria
- Functional wizard interface for real-time experiment control
- Reliable NAO6 robot integration
- Reference experiment implemented in both platforms
- Platform usable by non-programmers with minimal training
- Comprehensive data collection for comparative analysis

## Development Backlog by Timeline

### Phase 1: Core Development (September 23 - October 31, 2025)
**Goal**: Get essential systems working - 5-6 weeks available

#### Week 1-2: Foundation (Sept 23 - Oct 6)

**WIZARD-001: Wizard Interface Architecture** - ✅ COMPLETE (December 2024)
- **Story**: As a wizard, I need a functional interface to control experiments
- **Tasks**:
  - ✅ Design wizard interface wireframes and user flow
  - ✅ Implement three-panel layout (trial controls, execution view, monitoring)
  - ✅ Create role-based views (Wizard, Observer, Participant)
  - ✅ Build step navigation and progress tracking
  - ✅ Fix layout issues (double headers, bottom cut-off)
- **Deliverable**: Complete wizard interface with role-based views
- **Effort**: 12 days (completed)

**ROBOT-001: Robot Control Foundation** - CRITICAL (NEXT PRIORITY)
- **Story**: As a wizard, I need to send commands to NAO6 robot
- **Tasks**:
  - Research and implement NAO6 WebSocket connection
  - Create basic action execution engine
  - Implement mock robot mode for development
  - Build connection status monitoring
  - Integrate with existing wizard interface
- **Deliverable**: Robot connection established with basic commands
- **Effort**: 8 days (increased due to WebSocket server implementation needed)

**WEBSOCKET-001: Real-Time Infrastructure** - CRITICAL (NEW PRIORITY)
- **Story**: As a system, I need real-time communication between clients and robots
- **Tasks**:
  - Implement WebSocket server for real-time trial coordination
  - Create multi-client session management (wizard, observers, participants)
  - Build event broadcasting system for live trial updates
  - Add robust connection recovery and fallback mechanisms
- **Deliverable**: Working real-time infrastructure for trial execution
- **Effort**: 10 days

#### Week 3-4: Core Functionality (Oct 7 - Oct 20)

**ROBOT-002: Essential NAO6 Actions** - CRITICAL
- **Story**: As a wizard, I need basic robot actions for experiments
- **Tasks**:
  - Implement speech synthesis and playback
  - Add basic movement commands (walk, turn, sit, stand)
  - Create simple gesture library
  - Add LED color control
  - Implement error handling and recovery
  - Integrate with WebSocket infrastructure for real-time control
- **Deliverable**: NAO6 performs essential experiment actions reliably via wizard interface
- **Effort**: 10 days (increased due to real-time integration)

**TRIAL-001: Trial Execution Engine** - HIGH PRIORITY
- **Story**: As a wizard, I need to execute experiment protocols step-by-step
- **Tasks**:
  - ✅ Basic trial state machine exists (needs WebSocket integration)
  - Connect existing wizard interface to real-time execution
  - Enhance event logging with real-time broadcasting
  - Add manual intervention controls via WebSocket
  - Build trial completion and data export
- **Deliverable**: Complete trial execution with real-time data capture
- **Effort**: 8 days (integration with existing wizard interface)

#### Week 5-6: Integration & Testing (Oct 21 - Oct 31)

**INTEGRATION-001: End-to-End Workflow** - CRITICAL
- **Story**: As a researcher, I need complete workflow from design to execution
- **Tasks**:
  - Connect visual designer to trial execution
  - Test complete workflow: design → schedule → execute → analyze
  - Fix critical bugs and performance issues
  - Validate data consistency throughout pipeline
- **Deliverable**: Working end-to-end experiment workflow
- **Effort**: 8 days

### Phase 2: User Experience & Study Preparation (November 1-30, 2025)
**Goal**: Make platform usable and prepare study materials - 4 weeks available

#### Week 1-2: User Experience (Nov 1 - Nov 14)

**UX-001: Non-Programmer Interface** - HIGH PRIORITY
- **Story**: As a psychology researcher, I need intuitive tools to recreate experiments
- **Tasks**:
  - Simplify visual designer for non-technical users
  - Add contextual help and guided tutorials
  - Implement undo/redo functionality
  - Create error prevention and recovery mechanisms
  - Add visual feedback for successful actions
- **Deliverable**: Interface usable by non-programmers
- **Effort**: 10 days

#### Week 3-4: Study Foundation (Nov 15 - Nov 30)

**STUDY-001: Reference Experiment** - HIGH PRIORITY
- **Story**: As a researcher, I need a validated experiment for comparison study
- **Tasks**:
  - Select appropriate HRI experiment from literature
  - Implement in HRIStudio visual designer
  - Create equivalent Choregraphe implementation
  - Validate both versions work correctly
  - Document implementation decisions and constraints
- **Deliverable**: Reference experiment working in both platforms
- **Effort**: 8 days

**IRB-001: IRB Application Preparation** - CRITICAL
- **Story**: As a researcher, I need IRB approval for user study
- **Tasks**:
  - Draft complete IRB application
  - Create consent forms and participant materials
  - Design study protocols and procedures
  - Prepare risk assessment and mitigation plans
  - Design data collection and privacy protection measures
- **Deliverable**: Complete IRB application ready for submission
- **Effort**: 6 days

### Phase 3: Polish & IRB Submission (December 1-31, 2025)
**Goal**: Finalize platform and submit IRB - 4 weeks available

#### Week 1-2: Platform Validation (Dec 1 - Dec 14)

**VALIDATE-001: Platform Reliability** - CRITICAL
- **Story**: As a researcher, I need confidence the platform works reliably
- **Tasks**:
  - Conduct extensive testing with multiple scenarios
  - Fix any critical bugs or stability issues
  - Test on different laptop configurations (Mac, PC, browsers)
  - Validate data collection and export functionality
  - Performance optimization for laptop hardware
- **Deliverable**: Stable, reliable platform ready for study use
- **Effort**: 10 days

**TRAIN-001: Training Materials** - HIGH PRIORITY
- **Story**: As study participants, we need equivalent training for both platforms
- **Tasks**:
  - Create HRIStudio training workshop materials
  - Develop Choregraphe training equivalent
  - Record instructional videos
  - Create quick reference guides and cheat sheets
  - Design hands-on practice exercises
- **Deliverable**: Complete training materials for both platforms
- **Effort**: 4 days

#### Week 3-4: Final Preparations (Dec 15-31)

**IRB-002: IRB Submission** - CRITICAL
- **Story**: As a researcher, I need IRB approval to proceed with human subjects
- **Tasks**:
  - Finalize IRB application with all supporting materials
  - Submit to university IRB committee
  - Respond to any initial questions or clarifications
  - Prepare for potential revisions or additional requirements
- **Deliverable**: IRB application submitted and under review
- **Effort**: 2 days

**PILOT-001: Internal Pilot Testing** - HIGH PRIORITY
- **Story**: As a researcher, I need to validate study methodology
- **Tasks**:
  - Recruit 2-3 internal pilot participants from target demographic
  - Run complete study protocol with both platforms
  - Test wizard interface reliability during real sessions
  - Identify and fix any procedural issues
  - Refine training materials based on feedback
  - Document lessons learned and methodology improvements
- **Deliverable**: Validated study methodology ready for execution
- **Effort**: 6 days

### Phase 4: Study Execution (January - February 2026)
**Goal**: Execute user study with 10-12 participants

#### Study Preparation (January 2026)

**RECRUIT-001: Participant Recruitment**
- **Story**: As a researcher, I need to recruit qualified study participants
- **Tasks**:
  - Create participant screening survey
  - Recruit from Psychology, Education, and other non-engineering departments
  - Schedule study sessions to avoid conflicts
  - Send confirmation and preparation materials
- **Deliverable**: 10-12 confirmed participants scheduled
- **Effort**: Ongoing through January

**EXECUTE-001: Study Session Management**
- **Story**: As a researcher, I need reliable execution of each study session
- **Tasks**:
  - Create detailed session procedures and checklists
  - Implement real-time monitoring dashboard for study staff
  - Build backup procedures for technical failures
  - Create automated data validation after each session
  - Design post-session debriefing workflows
- **Deliverable**: Reliable study execution infrastructure
- **Effort**: 4 days

#### Data Collection (February 2026)

**DATA-001: Comprehensive Data Collection**
- **Story**: As a researcher, I need rich data for comparative analysis
- **Tasks**:
  - Automatic time-tracking for all participant actions
  - User interaction logging (clicks, errors, help usage)
  - Screen recording of participant sessions
  - Post-task survey integration
  - Experiment fidelity scoring system
- **Deliverable**: Complete behavioral and performance data
- **Effort**: Built into platform, minimal additional work

### Phase 5: Analysis & Writing (March - May 2026)
**Goal**: Analyze results and complete thesis

#### Analysis Tools (March 2026)

**ANALYSIS-001: Quantitative Analysis Support**
- **Story**: As a researcher, I need tools to analyze study results
- **Tasks**:
  - Implement automated experiment fidelity scoring
  - Build statistical comparison tools for platform differences
  - Create completion time and error rate analysis
  - Generate charts and visualizations for thesis
  - Export data in formats suitable for statistical software (R, SPSS)
- **Deliverable**: Analysis-ready data and initial results
- **Effort**: 5 days

#### Thesis Writing (March - May 2026)

**THESIS-001: Results and Discussion**
- **Tasks**:
  - Quantitative analysis of methodological consistency
  - Qualitative analysis of participant feedback
  - Statistical comparison of user experience metrics
  - Discussion of implications for HRI research
- **Deliverable**: Thesis chapters 4-5 (Results and Discussion)

**THESIS-002: Conclusion and Defense Preparation**
- **Tasks**:
  - Synthesis of research contributions
  - Limitations and future work discussion
  - Defense presentation preparation
  - Final thesis formatting and submission
- **Deliverable**: Complete thesis and successful defense

## Critical Success Factors

### End of December 2025 Must-Haves
1. **Functional Platform**: Wizard can execute experiments with NAO6 robot
2. **Reference Experiment**: Working implementation in both HRIStudio and Choregraphe
3. **User-Ready Interface**: Non-programmers can use with minimal training
4. **IRB Application**: Submitted and under review
5. **Training Materials**: Complete workshop materials for both platforms
6. **Pilot Validation**: Study methodology tested and refined

### Risk Mitigation Strategies

**Technical Risks**
- **Robot Hardware Failure**: Have backup NAO6 unit available, implement robust mock mode
- **Platform Stability**: Extensive testing across different laptop configurations
- **Data Loss**: Implement automatic session backup and recovery
- **Performance Issues**: Optimize for older laptop hardware

**Study Execution Risks**
- **Participant Recruitment**: Start early, have backup recruitment channels
- **Learning Curve**: Extensive pilot testing to refine training materials
- **Platform Comparison Fairness**: Ensure equivalent training quality for both platforms
- **IRB Delays**: Submit early with complete application to allow for revisions

**Timeline Risks**
- **Development Delays**: Focus on minimum viable features for research needs
- **Academic Calendar**: Align all deadlines with university schedule
- **Winter Break**: Use break time for IRB follow-up and final preparations

## Sprint Planning

### October Sprint (Core Development)
- **Total Development Days**: 28 days
- **Key Milestone**: Working wizard interface + robot control
- **Priority**: Technical foundation - everything depends on this

### November Sprint (User Experience & Study Prep)  
- **Total Development Days**: 24 days
- **Key Milestone**: Non-programmer ready interface + IRB draft
- **Priority**: Usability and study preparation

### December Sprint (Polish & Launch Prep)
- **Total Development Days**: 22 days  
- **Key Milestone**: IRB submitted + reliable platform
- **Priority**: Quality assurance and study readiness

### Buffer and Contingency
- **Built-in Buffer**: 10-15% buffer time in each sprint for unexpected issues
- **Parallel Workstreams**: IRB preparation can happen alongside platform development
- **Fallback Options**: Mock robot mode if hardware integration proves challenging
- **Academic Alignment**: All deadlines respect university calendar and requirements

## Success Metrics for Thesis Research

### Primary Research Outcomes
- **Methodological Consistency**: Quantitative fidelity scores comparing participant implementations to reference experiment
- **User Experience**: Task completion rates, error rates, time-to-completion, satisfaction scores
- **Accessibility**: Learning curve differences between platforms, help-seeking behavior
- **Efficiency**: Setup time, execution time, and total task completion time comparisons

### Platform Quality Gates
- Zero critical bugs during study sessions
- Sub-100ms response time for core wizard interface interactions  
- 100% data collection success rate across all study sessions
- Participant satisfaction score > 4.0/5.0 for HRIStudio usability
- Successful completion of reference experiment by 90%+ of participants

### Thesis Contributions
- **Empirical Evidence**: Quantitative comparison of WoZ platform approaches
- **Design Insights**: Specific recommendations for accessible HRI research tools
- **Methodological Framework**: Validated approach for comparing research software platforms
- **Open Source Contribution**: Functional platform available for broader HRI community

This backlog prioritizes research success over platform perfection, focusing on delivering the minimum viable system needed to conduct a rigorous comparative study while maintaining the scientific integrity required for honors thesis research.