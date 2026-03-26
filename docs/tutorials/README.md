# HRIStudio Tutorials

Welcome to the HRIStudio tutorials! These guides will help you get up and running with the platform for your HRI research.

## Tutorial Overview

| Tutorial | Description | Time |
|----------|-------------|------|
| **[Getting Started](tutorials/01-getting-started.md)** | Installation, setup, and first login | 10 min |
| **[Your First Study](tutorials/02-your-first-study.md)** | Creating a study and adding team members | 15 min |
| **[Designing Experiments](tutorials/03-designing-experiments.md)** | Building experiment protocols with blocks | 25 min |
| **[Running Trials](tutorials/04-running-trials.md)** | Executing trials and managing participants | 20 min |
| **[Wizard Interface](tutorials/05-wizard-interface.md)** | Real-time trial control and monitoring | 15 min |
| **[Robot Integration](tutorials/06-robot-integration.md)** | Connecting NAO6 and other robots | 20 min |
| **[Forms & Surveys](tutorials/07-forms-and-surveys.md)** | Creating consent forms and questionnaires | 15 min |
| **[Data & Analysis](tutorials/08-data-and-analysis.md)** | Collecting and exporting trial data | 15 min |
| **[Simulation Mode](tutorials/09-simulation-mode.md)** | Testing without a physical robot | 10 min |

## Quick Navigation

### For Researchers
1. [Getting Started](tutorials/01-getting-started.md) - Set up your environment
2. [Your First Study](tutorials/02-your-first-study.md) - Create your study
3. [Designing Experiments](tutorials/03-designing-experiments.md) - Build your protocol
4. [Running Trials](tutorials/04-running-trials.md) - Execute your study
5. [Data & Analysis](tutorials/08-data-and-analysis.md) - Analyze results

### For Wizards
1. [Getting Started](tutorials/01-getting-started.md) - Basic setup
2. [Wizard Interface](tutorials/05-wizard-interface.md) - Control trials
3. [Robot Integration](tutorials/06-robot-integration.md) - Connect to robot

### For Administrators
1. [Getting Started](tutorials/01-getting-started.md) - Full setup
2. [Robot Integration](tutorials/06-robot-integration.md) - Configure robots
3. [Forms & Surveys](tutorials/07-forms-and-surveys.md) - Manage templates

## Common Workflows

### Basic HRI Experiment
```
Create Study → Design Experiment → Add Participants → Run Trials → Collect Data
```

### Wizard-of-Oz Study
```
Create Study → Design Experiment with Wizard Blocks → Configure Robot → 
Add Wizards → Run Trials with Live Control → Collect Data
```

### Pilot Testing
```
Create Study → Design Experiment → Enable Simulation Mode → Run Test Trials → 
Refine Protocol → Connect Real Robot → Run Study
```

## Prerequisites

- **For local development**: Bun, Docker, PostgreSQL
- **For robot studies**: NAO6 robot or compatible robot
- **For cloud deployment**: Vercel, Cloudflare R2, PostgreSQL database

## Getting Help

- Check the [Quick Reference](../quick-reference.md) for common commands
- Review the [Implementation Guide](../implementation-guide.md) for technical details
- Visit the [NAO6 Integration](../nao6-quick-reference.md) for robot-specific help

---

**Next**: [Getting Started](tutorials/01-getting-started.md)
