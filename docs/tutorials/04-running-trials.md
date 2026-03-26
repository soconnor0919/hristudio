# Tutorial 4: Running Trials

Learn how to execute experiments and manage participant trials.

## Objectives

- Schedule and start trials
- Monitor trial progress
- Handle trial interruptions
- Collect trial data

## What is a Trial?

A **Trial** is a single execution of an experiment with one participant:

```
Trial
├── Participant (who took part)
├── Experiment (which protocol)
├── Status (scheduled, in_progress, completed)
├── Events (timestamped actions)
└── Data (collected responses)
```

## Trial Lifecycle

```
Scheduled → In Progress → Completed
    │            │            │
    │            ▼            │
    │        Aborted ◄────────┤
    │            │            │
    └────────► Failed ◄───────┘
```

| Status | Description |
|--------|-------------|
| Scheduled | Trial is planned but not started |
| In Progress | Trial is currently running |
| Completed | Trial finished successfully |
| Aborted | Trial stopped early by wizard |
| Failed | Trial failed due to error |

## Step 1: Schedule a Trial

### Create Trial for Participant

1. Go to your **Study**
2. Open **Trials** tab
3. Click **Schedule Trial**
4. Select:
   - **Participant**: P001
   - **Experiment**: The Interactive Storyteller
   - **Scheduled Time**: Today, 2:00 PM

### Batch Scheduling

For multiple participants:

1. Click **Batch Schedule**
2. Select participants (P001-P020)
3. Select experiment
4. Set time slots

```
| Time       | Participant |
|------------|-------------|
| 2:00 PM   | P001        |
| 2:15 PM   | P002        |
| 2:30 PM   | P003        |
| ...        | ...         |
```

## Step 2: Prepare for Trial

Before starting:

1. **Verify Robot Connection**
   - Check robot is powered on
   - Verify network connection
   - Test WebSocket connection

2. **Review Experiment**
   - Ensure experiment is "Ready" status
   - Check step count and timing
   - Verify all actions are configured

3. **Prepare Environment**
   - Ensure participant consent is obtained
   - Set up recording equipment (if needed)
   - Remove distractions

## Step 3: Start a Trial

### From Trials List

1. Find the scheduled trial
2. Click **Start Trial**
3. Confirm participant is ready
4. Click **Begin**

### From Wizard Interface

1. Open **Wizard Interface**
2. Select trial from queue
3. Click **Start**

## Step 4: During the Trial

### Wizard Interface Overview

```
┌──────────────────────────────────────────────────────────────┐
│ Trial: P001 - Interactive Storyteller              [00:05:23]│
├──────────────────────────────────────────────────────────────┤
│ ┌──────────────┐ ┌────────────────────┐ ┌─────────────────┐ │
│ │ Trial        │ │ Timeline           │ │ Robot Control    │ │
│ │ Controls     │ │                    │ │                 │ │
│ │              │ │ ●───●───○───○      │ │ ┌─────────────┐ │ │
│ │ [▶ Play]     │ │ Step 1  2  3  4   │ │ │ Connected ✓ │ │ │
│ │ [⏸ Pause]   │ │             ↑      │ │ │ Battery: 85%│ │ │
│ │ [⏹ Stop]    │ │ Current: Step 2    │ │ └─────────────┘ │ │
│ │              │ │                    │ │                 │ │
│ │ [📝 Notes]   │ │ Progress: 40%     │ │ [Say Text]     │ │
│ │ [⚠ Alert]   │ │                    │ │ [Move Robot]   │ │
│ └──────────────┘ └────────────────────┘ │ [Custom Action]│ │
│                                          └─────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

### Trial Controls

| Button | Action | Keyboard |
|--------|--------|----------|
| Play | Resume trial | Space |
| Pause | Pause trial | Space |
| Stop | End trial early | Escape |
| Notes | Add timestamped note | N |
| Alert | Send alert notification | A |

### Monitoring Progress

**Timeline View:**
- Visual step progression
- Current step highlighted
- Completed steps checked
- Estimated time remaining

**Event Log:**
- Timestamped events
- Action executions
- Wizard interventions
- Robot responses

## Step 5: Wizard Interventions

During Wizard-of-Oz studies, wizards can intervene:

### Add Intervention

1. Click **+ Intervention**
2. Select type:
   - **Pause**: Temporarily stop trial
   - **Resume**: Continue after pause
   - **Note**: Add observation
   - **Alert**: Notify researcher

### Branch Selection

When reaching a conditional step:

1. Observe participant response
2. Select appropriate branch:
   - **Correct**: Proceed to positive path
   - **Incorrect**: Proceed to correction path
3. Select is logged for analysis

### Manual Actions

Execute unplanned actions:

1. Click **+ Action**
2. Select from robot actions
3. Configure parameters
4. Execute immediately

## Step 6: Trial Completion

### Automatic Completion

When all steps complete:
1. Final step executes
2. Trial status → "Completed"
3. Data is saved automatically
4. Summary shown

### Manual Completion

To end early:

1. Click **Stop Trial**
2. Confirm completion
3. Select reason:
   - Participant fatigue
   - Technical issue
   - Protocol complete
4. Save partial data

## Step 7: Post-Trial

### Automatic Prompts

After trial completion:

1. **Participant Debrief**
   - Thank participant
   - Answer questions
   - Collect final feedback

2. **Survey Distribution**
   - Send post-session survey
   - Collect responses

3. **Data Export**
   - Download trial data
   - Export event log

### Trial Summary

View trial summary:

```
┌─────────────────────────────────────────────────────────────┐
│ Trial Summary - P001                                       │
├─────────────────────────────────────────────────────────────┤
│ Duration: 5:23                                            │
│ Steps Completed: 6/6 (100%)                               │
│ Interventions: 2                                          │
│                                                             │
│ Actions:                                                    │
│  ✓ Say Text: "Hello..." (2.3s)                            │
│  ✓ Turn Head: yaw=1.5 (1.1s)                              │
│  ✓ Say Text: "What color..." (3.2s)                       │
│  ⚠ Intervention: Pause (10s)                               │
│  ✓ Branch: Correct selected                                │
│  ✓ Say Text: "Yes! It was red" (2.8s)                     │
│                                                             │
│ Events: 18 logged                                         │
└─────────────────────────────────────────────────────────────┘
```

## Managing Multiple Trials

### Trial Queue

View upcoming trials:

```
┌─────────────────────────────────────────────────────────────┐
│ Trial Queue                                    [Refresh]    │
├─────────────────────────────────────────────────────────────┤
│ 2:00 PM │ P001 │ Interactive Storyteller │ Scheduled       │
│ 2:20 PM │ P002 │ Interactive Storyteller │ Scheduled       │
│ 2:40 PM │ P003 │ Interactive Storyteller │ Scheduled       │
│ 3:00 PM │ P004 │ Interactive Storyteller │ Scheduled       │
└─────────────────────────────────────────────────────────────┘
```

### Trial History

View past trials:

| Participant | Started | Duration | Status | Interventions |
|-------------|---------|----------|--------|---------------|
| P001 | Today 2:00 PM | 5:23 | Completed | 2 |
| P002 | Today 2:20 PM | 4:58 | Completed | 1 |
| P003 | Today 2:45 PM | - | In Progress | 0 |

## Data Collection

### Automatic Data Capture

HRIStudio automatically logs:

- Timestamps for all events
- Action executions
- Robot responses
- Wizard interventions
- Participant responses
- Timing data

### Manual Data

Wizards can add:

- Timestamped notes
- Observation categories
- Participant behavior codes
- Custom annotations

### Export Formats

Download trial data:

| Format | Contents |
|--------|----------|
| CSV | Tabular data for spreadsheets |
| JSON | Full event log with metadata |
| Video | Screen recording (if enabled) |

## Troubleshooting

### Trial Won't Start

1. Check robot connection
2. Verify experiment is "Ready"
3. Check participant consent
4. Review error logs

### Trial Paused Unexpectedly

- Robot may have disconnected
- Check network connection
- Resume when connection restored

### Data Not Saved

- Ensure database connection
- Check disk space
- Export data manually

## Best Practices

### Before Trials

- [ ] Robot connected and tested
- [ ] Experiment verified
- [ ] Participant consent obtained
- [ ] Recording equipment ready
- [ ] Wizard briefed on protocol

### During Trials

- [ ] Monitor timeline progress
- [ ] Take timestamped notes
- [ ] Document interventions
- [ ] Watch for issues

### After Trials

- [ ] Review trial summary
- [ ] Export data promptly
- [ ] Send follow-up surveys
- [ ] Update participant status

## Next Steps

Now that you can run trials:

1. **[Wizard Interface](05-wizard-interface.md)** - Master real-time control
2. **[Data & Analysis](08-data-and-analysis.md)** - Analyze your results
3. **[Forms & Surveys](07-forms-and-surveys.md)** - Collect post-trial data

---

**Previous**: [Designing Experiments](03-designing-experiments.md) | **Next**: [Wizard Interface](05-wizard-interface.md)
