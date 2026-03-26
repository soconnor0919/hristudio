# Tutorial 8: Data & Analysis

Learn how to collect, export, and analyze trial data from HRIStudio.

## Objectives

- Understand data collection in HRIStudio
- Export trial data in various formats
- Analyze event logs
- Generate reports

## Data Collection Overview

HRIStudio automatically captures comprehensive data during trials:

```
┌─────────────────────────────────────────────────────────────┐
│                    Data Collection                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Trial Metadata                                             │
│  ├── Start/End times                                        │
│  ├── Duration                                               │
│  ├── Participant info                                       │
│  └── Experiment version                                     │
│                                                             │
│  Event Log (Timestamped)                                    │
│  ├── Step changes                                           │
│  ├── Action executions                                      │
│  ├── Robot responses                                        │
│  └── Wizard interventions                                   │
│                                                             │
│  Form Responses                                             │
│  ├── Consent forms                                          │
│  ├── Surveys                                                │
│  └── Questionnaires                                        │
│                                                             │
│  Sensor Data                                                │
│  ├── Joint positions                                        │
│  ├── Touch events                                           │
│  └── Audio/video (if enabled)                               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Step 1: Accessing Trial Data

### From Trial List

1. Go to **Trials** tab
2. Find completed trial
3. Click **View Details**

### From Study Dashboard

1. Open your study
2. Go to **Data** tab
3. Select trial or view aggregate

## Step 2: Trial Event Log

Each trial generates a complete event log:

```json
{
  "trialId": "trial_abc123",
  "participantCode": "P001",
  "experimentName": "Interactive Storyteller",
  "startedAt": "2024-03-15T14:00:00Z",
  "completedAt": "2024-03-15T14:05:23Z",
  "duration": 323,
  "status": "completed",
  "events": [
    {
      "timestamp": "2024-03-15T14:00:00.123Z",
      "type": "trial_started",
      "stepId": null,
      "data": {}
    },
    {
      "timestamp": "2024-03-15T14:00:02.456Z",
      "type": "step_changed",
      "stepId": "step_1",
      "stepName": "The Hook",
      "data": {}
    },
    {
      "timestamp": "2024-03-15T14:00:03.789Z",
      "type": "action_executed",
      "actionName": "Say Text",
      "parameters": { "text": "Hello!" },
      "duration": 2300,
      "status": "completed"
    },
    {
      "timestamp": "2024-03-15T14:00:08.012Z",
      "type": "action_executed",
      "actionName": "Wave",
      "duration": 1500,
      "status": "completed"
    },
    {
      "timestamp": "2024-03-15T14:02:30.123Z",
      "type": "intervention",
      "interventionType": "note",
      "data": { "note": "Participant laughed" }
    },
    {
      "timestamp": "2024-03-15T14:03:00.456Z",
      "type": "wizard_response",
      "variable": "last_response",
      "selectedValue": "correct",
      "data": {}
    },
    {
      "timestamp": "2024-03-15T14:05:23.789Z",
      "type": "trial_completed",
      "data": { "stepsCompleted": 6 }
    }
  ]
}
```

### Event Types

| Event Type | Description | Data Captured |
|------------|-------------|---------------|
| `trial_started` | Trial began | Timestamp |
| `step_changed` | New step began | Step ID, name |
| `action_executed` | Robot action | Action details, duration |
| `action_completed` | Action finished | Duration, result |
| `action_failed` | Action failed | Error details |
| `wizard_response` | Wizard decision | Selected option |
| `intervention` | Wizard intervention | Type, note |
| `trial_paused` | Trial paused | Reason |
| `trial_resumed` | Trial resumed | Pause duration |
| `trial_completed` | Trial finished | Summary |

## Step 3: Exporting Data

### Export Single Trial

1. Open trial details
2. Click **Export**
3. Select format

### Export Study Data

1. Open study
2. Go to **Data** tab
3. Click **Export All**
4. Select options:
   - Date range
   - Trial status
   - Include forms

### Export Formats

#### CSV Format

```csv
trial_id,participant,experiment,started_at,duration,status,steps_completed
trial_abc,P001,Interactive Storyteller,2024-03-15T14:00:00Z,323,completed,6
trial_def,P002,Interactive Storyteller,2024-03-15T14:20:00Z,298,completed,6
trial_ghi,P003,Interactive Storyteller,2024-03-15T14:40:00Z,0,failed,1
```

#### JSON Format

```json
{
  "exportDate": "2024-03-15T15:00:00Z",
  "studyName": "Robot Trust Study",
  "trials": [...],
  "forms": [...],
  "metadata": {
    "totalTrials": 20,
    "completedTrials": 18,
    "averageDuration": 312
  }
}
```

#### Event Log CSV

```csv
timestamp,event_type,step_name,action_name,parameters,duration,status
2024-03-15T14:00:00.123Z,trial_started,,,,,
2024-03-15T14:00:02.456Z,step_changed,The Hook,,,,
2024-03-15T14:00:03.789Z,action_executed,The Hook,Say Text,"{""text"":""Hello!""}",2300,completed
2024-03-15T14:00:08.012Z,action_executed,The Hook,Wave,,1500,completed
2024-03-15T14:02:30.123Z,intervention,The Narrative,Note,"{""note"":""Participant laughed""}",,,
2024-03-15T14:03:00.456Z,wizard_response,Comprehension Check,Correct,,,,
2024-03-15T14:05:23.789Z,trial_completed,,,,323,
```

## Step 4: Data Dashboard

### Study Dashboard

View aggregate statistics:

```
┌─────────────────────────────────────────────────────────────┐
│ Study Dashboard: Robot Trust Study                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Overview                                                   │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐         │
│  │   20    │ │   18    │ │  5m12s  │ │   2     │         │
│  │ Trials  │ │ Complete│ │ Avg Time│ │ Failed  │         │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘         │
│                                                             │
│  Completion Rate                                           │
│  ████████████████████████████████████░░░░ 90%              │
│                                                             │
│  Timeline                                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ P001 ████████████████████████████████ 5:23         │   │
│  │ P002 ██████████████████████████████ 5:02            │   │
│  │ P003 ██████████████████████████ 4:45                │   │
│  │ ...                                                  │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Metrics

| Metric | Description |
|--------|-------------|
| Total Trials | Number of scheduled trials |
| Completed | Successfully completed trials |
| Average Duration | Mean trial time |
| Completion Rate | % of trials completed |
| Failed | Trials that failed |
| Average Steps | Mean steps per trial |

## Step 5: Analyzing Event Data

### Timing Analysis

Calculate action durations:

```python
import json

with open('trial_events.json') as f:
    data = json.load(f)

# Calculate action durations
for event in data['events']:
    if event['type'] == 'action_executed':
        duration = event.get('duration', 0)
        print(f"{event['actionName']}: {duration/1000:.1f}s")
```

### Intervention Analysis

Track wizard interventions:

```python
# Count interventions by type
interventions = [
    e for e in data['events'] 
    if e['type'] == 'intervention'
]

by_type = {}
for i in interventions:
    itype = i['data'].get('type', 'unknown')
    by_type[itype] = by_type.get(itype, 0) + 1

print(by_type)
# {'note': 15, 'pause': 3, 'alert': 1}
```

### Branch Selection Analysis

Analyze wizard decisions:

```python
# Get wizard responses
responses = [
    e for e in data['events']
    if e['type'] == 'wizard_response'
]

# Count by value
by_value = {}
for r in responses:
    value = r.get('selectedValue', 'unknown')
    by_value[value] = by_value.get(value, 0) + 1

print(by_value)
# {'correct': 12, 'incorrect': 6}
```

## Step 6: Form Data Analysis

### Response Aggregation

Aggregate survey responses:

```python
# Calculate average rating
ratings = [
    r['responses']['engagement_rating']
    for r in form_responses
]

avg_rating = sum(ratings) / len(ratings)
print(f"Average engagement: {avg_rating:.2f}/5")
```

### Cross-Tabulation

Compare responses across conditions:

```
                    | Condition A | Condition B | Total
--------------------|------------|-------------|-------
Robot engaged       |     4.2    |     4.5     |  4.35
Natural interaction |     3.8    |     4.1     |  3.95
Would use again     |     78%    |     85%     |  81%
```

## Step 7: Data Visualization

### Trial Timeline

Visualize trial progression:

```
P001: ████████████████░░░░░░░░░░░░░░░░░ 5:23
P002: ███████████████░░░░░░░░░░░░░░░░░░ 4:58
P003: ██████████████████████████████░░░░ 6:02
P004: ████████████████░░░░░░░░░░░░░░░░░░ 5:15
```

### Action Distribution

```
Action Frequency
────────────────
Say Text       ████████████████████ 45
Wave           ████████████ 25
Turn Head      ████████████ 25
Move Arm       ████ 5
```

### Branch Outcomes

```
Branch Selection
────────────────
Correct Response (A): ██████████████████████████ 67%
Incorrect Response (B): █████████████ 33%
```

## Step 8: Generating Reports

### Trial Summary Report

Generate PDF summary:

```
═══════════════════════════════════════════════════════════
                    TRIAL SUMMARY REPORT
═══════════════════════════════════════════════════════════

Study: Robot Trust Study
Participant: P001
Date: March 15, 2024
Experiment: Interactive Storyteller v1

EXECUTIVE SUMMARY
───────────────────────────────────────────────────────────
Duration: 5 minutes 23 seconds
Status: Completed successfully
Steps Completed: 6/6
Interventions: 2

TIMELINE
───────────────────────────────────────────────────────────
14:00:00 Trial started
14:00:02 Step 1: The Hook
14:00:08 Step 2: The Narrative  
14:02:30 Wizard note: "Participant engaged"
14:03:00 Step 3: Comprehension Check
14:03:28 Branch selected: Correct
14:03:30 Step 4a: Correct Response
14:05:23 Trial completed

METRICS
───────────────────────────────────────────────────────────
Actions Executed: 12
Action Success Rate: 100%
Average Action Duration: 2.1s
Wizard Intervention Rate: 0.37/min

═══════════════════════════════════════════════════════════
```

### Study Report

Aggregate across participants:

```
═══════════════════════════════════════════════════════════
                    STUDY REPORT
═══════════════════════════════════════════════════════════

Study: Robot Trust Study
Date Range: March 1-15, 2024
Participants: 20

PARTICIPATION
───────────────────────────────────────────────────────────
Enrolled: 20
Completed: 18 (90%)
Withdrew: 1 (5%)
Failed: 1 (5%)

TIMING
───────────────────────────────────────────────────────────
Mean Duration: 5m 12s ± 28s
Min Duration: 4m 45s
Max Duration: 6m 02s

INTERVENTIONS
───────────────────────────────────────────────────────────
Total Interventions: 34
Notes: 25 (73%)
Pauses: 7 (21%)
Alerts: 2 (6%)

BRANCH SELECTION
───────────────────────────────────────────────────────────
Branch A (Correct): 12 (67%)
Branch B (Incorrect): 6 (33%)

═══════════════════════════════════════════════════════════
```

## Step 9: Data Privacy

### Anonymization

Remove identifying information:

```python
# Replace participant codes with anonymous IDs
participant_map = {
    'P001': 'S001',
    'P002': 'S002',
    'P003': 'S003',
}
```

### Export Settings

Configure export options:

| Option | Description |
|--------|-------------|
| Include participant codes | Keep or anonymize |
| Include timestamps | Full or relative |
| Include notes | Include/exclude |
| Include form responses | Include/exclude |

## Best Practices

### Data Collection

- [ ] Enable all event logging
- [ ] Configure sensor data capture
- [ ] Set up automatic backups
- [ ] Test data export before study

### Data Storage

- [ ] Export regularly (daily/weekly)
- [ ] Store in secure location
- [ ] Follow IRB data retention
- [ ] Backup critical data

### Data Analysis

- [ ] Document analysis methods
- [ ] Track protocol versions
- [ ] Note data quality issues
- [ ] Share data dictionary

## Next Steps

Now that you understand data collection:

1. **[Your First Study](02-your-first-study.md)** - Apply data practices
2. **[Simulation Mode](09-simulation-mode.md)** - Test data collection
3. **[Running Trials](04-running-trials.md)** - Practice with data capture

---

**Previous**: [Forms & Surveys](07-forms-and-surveys.md) | **Next**: [Simulation Mode](09-simulation-mode.md)
