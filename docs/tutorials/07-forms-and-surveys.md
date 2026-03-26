# Tutorial 7: Forms & Surveys

Learn how to create and manage consent forms, surveys, and questionnaires.

## Objectives

- Create consent forms for IRB compliance
- Build post-session surveys
- Collect participant responses
- Manage form templates

## Form Types

HRIStudio supports three form types:

| Type | Purpose | When |
|------|---------|------|
| **Consent** | Informed consent for participation | Before trial |
| **Survey** | Collect feedback and observations | After trial |
| **Questionnaire** | Demographic data collection | Any time |

## Step 1: Access Forms

1. Go to your **Study**
2. Click **Forms** tab
3. View existing forms and templates

### Form List View

```
┌─────────────────────────────────────────────────────────────┐
│ Forms                                           [+ Create]  │
├─────────────────────────────────────────────────────────────┤
│ Name                    Type        Responses    Status     │
│ ─────────────────────────────────────────────────────────── │
│ Informed Consent        Consent     12/20        Active     │
│ Post-Session Survey    Survey      8/20         Active     │
│ Demographics           Questionnaire 15/20      Active     │
│ Template: Standard     Consent     -            Template    │
│ Template: Feedback     Survey      -            Template    │
└─────────────────────────────────────────────────────────────┘
```

## Step 2: Create a Form

### Using a Template

1. Click **Create Form**
2. Select **Use Template**
3. Choose template:
   - Informed Consent
   - Post-Session Survey
   - Demographics
4. Customize as needed

### From Scratch

1. Click **Create Form**
2. Select **Blank Form**
3. Choose form type
4. Build fields manually

## Step 3: Form Builder

The form builder lets you create custom fields:

```
┌─────────────────────────────────────────────────────────────┐
│ Form Builder: Post-Session Survey                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Form Settings                                              │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Title: Post-Session Survey                          │   │
│  │ Type: Survey                                        │   │
│  │ Active: ☑                                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Fields                                                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 1. [Rating] How engaging was the robot?     [✕]   │   │
│  │ 2. [Text] What did you enjoy most?         [✕]   │   │
│  │ 3. [Multiple Choice] Robot personality?     [✕]   │   │
│  │                                                      │   │
│  │ [+ Add Field]                                        │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│  Preview                                                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ How engaging was the robot?                         │   │
│  │ ○ 1  ○ 2  ○ 3  ○ 4  ○ 5                           │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
│                                          [Cancel] [Save]   │
└─────────────────────────────────────────────────────────────┘
```

## Step 4: Field Types

### Text Field

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Text                                           │
├─────────────────────────────────────────────────────────────┤
│ Label: Participant Age                                     │
│ Required: ☑                                               │
│ Placeholder: e.g., 25                                      │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ Participant Age *                                    │   │
│ │ [e.g., 25                                      ]     │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Rating Scale

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Rating                                         │
├─────────────────────────────────────────────────────────────┤
│ Label: How engaging was the robot?                         │
│ Required: ☑                                               │
│ Scale: 1 to [5]                                           │
│ Low Label: Not at all engaging                            │
│ High Label: Very engaging                                  │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ How engaging was the robot? *                       │   │
│ │                                                     │   │
│ │ 1      2      3      4      5                      │   │
│ │ ○      ○      ○      ○      ○                      │   │
│ │ Not at all              Very engaging               │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Multiple Choice

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Multiple Choice                                │
├─────────────────────────────────────────────────────────────┤
│ Label: Did the robot respond appropriately?                 │
│ Required: ☑                                               │
│ Options:                                                  │
│   1. Yes, always                                          │
│   2. Yes, most of the time                                │
│   3. Sometimes                                            │
│   4. Rarely                                               │
│   5. No                                                  │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ Did the robot respond appropriately? *               │   │
│ │                                                     │   │
│ │ ○ Yes, always                                        │   │
│ │ ○ Yes, most of the time                             │   │
│ │ ○ Sometimes                                         │   │
│ │ ○ Rarely                                            │   │
│ │ ○ No                                                │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Yes/No

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Yes/No                                        │
├─────────────────────────────────────────────────────────────┤
│ Label: Would you interact with this robot again?            │
│ Required: ☐                                               │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ Would you interact with this robot again?            │   │
│ │                                                     │   │
│ │ ○ Yes    ○ No                                       │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Text Area

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Text Area                                      │
├─────────────────────────────────────────────────────────────┤
│ Label: What did you enjoy most about the interaction?      │
│ Required: ☐                                               │
│ Rows: [4]                                                 │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ What did you enjoy most about the interaction?       │   │
│ │                                                     │   │
│ │ [                                            ]      │   │
│ │ [                                            ]      │   │
│ │ [                                            ]      │   │
│ │ [                                            ]      │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Date

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Date                                           │
├─────────────────────────────────────────────────────────────┤
│ Label: Session Date                                        │
│ Required: ☑                                               │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ Session Date *                                       │   │
│ │ [📅 Select date                               ]     │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Signature

```
┌─────────────────────────────────────────────────────────────┐
│ Field Type: Signature                                      │
├─────────────────────────────────────────────────────────────┤
│ Label: Participant Signature                                │
│ Required: ☑                                               │
│                                                             │
│ Preview:                                                   │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ Participant Signature *                              │   │
│ │                                                     │   │
│ │ ┌───────────────────────────────────────────────┐   │   │
│ │ │                                               │   │   │
│ │ │            [Sign here]                       │   │   │
│ │ │                                               │   │   │
│ │ └───────────────────────────────────────────────┘   │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Step 5: Consent Forms

### Required Elements

For IRB compliance, consent forms must include:

- [ ] Study title and purpose
- [ ] Principal investigator
- [ ] Procedures description
- [ ] Risks and benefits
- [ ] Confidentiality statement
- [ ] Voluntary participation note
- [ ] Signature and date fields

### Consent Form Template

```json
{
  "title": "Informed Consent",
  "type": "consent",
  "fields": [
    { "type": "text", "label": "Study Title", "required": true },
    { "type": "text", "label": "Principal Investigator", "required": true },
    { "type": "textarea", "label": "Purpose of the Study", "required": true },
    { "type": "textarea", "label": "Procedures", "required": true },
    { "type": "textarea", "label": "Risks and Benefits", "required": true },
    { "type": "textarea", "label": "Confidentiality", "required": true },
    { "type": "yes_no", "label": "I consent to participate", "required": true },
    { "type": "signature", "label": "Participant Signature", "required": true },
    { "type": "date", "label": "Date", "required": true }
  ]
}
```

## Step 6: Surveys

### Post-Session Survey Example

```json
{
  "title": "Post-Session Questionnaire",
  "type": "survey",
  "fields": [
    {
      "type": "rating",
      "label": "How engaging was the robot?",
      "settings": { "scale": 5 }
    },
    {
      "type": "rating",
      "label": "How natural did the interaction feel?",
      "settings": { "scale": 5 }
    },
    {
      "type": "multiple_choice",
      "label": "Did the robot respond appropriately?",
      "options": ["Always", "Usually", "Sometimes", "Rarely", "Never"]
    },
    {
      "type": "textarea",
      "label": "What did you like most?"
    },
    {
      "type": "textarea",
      "label": "What could be improved?"
    }
  ]
}
```

### Questionnaire Example (Demographics)

```json
{
  "title": "Demographics",
  "type": "questionnaire",
  "fields": [
    { "type": "text", "label": "Age" },
    {
      "type": "multiple_choice",
      "label": "Gender",
      "options": ["Male", "Female", "Non-binary", "Prefer not to say"]
    },
    {
      "type": "multiple_choice",
      "label": "Experience with robots",
      "options": ["None", "A little", "Moderate", "Extensive"]
    }
  ]
}
```

## Step 7: Form Versions

Forms support versioning for IRB compliance:

1. Create new version when modifying:
   - Question text changes
   - New fields added
   - Required fields changed

2. Version history:
   ```
   Version 1 (Current) - Active
   Version 2 - Draft
   Version 3 - Archived
   ```

3. Track changes:
   - Version number
   - Change date
   - Change description

## Step 8: Distributing Forms

### Automatic Distribution

Configure automatic form sending:

1. Open form settings
2. Enable **Auto-distribute**
3. Set trigger:
   - Before trial (consent)
   - After trial (survey)
4. Select participants

### Manual Distribution

Send forms manually:

1. Open form
2. Click **Distribute**
3. Select participants
4. Choose delivery method

### Participant Link

Generate shareable link:

```
https://hristudio.example.com/forms/{formId}?participant={participantCode}
```

## Step 9: Collecting Responses

### View Responses

1. Open form
2. Click **Responses** tab
3. View individual submissions

### Response Dashboard

```
┌─────────────────────────────────────────────────────────────┐
│ Form Responses: Post-Session Survey                        │
├─────────────────────────────────────────────────────────────┤
│ Total Responses: 15/20 (75%)                              │
│                                                             │
│ Question: How engaging was the robot?                      │
│ ┌─────────────────────────────────────────────────────┐   │
│ │ 5 ████████████████████████████████████ 8 responses  │   │
│ │ 4 ██████████████████ 5 responses                  │   │
│ │ 3 ████████ 2 responses                            │   │
│ │ 2 ████ 1 response                                 │   │
│ │ 1 ████ 1 response                                 │   │
│ │                                                     │   │
│ │ Average: 4.2 / 5.0                                 │   │
│ └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Export Responses

Download collected data:

| Format | Contents |
|--------|----------|
| CSV | Tabular data |
| JSON | Full response objects |
| PDF | Printed consent forms |

## Step 10: Form Templates

### Creating Templates

1. Create form with desired fields
2. Click **Save as Template**
3. Enter template name
4. Template is available for reuse

### Template Library

| Template | Use Case |
|----------|----------|
| Standard Consent | Generic research consent |
| Child Consent | Studies with minors |
| Extended Consent | Complex procedures |
| Feedback Survey | Post-session feedback |
| NASA-TLX | Workload assessment |
| SUS | System usability |

## Best Practices

### Consent Forms

- [ ] Review with IRB before use
- [ ] Keep language simple
- [ ] Include all required elements
- [ ] Version control for changes
- [ ] Store signed forms securely

### Surveys

- [ ] Keep questions concise
- [ ] Use appropriate scales
- [ ] Test with pilot participants
- [ ] Randomize order when appropriate
- [ ] Include open-ended questions

### Data Management

- [ ] Export data regularly
- [ ] Backup responses
- [ ] Anonymize data for analysis
- [ ] Follow data retention policy

## Troubleshooting

### Form Not Loading

- Check form is active
- Verify participant access
- Check network connection

### Response Not Saving

- Check required fields
- Verify session active
- Try again or refresh

### Participant Can't Access

- Verify participant code valid
- Check form is distributed
- Confirm study is active

## Next Steps

Now that you've created your forms:

1. **[Running Trials](04-running-trials.md)** - Connect forms to trials
2. **[Data & Analysis](08-data-and-analysis.md)** - Analyze collected data
3. **[Your First Study](02-your-first-study.md)** - Set up your study

---

**Previous**: [Robot Integration](06-robot-integration.md) | **Next**: [Data & Analysis](08-data-and-analysis.md)
