# HRIStudio API Routes Documentation

## Overview

HRIStudio uses tRPC for type-safe API communication between client and server. All routes are organized into routers by feature area and composed into a single app router. Authentication is handled by NextAuth.js v5.

## Base Configuration

```typescript
// All routes are prefixed with /api/trpc/
// Example: POST /api/trpc/auth.login
```

## Authentication Routes (`auth`)

### `auth.login`
- **Description**: Authenticate user with email/password or OAuth
- **Type**: Mutation
- **Input**: 
  ```typescript
  {
    email?: string
    password?: string
    provider?: 'google' | 'github' | 'microsoft'
    callbackUrl?: string
  }
  ```
- **Output**: Session object with user details
- **Auth Required**: No
- **Notes**: Handled primarily by NextAuth.js

### `auth.logout`
- **Description**: End user session
- **Type**: Mutation
- **Input**: None
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes

### `auth.register`
- **Description**: Create new user account
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    email: string
    password: string
    name: string
    institution?: string
  }
  ```
- **Output**: User object
- **Auth Required**: No

### `auth.me`
- **Description**: Get current user profile
- **Type**: Query
- **Input**: None
- **Output**: User object with roles
- **Auth Required**: Yes

## User Management Routes (`users`)

### `users.list`
- **Description**: List all users (admin only)
- **Type**: Query
- **Input**:
  ```typescript
  {
    page?: number
    limit?: number
    search?: string
    role?: SystemRole
  }
  ```
- **Output**: Paginated user list
- **Auth Required**: Yes (Administrator)

### `users.get`
- **Description**: Get user by ID
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: User object
- **Auth Required**: Yes

### `users.update`
- **Description**: Update user profile
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    id: string
    name?: string
    email?: string
    image?: string
  }
  ```
- **Output**: Updated user object
- **Auth Required**: Yes (Self or Administrator)

### `users.assignRole`
- **Description**: Assign system role to user
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    userId: string
    role: SystemRole
  }
  ```
- **Output**: Updated user roles
- **Auth Required**: Yes (Administrator)

### `users.delete`
- **Description**: Soft delete user account
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Administrator)

## Study Management Routes (`studies`)

### `studies.list`
- **Description**: List studies accessible to user
- **Type**: Query
- **Input**:
  ```typescript
  {
    page?: number
    limit?: number
    status?: StudyStatus
    search?: string
    myStudiesOnly?: boolean
  }
  ```
- **Output**: Paginated study list with member counts
- **Auth Required**: Yes

### `studies.get`
- **Description**: Get study details
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Complete study object with metadata
- **Auth Required**: Yes (Study member)

### `studies.create`
- **Description**: Create new study
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    name: string
    description?: string
    institution?: string
    irbProtocol?: string
    metadata?: Record<string, any>
  }
  ```
- **Output**: Created study object
- **Auth Required**: Yes (Researcher)

### `studies.update`
- **Description**: Update study details
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    id: string
    name?: string
    description?: string
    status?: StudyStatus
    metadata?: Record<string, any>
    settings?: Record<string, any>
  }
  ```
- **Output**: Updated study object
- **Auth Required**: Yes (Study owner/researcher)

### `studies.delete`
- **Description**: Archive study (soft delete)
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study owner)

### `studies.addMember`
- **Description**: Add member to study team
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    userId: string
    role: StudyRole
    permissions?: string[]
  }
  ```
- **Output**: Study member object
- **Auth Required**: Yes (Study owner/researcher)

### `studies.removeMember`
- **Description**: Remove member from study
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    userId: string
  }
  ```
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study owner)

### `studies.getMembers`
- **Description**: List study team members
- **Type**: Query
- **Input**: `{ studyId: string }`
- **Output**: Array of member objects with roles
- **Auth Required**: Yes (Study member)

### `studies.getActivity`
- **Description**: Get study activity log
- **Type**: Query
- **Input**:
  ```typescript
  {
    studyId: string
    page?: number
    limit?: number
    startDate?: Date
    endDate?: Date
  }
  ```
- **Output**: Paginated activity log
- **Auth Required**: Yes (Study member)

## Experiment Design Routes (`experiments`)

### `experiments.list`
- **Description**: List experiments in a study
- **Type**: Query
- **Input**:
  ```typescript
  {
    studyId: string
    status?: ExperimentStatus
  }
  ```
- **Output**: Array of experiment objects
- **Auth Required**: Yes (Study member)

### `experiments.get`
- **Description**: Get experiment details with steps
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Experiment object with nested steps and actions
- **Auth Required**: Yes (Study member)

### `experiments.create`
- **Description**: Create new experiment
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    name: string
    description?: string
    robotId?: string
    estimatedDuration?: number
    metadata?: Record<string, any>
  }
  ```
- **Output**: Created experiment object
- **Auth Required**: Yes (Study researcher)

### `experiments.update`
- **Description**: Update experiment details
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    id: string
    name?: string
    description?: string
    status?: ExperimentStatus
    estimatedDuration?: number
    metadata?: Record<string, any>
  }
  ```
- **Output**: Updated experiment object
- **Auth Required**: Yes (Study researcher)

### `experiments.duplicate`
- **Description**: Create copy of experiment
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    experimentId: string
    newName: string
    includeSteps?: boolean
  }
  ```
- **Output**: New experiment object
- **Auth Required**: Yes (Study researcher)

### `experiments.delete`
- **Description**: Delete experiment
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `experiments.addStep`
- **Description**: Add step to experiment
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    experimentId: string
    name: string
    description?: string
    type: StepType
    orderIndex: number
    durationEstimate?: number
    required?: boolean
    conditions?: Record<string, any>
  }
  ```
- **Output**: Created step object
- **Auth Required**: Yes (Study researcher)

### `experiments.updateStep`
- **Description**: Update step details
- **Type**: Mutation
- **Input**: Step object with id
- **Output**: Updated step object
- **Auth Required**: Yes (Study researcher)

### `experiments.deleteStep`
- **Description**: Remove step from experiment
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `experiments.reorderSteps`
- **Description**: Change step order
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    experimentId: string
    stepIds: string[] // in new order
  }
  ```
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `experiments.addAction`
- **Description**: Add action to step
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    stepId: string
    name: string
    description?: string
    type: string
    orderIndex: number
    parameters?: Record<string, any>
    validationSchema?: Record<string, any>
    timeout?: number
  }
  ```
- **Output**: Created action object
- **Auth Required**: Yes (Study researcher)

### `experiments.updateAction`
- **Description**: Update action details
- **Type**: Mutation
- **Input**: Action object with id
- **Output**: Updated action object
- **Auth Required**: Yes (Study researcher)

### `experiments.deleteAction`
- **Description**: Remove action from step
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `experiments.validate`
- **Description**: Validate experiment configuration
- **Type**: Query
- **Input**: `{ experimentId: string }`
- **Output**:
  ```typescript
  {
    valid: boolean
    errors?: ValidationError[]
    warnings?: ValidationWarning[]
  }
  ```
- **Auth Required**: Yes (Study member)

## Trial Execution Routes (`trials`)

### `trials.list`
- **Description**: List trials for experiment
- **Type**: Query
- **Input**:
  ```typescript
  {
    experimentId?: string
    studyId?: string
    status?: TrialStatus
    participantId?: string
    wizardId?: string
    page?: number
    limit?: number
  }
  ```
- **Output**: Paginated trial list
- **Auth Required**: Yes (Study member)

### `trials.get`
- **Description**: Get trial details
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Trial object with events
- **Auth Required**: Yes (Study member)

### `trials.create`
- **Description**: Schedule new trial
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    experimentId: string
    participantId?: string
    wizardId?: string
    scheduledAt?: Date
    parameters?: Record<string, any>
  }
  ```
- **Output**: Created trial object
- **Auth Required**: Yes (Study researcher)

### `trials.update`
- **Description**: Update trial details
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    id: string
    participantId?: string
    wizardId?: string
    scheduledAt?: Date
    status?: TrialStatus
    notes?: string
  }
  ```
- **Output**: Updated trial object
- **Auth Required**: Yes (Study researcher/wizard)

### `trials.start`
- **Description**: Begin trial execution
- **Type**: Mutation
- **Input**: `{ trialId: string }`
- **Output**:
  ```typescript
  {
    success: boolean
    sessionToken: string
    websocketUrl: string
  }
  ```
- **Auth Required**: Yes (Assigned wizard)

### `trials.complete`
- **Description**: Mark trial as completed
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    notes?: string
  }
  ```
- **Output**: Updated trial object
- **Auth Required**: Yes (Assigned wizard)

### `trials.abort`
- **Description**: Abort trial execution
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    reason: string
  }
  ```
- **Output**: Updated trial object
- **Auth Required**: Yes (Assigned wizard)

### `trials.logEvent`
- **Description**: Log trial event
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    eventType: string
    actionId?: string
    data?: Record<string, any>
  }
  ```
- **Output**: Event object
- **Auth Required**: Yes (Assigned wizard)

### `trials.addIntervention`
- **Description**: Record wizard intervention
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    interventionType: string
    description?: string
    parameters?: Record<string, any>
    reason?: string
  }
  ```
- **Output**: Intervention object
- **Auth Required**: Yes (Assigned wizard)

### `trials.getEvents`
- **Description**: Get trial event timeline
- **Type**: Query
- **Input**:
  ```typescript
  {
    trialId: string
    eventType?: string
    startTime?: Date
    endTime?: Date
  }
  ```
- **Output**: Array of event objects
- **Auth Required**: Yes (Study member)

## Participant Management Routes (`participants`)

### `participants.list`
- **Description**: List study participants
- **Type**: Query
- **Input**:
  ```typescript
  {
    studyId: string
    page?: number
    limit?: number
    search?: string
  }
  ```
- **Output**: Paginated participant list
- **Auth Required**: Yes (Study member)

### `participants.get`
- **Description**: Get participant details
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Participant object (sensitive data based on permissions)
- **Auth Required**: Yes (Study member)

### `participants.create`
- **Description**: Register new participant
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    participantCode: string
    email?: string
    name?: string
    demographics?: Record<string, any>
  }
  ```
- **Output**: Created participant object
- **Auth Required**: Yes (Study researcher)

### `participants.update`
- **Description**: Update participant information
- **Type**: Mutation
- **Input**: Participant object with id
- **Output**: Updated participant object
- **Auth Required**: Yes (Study researcher)

### `participants.delete`
- **Description**: Remove participant data
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `participants.recordConsent`
- **Description**: Record participant consent
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    participantId: string
    consentFormId: string
    signatureData?: string
    ipAddress?: string
  }
  ```
- **Output**: Consent record object
- **Auth Required**: Yes (Study researcher/wizard)

## Robot and Plugin Routes (`robots`)

### `robots.list`
- **Description**: List available robots
- **Type**: Query
- **Input**: None
- **Output**: Array of robot objects
- **Auth Required**: Yes

### `robots.get`
- **Description**: Get robot details
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Robot object with capabilities
- **Auth Required**: Yes

### `robots.testConnection`
- **Description**: Test robot connectivity
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    robotId: string
    configuration: Record<string, any>
  }
  ```
- **Output**:
  ```typescript
  {
    connected: boolean
    latency?: number
    error?: string
  }
  ```
- **Auth Required**: Yes (Study researcher)

### `plugins.list`
- **Description**: List available plugins
- **Type**: Query
- **Input**:
  ```typescript
  {
    robotId?: string
    trustLevel?: PluginTrustLevel
  }
  ```
- **Output**: Array of plugin objects
- **Auth Required**: Yes

### `plugins.get`
- **Description**: Get plugin details
- **Type**: Query
- **Input**: `{ id: string }`
- **Output**: Plugin object with action definitions
- **Auth Required**: Yes

### `plugins.install`
- **Description**: Install plugin for study
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    pluginId: string
    configuration?: Record<string, any>
  }
  ```
- **Output**: Installation record
- **Auth Required**: Yes (Study researcher)

### `plugins.uninstall`
- **Description**: Remove plugin from study
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    pluginId: string
  }
  ```
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `plugins.getActions`
- **Description**: Get available actions for robot
- **Type**: Query
- **Input**:
  ```typescript
  {
    studyId: string
    robotId: string
  }
  ```
- **Output**: Array of action definitions
- **Auth Required**: Yes (Study member)

## Media and Data Routes (`media`)

### `media.uploadVideo`
- **Description**: Upload video file
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    file: File
    startTimestamp: Date
    endTimestamp?: Date
  }
  ```
- **Output**: Media capture object
- **Auth Required**: Yes (Study member)

### `media.uploadAudio`
- **Description**: Upload audio file
- **Type**: Mutation
- **Input**: Similar to uploadVideo
- **Output**: Media capture object
- **Auth Required**: Yes (Study member)

### `media.list`
- **Description**: List media files for trial
- **Type**: Query
- **Input**:
  ```typescript
  {
    trialId: string
    mediaType?: MediaType
  }
  ```
- **Output**: Array of media objects
- **Auth Required**: Yes (Study member)

### `media.getUrl`
- **Description**: Get presigned URL for media
- **Type**: Query
- **Input**: `{ mediaId: string }`
- **Output**:
  ```typescript
  {
    url: string
    expiresAt: Date
  }
  ```
- **Auth Required**: Yes (Study member)

### `media.delete`
- **Description**: Delete media file
- **Type**: Mutation
- **Input**: `{ mediaId: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Study researcher)

### `sensorData.record`
- **Description**: Record sensor data batch
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    sensorType: string
    data: Array<{
      timestamp: Date
      values: Record<string, any>
      robotState?: Record<string, any>
    }>
  }
  ```
- **Output**: `{ recordedCount: number }`
- **Auth Required**: Yes (System/Robot)

### `sensorData.query`
- **Description**: Query sensor data
- **Type**: Query
- **Input**:
  ```typescript
  {
    trialId: string
    sensorType?: string
    startTime: Date
    endTime: Date
    limit?: number
  }
  ```
- **Output**: Array of sensor readings
- **Auth Required**: Yes (Study member)

## Analysis Routes (`analysis`)

### `analysis.createAnnotation`
- **Description**: Add annotation to trial
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    trialId: string
    timestampStart: Date
    timestampEnd?: Date
    category?: string
    description: string
    tags?: string[]
  }
  ```
- **Output**: Annotation object
- **Auth Required**: Yes (Study member)

### `analysis.updateAnnotation`
- **Description**: Update existing annotation
- **Type**: Mutation
- **Input**: Annotation object with id
- **Output**: Updated annotation
- **Auth Required**: Yes (Annotation creator)

### `analysis.deleteAnnotation`
- **Description**: Remove annotation
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Annotation creator)

### `analysis.getAnnotations`
- **Description**: Get annotations for trial
- **Type**: Query
- **Input**:
  ```typescript
  {
    trialId: string
    category?: string
    annotatorId?: string
  }
  ```
- **Output**: Array of annotations
- **Auth Required**: Yes (Study member)

### `analysis.exportData`
- **Description**: Export study data
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    exportType: ExportType
    format: ExportFormat
    filters?: {
      experiments?: string[]
      trials?: string[]
      dateRange?: {
        start: Date
        end: Date
      }
      includeMedia?: boolean
    }
  }
  ```
- **Output**: Export job object
- **Auth Required**: Yes (Study researcher)

### `analysis.getExportStatus`
- **Description**: Check export job status
- **Type**: Query
- **Input**: `{ jobId: string }`
- **Output**:
  ```typescript
  {
    status: JobStatus
    progress?: number
    downloadUrl?: string
    error?: string
  }
  ```
- **Auth Required**: Yes (Job creator)

## Collaboration Routes (`collaboration`)

### `collaboration.createComment`
- **Description**: Add comment to resource
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    resourceType: ResourceType
    resourceId: string
    content: string
    parentId?: string
  }
  ```
- **Output**: Comment object
- **Auth Required**: Yes (Study member)

### `collaboration.getComments`
- **Description**: Get comments for resource
- **Type**: Query
- **Input**:
  ```typescript
  {
    resourceType: ResourceType
    resourceId: string
  }
  ```
- **Output**: Nested comment tree
- **Auth Required**: Yes (Study member)

### `collaboration.deleteComment`
- **Description**: Delete comment (soft)
- **Type**: Mutation
- **Input**: `{ id: string }`
- **Output**: `{ success: boolean }`
- **Auth Required**: Yes (Comment author)

### `collaboration.uploadAttachment`
- **Description**: Upload file attachment
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    file: File
    description?: string
    resourceType?: ResourceType
    resourceId?: string
  }
  ```
- **Output**: Attachment object
- **Auth Required**: Yes (Study member)

### `collaboration.shareResource`
- **Description**: Create shareable link
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    studyId: string
    resourceType: ResourceType
    resourceId: string
    permissions?: string[]
    expiresAt?: Date
  }
  ```
- **Output**:
  ```typescript
  {
    shareToken: string
    shareUrl: string
  }
  ```
- **Auth Required**: Yes (Study researcher)

## System Administration Routes (`admin`)

### `admin.getSystemStats`
- **Description**: Get system statistics
- **Type**: Query
- **Input**: None
- **Output**:
  ```typescript
  {
    userCount: number
    studyCount: number
    trialCount: number
    storageUsed: number
    activeUsers: number
  }
  ```
- **Auth Required**: Yes (Administrator)

### `admin.getSystemSettings`
- **Description**: Get system configuration
- **Type**: Query
- **Input**: None
- **Output**: Settings object
- **Auth Required**: Yes (Administrator)

### `admin.updateSystemSettings`
- **Description**: Update system configuration
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    key: string
    value: any
  }
  ```
- **Output**: Updated settings
- **Auth Required**: Yes (Administrator)

### `admin.getAuditLog`
- **Description**: Query audit log
- **Type**: Query
- **Input**:
  ```typescript
  {
    userId?: string
    action?: string
    resourceType?: string
    startDate?: Date
    endDate?: Date
    page?: number
    limit?: number
  }
  ```
- **Output**: Paginated audit entries
- **Auth Required**: Yes (Administrator)

### `admin.createBackup`
- **Description**: Initiate system backup
- **Type**: Mutation
- **Input**:
  ```typescript
  {
    includeMedia?: boolean
    encryptionKey?: string
  }
  ```
- **Output**: Backup job object
- **Auth Required**: Yes (Administrator)

## WebSocket Events

### Trial Execution Events
- `trial.started`: Trial execution began
- `trial.action.started`: Action execution started
- `trial.action.completed`: Action execution completed
- `trial.intervention`: Wizard intervention occurred
- `trial.error`: Error during execution
- `trial.completed`: Trial finished
- `trial.aborted`: Trial was aborted

### Collaboration Events
- `study.member.joined`: New member added
- `study.member.left`: Member removed
- `comment.created`: New comment added
- `annotation.created`: New annotation added

### System Events
- `plugin.status`: Plugin connection status change
- `export.progress`: Export job progress update
- `system.maintenance`: System maintenance notification

## Error Handling

All routes return consistent error responses:

```typescript
{
  error: {
    code: string // e.g., 'UNAUTHORIZED', 'NOT_FOUND', 'VALIDATION_ERROR'
    message: string
    details?: any
  }
}
```

Common error codes:
- `UNAUTHORIZED`: User not authenticated
- `FORBIDDEN`: User lacks permission
- `NOT_FOUND`: Resource doesn't exist
- `VALIDATION_ERROR`: Input validation failed
- `CONFLICT`: Resource conflict (e.g., duplicate)
- `INTERNAL_ERROR`: Server error

## Rate Limiting

- Authentication endpoints: 5 requests per minute
- File uploads: 10 per hour per user
- Data exports: 5 per hour per study
- General API calls: 100 per minute per user