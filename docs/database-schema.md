# HRIStudio Database Schema

## Overview

This document provides a comprehensive database schema for HRIStudio using PostgreSQL with Drizzle ORM. The schema follows the hierarchical structure of WoZ studies and implements role-based access control, comprehensive data capture, and collaboration features.

## Core Entities

### Users and Authentication

```sql
-- Users table for authentication and profile information
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  email_verified TIMESTAMP,
  name VARCHAR(255),
  image TEXT,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  deleted_at TIMESTAMP,
  CONSTRAINT email_format CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$')
);

-- NextAuth accounts table
CREATE TABLE accounts (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  type VARCHAR(255) NOT NULL,
  provider VARCHAR(255) NOT NULL,
  provider_account_id VARCHAR(255) NOT NULL,
  refresh_token TEXT,
  access_token TEXT,
  expires_at INTEGER,
  token_type VARCHAR(255),
  scope VARCHAR(255),
  id_token TEXT,
  session_state VARCHAR(255),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(provider, provider_account_id)
);

-- NextAuth sessions table
CREATE TABLE sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_token VARCHAR(255) UNIQUE NOT NULL,
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  expires TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- NextAuth verification tokens
CREATE TABLE verification_tokens (
  identifier VARCHAR(255) NOT NULL,
  token VARCHAR(255) UNIQUE NOT NULL,
  expires TIMESTAMP NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (identifier, token)
);
```

### Roles and Permissions

```sql
-- System roles
CREATE TYPE system_role AS ENUM ('administrator', 'researcher', 'wizard', 'observer');

-- User system roles
CREATE TABLE user_system_roles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  role system_role NOT NULL,
  granted_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  granted_by UUID REFERENCES users(id),
  UNIQUE(user_id, role)
);

-- Custom permissions for fine-grained access control
CREATE TABLE permissions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name VARCHAR(100) UNIQUE NOT NULL,
  description TEXT,
  resource VARCHAR(50) NOT NULL,
  action VARCHAR(50) NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Role permissions mapping
CREATE TABLE role_permissions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  role system_role NOT NULL,
  permission_id UUID NOT NULL REFERENCES permissions(id) ON DELETE CASCADE,
  UNIQUE(role, permission_id)
);
```

### Study Hierarchy

```sql
-- Studies: Top-level research projects
CREATE TABLE studies (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name VARCHAR(255) NOT NULL,
  description TEXT,
  institution VARCHAR(255),
  irb_protocol VARCHAR(100),
  status VARCHAR(50) DEFAULT 'draft' CHECK (status IN ('draft', 'active', 'completed', 'archived')),
  created_by UUID NOT NULL REFERENCES users(id),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}',
  settings JSONB DEFAULT '{}',
  deleted_at TIMESTAMP
);

-- Study team members with roles
CREATE TABLE study_members (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  role VARCHAR(50) NOT NULL CHECK (role IN ('owner', 'researcher', 'wizard', 'observer')),
  permissions JSONB DEFAULT '[]',
  joined_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  invited_by UUID REFERENCES users(id),
  UNIQUE(study_id, user_id)
);

-- Experiments: Protocol templates within studies
CREATE TABLE experiments (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  name VARCHAR(255) NOT NULL,
  description TEXT,
  version INTEGER DEFAULT 1,
  robot_id UUID REFERENCES robots(id),
  status VARCHAR(50) DEFAULT 'draft' CHECK (status IN ('draft', 'testing', 'ready', 'deprecated')),
  estimated_duration INTEGER, -- in minutes
  created_by UUID NOT NULL REFERENCES users(id),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}',
  deleted_at TIMESTAMP,
  UNIQUE(study_id, name, version)
);

-- Trials: Executable instances of experiments
CREATE TABLE trials (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  experiment_id UUID NOT NULL REFERENCES experiments(id),
  participant_id UUID REFERENCES participants(id),
  wizard_id UUID REFERENCES users(id),
  session_number INTEGER NOT NULL DEFAULT 1,
  status VARCHAR(50) DEFAULT 'scheduled' CHECK (status IN ('scheduled', 'in_progress', 'completed', 'aborted', 'failed')),
  scheduled_at TIMESTAMP,
  started_at TIMESTAMP,
  completed_at TIMESTAMP,
  duration INTEGER, -- actual duration in seconds
  notes TEXT,
  parameters JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}'
);

-- Steps: Phases within experiments
CREATE TABLE steps (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  experiment_id UUID NOT NULL REFERENCES experiments(id) ON DELETE CASCADE,
  name VARCHAR(255) NOT NULL,
  description TEXT,
  type VARCHAR(50) NOT NULL CHECK (type IN ('wizard', 'robot', 'parallel', 'conditional')),
  order_index INTEGER NOT NULL,
  duration_estimate INTEGER, -- in seconds
  required BOOLEAN DEFAULT true,
  conditions JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(experiment_id, order_index)
);

-- Actions: Atomic tasks within steps
CREATE TABLE actions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  step_id UUID NOT NULL REFERENCES steps(id) ON DELETE CASCADE,
  name VARCHAR(255) NOT NULL,
  description TEXT,
  type VARCHAR(100) NOT NULL, -- e.g., 'speak', 'move', 'wait', 'collect_data'
  order_index INTEGER NOT NULL,
  parameters JSONB DEFAULT '{}',
  validation_schema JSONB,
  timeout INTEGER, -- in seconds
  retry_count INTEGER DEFAULT 0,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(step_id, order_index)
);
```

### Participants and Data Protection

```sql
-- Participants in studies
CREATE TABLE participants (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  participant_code VARCHAR(50) NOT NULL,
  email VARCHAR(255),
  name VARCHAR(255),
  demographics JSONB DEFAULT '{}', -- encrypted
  consent_given BOOLEAN DEFAULT false,
  consent_date TIMESTAMP,
  notes TEXT, -- encrypted
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE(study_id, participant_code)
);

-- Consent forms and documents
CREATE TABLE consent_forms (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  version INTEGER DEFAULT 1,
  title VARCHAR(255) NOT NULL,
  content TEXT NOT NULL,
  active BOOLEAN DEFAULT true,
  created_by UUID NOT NULL REFERENCES users(id),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  storage_path TEXT, -- path in MinIO
  UNIQUE(study_id, version)
);

-- Participant consent records
CREATE TABLE participant_consents (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  participant_id UUID NOT NULL REFERENCES participants(id) ON DELETE CASCADE,
  consent_form_id UUID NOT NULL REFERENCES consent_forms(id),
  signed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  signature_data TEXT, -- encrypted
  ip_address INET,
  storage_path TEXT, -- path to signed PDF in MinIO
  UNIQUE(participant_id, consent_form_id)
);
```

### Robot Platform Integration

```sql
-- Robot types/models
CREATE TABLE robots (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  name VARCHAR(255) NOT NULL,
  manufacturer VARCHAR(255),
  model VARCHAR(255),
  description TEXT,
  capabilities JSONB DEFAULT '[]',
  communication_protocol VARCHAR(50) CHECK (communication_protocol IN ('rest', 'ros2', 'custom')),
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Plugin definitions
CREATE TABLE plugins (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  robot_id UUID REFERENCES robots(id) ON DELETE CASCADE,
  name VARCHAR(255) NOT NULL,
  version VARCHAR(50) NOT NULL,
  description TEXT,
  author VARCHAR(255),
  repository_url TEXT,
  trust_level VARCHAR(20) CHECK (trust_level IN ('official', 'verified', 'community')),
  status VARCHAR(20) DEFAULT 'active' CHECK (status IN ('active', 'deprecated', 'disabled')),
  configuration_schema JSONB,
  action_definitions JSONB DEFAULT '[]',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  metadata JSONB DEFAULT '{}',
  UNIQUE(name, version)
);

-- Plugin installations per study
CREATE TABLE study_plugins (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  plugin_id UUID NOT NULL REFERENCES plugins(id),
  configuration JSONB DEFAULT '{}',
  installed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  installed_by UUID NOT NULL REFERENCES users(id),
  UNIQUE(study_id, plugin_id)
);
```

### Experiment Execution and Data Capture

```sql
-- Trial events log
CREATE TABLE trial_events (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  trial_id UUID NOT NULL REFERENCES trials(id) ON DELETE CASCADE,
  event_type VARCHAR(50) NOT NULL, -- 'action_started', 'action_completed', 'error', 'intervention'
  action_id UUID REFERENCES actions(id),
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  data JSONB DEFAULT '{}',
  created_by UUID REFERENCES users(id), -- NULL for system events
  INDEX idx_trial_events_trial_timestamp (trial_id, timestamp)
);

-- Wizard interventions/quick actions
CREATE TABLE wizard_interventions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  trial_id UUID NOT NULL REFERENCES trials(id) ON DELETE CASCADE,
  wizard_id UUID NOT NULL REFERENCES users(id),
  intervention_type VARCHAR(100) NOT NULL,
  description TEXT,
  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  parameters JSONB DEFAULT '{}',
  reason TEXT
);

-- Media captures (video, audio)
CREATE TABLE media_captures (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  trial_id UUID NOT NULL REFERENCES trials(id) ON DELETE CASCADE,
  media_type VARCHAR(20) CHECK (media_type IN ('video', 'audio', 'image')),
  storage_path TEXT NOT NULL, -- MinIO path
  file_size BIGINT,
  duration INTEGER, -- in seconds for video/audio
  format VARCHAR(20),
  resolution VARCHAR(20), -- for video
  start_timestamp TIMESTAMP,
  end_timestamp TIMESTAMP,
  metadata JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Sensor data captures
CREATE TABLE sensor_data (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  trial_id UUID NOT NULL REFERENCES trials(id) ON DELETE CASCADE,
  sensor_type VARCHAR(50) NOT NULL,
  timestamp TIMESTAMP NOT NULL,
  data JSONB NOT NULL,
  robot_state JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  INDEX idx_sensor_data_trial_timestamp (trial_id, timestamp)
);

-- Analysis annotations
CREATE TABLE annotations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  trial_id UUID NOT NULL REFERENCES trials(id) ON DELETE CASCADE,
  annotator_id UUID NOT NULL REFERENCES users(id),
  timestamp_start TIMESTAMP NOT NULL,
  timestamp_end TIMESTAMP,
  category VARCHAR(100),
  description TEXT,
  tags JSONB DEFAULT '[]',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Collaboration and Activity Tracking

```sql
-- Study activity log
CREATE TABLE activity_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID REFERENCES studies(id) ON DELETE CASCADE,
  user_id UUID REFERENCES users(id),
  action VARCHAR(100) NOT NULL,
  resource_type VARCHAR(50),
  resource_id UUID,
  description TEXT,
  ip_address INET,
  user_agent TEXT,
  metadata JSONB DEFAULT '{}',
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  INDEX idx_activity_logs_study_created (study_id, created_at DESC)
);

-- Comments and discussions
CREATE TABLE comments (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  parent_id UUID REFERENCES comments(id) ON DELETE CASCADE,
  resource_type VARCHAR(50) NOT NULL, -- 'experiment', 'trial', 'annotation'
  resource_id UUID NOT NULL,
  author_id UUID NOT NULL REFERENCES users(id),
  content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  deleted_at TIMESTAMP
);

-- File attachments
CREATE TABLE attachments (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  uploaded_by UUID NOT NULL REFERENCES users(id),
  filename VARCHAR(255) NOT NULL,
  mime_type VARCHAR(100),
  file_size BIGINT,
  storage_path TEXT NOT NULL, -- MinIO path
  description TEXT,
  resource_type VARCHAR(50),
  resource_id UUID,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### Data Export and Sharing

```sql
-- Export jobs
CREATE TABLE export_jobs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  requested_by UUID NOT NULL REFERENCES users(id),
  export_type VARCHAR(50) NOT NULL, -- 'full', 'trials', 'analysis', 'media'
  format VARCHAR(20) NOT NULL, -- 'json', 'csv', 'zip'
  filters JSONB DEFAULT '{}',
  status VARCHAR(20) DEFAULT 'pending' CHECK (status IN ('pending', 'processing', 'completed', 'failed')),
  storage_path TEXT,
  expires_at TIMESTAMP,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  completed_at TIMESTAMP,
  error_message TEXT
);

-- Shared resources
CREATE TABLE shared_resources (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  study_id UUID NOT NULL REFERENCES studies(id) ON DELETE CASCADE,
  resource_type VARCHAR(50) NOT NULL,
  resource_id UUID NOT NULL,
  shared_by UUID NOT NULL REFERENCES users(id),
  share_token VARCHAR(255) UNIQUE,
  permissions JSONB DEFAULT '["read"]',
  expires_at TIMESTAMP,
  access_count INTEGER DEFAULT 0,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

### System Configuration

```sql
-- System settings
CREATE TABLE system_settings (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  key VARCHAR(100) UNIQUE NOT NULL,
  value JSONB NOT NULL,
  description TEXT,
  updated_by UUID REFERENCES users(id),
  updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Audit log for compliance
CREATE TABLE audit_logs (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id),
  action VARCHAR(100) NOT NULL,
  resource_type VARCHAR(50),
  resource_id UUID,
  changes JSONB DEFAULT '{}',
  ip_address INET,
  user_agent TEXT,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  INDEX idx_audit_logs_created (created_at DESC)
);
```

## Indexes and Performance

```sql
-- Performance indexes
CREATE INDEX idx_users_email ON users(email) WHERE deleted_at IS NULL;
CREATE INDEX idx_studies_created_by ON studies(created_by) WHERE deleted_at IS NULL;
CREATE INDEX idx_trials_experiment ON trials(experiment_id);
CREATE INDEX idx_trials_status ON trials(status) WHERE status IN ('scheduled', 'in_progress');
CREATE INDEX idx_trial_events_type ON trial_events(event_type);
CREATE INDEX idx_participants_study ON participants(study_id);
CREATE INDEX idx_study_members_user ON study_members(user_id);
CREATE INDEX idx_media_captures_trial ON media_captures(trial_id);
CREATE INDEX idx_annotations_trial ON annotations(trial_id);

-- Full text search indexes
CREATE INDEX idx_studies_search ON studies USING GIN (to_tsvector('english', name || ' ' || COALESCE(description, '')));
CREATE INDEX idx_experiments_search ON experiments USING GIN (to_tsvector('english', name || ' ' || COALESCE(description, '')));
```

## Views for Common Queries

```sql
-- Active studies with member count
CREATE VIEW active_studies_summary AS
SELECT 
  s.id,
  s.name,
  s.status,
  s.created_at,
  u.name as creator_name,
  COUNT(DISTINCT sm.user_id) as member_count,
  COUNT(DISTINCT e.id) as experiment_count,
  COUNT(DISTINCT t.id) as trial_count
FROM studies s
LEFT JOIN users u ON s.created_by = u.id
LEFT JOIN study_members sm ON s.id = sm.study_id
LEFT JOIN experiments e ON s.id = e.study_id AND e.deleted_at IS NULL
LEFT JOIN trials t ON e.id = t.experiment_id
WHERE s.deleted_at IS NULL AND s.status = 'active'
GROUP BY s.id, s.name, s.status, s.created_at, u.name;

-- Trial execution summary
CREATE VIEW trial_execution_summary AS
SELECT 
  t.id,
  t.experiment_id,
  t.status,
  t.scheduled_at,
  t.started_at,
  t.completed_at,
  t.duration,
  p.participant_code,
  w.name as wizard_name,
  COUNT(DISTINCT te.id) as event_count,
  COUNT(DISTINCT wi.id) as intervention_count
FROM trials t
LEFT JOIN participants p ON t.participant_id = p.id
LEFT JOIN users w ON t.wizard_id = w.id
LEFT JOIN trial_events te ON t.id = te.trial_id
LEFT JOIN wizard_interventions wi ON t.id = wi.trial_id
GROUP BY t.id, t.experiment_id, t.status, t.scheduled_at, t.started_at, 
         t.completed_at, t.duration, p.participant_code, w.name;
```

## Database Functions and Triggers

```sql
-- Update timestamp trigger
CREATE OR REPLACE FUNCTION update_updated_at()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Apply update trigger to relevant tables
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
  FOR EACH ROW EXECUTE FUNCTION update_updated_at();
CREATE TRIGGER update_studies_updated_at BEFORE UPDATE ON studies
  FOR EACH ROW EXECUTE FUNCTION update_updated_at();
CREATE TRIGGER update_experiments_updated_at BEFORE UPDATE ON experiments
  FOR EACH ROW EXECUTE FUNCTION update_updated_at();
-- Apply to other tables as needed...

-- Function to check user permissions
CREATE OR REPLACE FUNCTION check_user_permission(
  p_user_id UUID,
  p_study_id UUID,
  p_action VARCHAR
) RETURNS BOOLEAN AS $$
DECLARE
  v_has_permission BOOLEAN;
BEGIN
  -- Check if user has permission through study membership or system role
  SELECT EXISTS (
    SELECT 1 FROM study_members sm
    WHERE sm.user_id = p_user_id 
    AND sm.study_id = p_study_id
    AND (
      sm.role = 'owner' OR
      p_action = ANY(sm.permissions::text[])
    )
  ) OR EXISTS (
    SELECT 1 FROM user_system_roles usr
    WHERE usr.user_id = p_user_id
    AND usr.role = 'administrator'
  ) INTO v_has_permission;
  
  RETURN v_has_permission;
END;
$$ LANGUAGE plpgsql;
```

## Migration Notes

1. Tables should be created in the order listed to respect foreign key constraints
2. Sensitive data in `participants`, `participant_consents`, and related tables should use PostgreSQL's pgcrypto extension for encryption
3. Consider partitioning large tables like `sensor_data` and `trial_events` by date for better performance
4. Implement regular vacuum and analyze schedules for optimal performance
5. Set up appropriate backup strategies for both PostgreSQL and MinIO data