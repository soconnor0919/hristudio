# HRIStudio Seed Script Documentation

## Overview

The HRIStudio seed script (`scripts/seed-dev.ts`) provides a comprehensive development database with realistic test data for all major entities in the system. This script is designed to give developers and testers a fully functional environment with diverse scenarios to work with.

## Quick Start

Run the seed script with:

```bash
bun run db:seed
```

**Note**: This script will completely clean the database before seeding new data.

## Default Login Credentials

### Primary Administrator Account
- **Email**: `sean@soconnor.dev`
- **Password**: `password123`
- **Role**: Administrator
- **Access**: Full system access and user management

### Additional Test Users
All users use the same password: `password123`

- **alice.rodriguez@university.edu** (Researcher)
- **bob.chen@research.org** (Researcher)  
- **emily.watson@lab.edu** (Wizard)
- **maria.santos@tech.edu** (Researcher)

## Seeded Data Structure

### 🤖 Robots (3 total)
1. **NAO Robot** (SoftBank Robotics, V6)
   - Capabilities: Speech, movement, vision, touch, LEDs
   - Status: Available
   - Connection: WiFi

2. **Pepper Robot** (SoftBank Robotics)
   - Capabilities: Speech, movement, vision, touch, tablet
   - Status: Available
   - Connection: WiFi

3. **TurtleBot3** (ROBOTIS, Burger)
   - Capabilities: Movement, vision, LiDAR
   - Status: Maintenance
   - Connection: ROS2

### 📚 Studies (3 total)

#### 1. Robot-Assisted Learning in Elementary Education
- **Owner**: Alice Rodriguez
- **Institution**: University of Technology
- **IRB Protocol**: IRB-2024-001
- **Status**: Active
- **Team**: Alice (Owner), Emily (Wizard), Sean (Observer)
- **Focus**: Mathematics learning for elementary students

#### 2. Elderly Care Robot Acceptance Study
- **Owner**: Bob Chen
- **Institution**: Research Institute for Aging
- **IRB Protocol**: IRB-2024-002
- **Status**: Active
- **Team**: Bob (Owner), Alice (Researcher), Emily (Wizard)
- **Focus**: Companion robots in assisted living

#### 3. Navigation Robot Trust Study
- **Owner**: Maria Santos
- **Institution**: Tech University
- **IRB Protocol**: IRB-2024-003
- **Status**: Draft
- **Team**: Maria (Owner), Sean (Researcher)
- **Focus**: Trust in autonomous navigation robots

### 👤 Participants (8 total)

#### Elementary Education Study
- **CHILD_001**: Alex Johnson (8, male, grade 3)
- **CHILD_002**: Emma Davis (9, female, grade 4)
- **CHILD_003**: Oliver Smith (8, male, grade 3)

#### Elderly Care Study
- **ELDERLY_001**: Margaret Thompson (78, female, retired teacher)
- **ELDERLY_002**: Robert Wilson (82, male, retired engineer)
- **ELDERLY_003**: Dorothy Garcia (75, female, retired nurse)

#### Navigation Study
- **ADULT_001**: James Miller (28, male, engineer)
- **ADULT_002**: Sarah Brown (34, female, teacher)

### 🧪 Experiments (5 total)

1. **Math Tutoring Session** (NAO Robot)
   - Study: Elementary Education
   - Duration: 30 minutes
   - Status: Ready

2. **Reading Comprehension Support** (NAO Robot)
   - Study: Elementary Education
   - Duration: 25 minutes
   - Status: Testing

3. **Daily Companion Interaction** (Pepper Robot)
   - Study: Elderly Care
   - Duration: 45 minutes
   - Status: Ready

4. **Medication Reminder Protocol** (Pepper Robot)
   - Study: Elderly Care
   - Duration: 15 minutes
   - Status: Draft

5. **Campus Navigation Assistance** (TurtleBot3)
   - Study: Navigation Trust
   - Duration: 20 minutes
   - Status: Ready

### 📋 Experiment Steps (8 total)

Each experiment includes detailed steps with specific durations and requirements:

- **Welcome and Introduction** steps for user engagement
- **Task-specific steps** (math problems, companion interaction, navigation)
- **Feedback and encouragement** phases
- **Health check-ins** for elderly participants

### 🏃 Trials (7 total)

#### Completed Trials (3)
- Alex Johnson: Math tutoring (27 min, successful)
- Emma Davis: Math tutoring (26 min, successful)
- Margaret Thompson: Companion interaction (45 min, successful)

#### In-Progress Trial (1)
- Oliver Smith: Math tutoring (currently active)

#### Scheduled Trials (3)
- Robert Wilson: Companion interaction (tomorrow)
- James Miller: Navigation assistance (next week)
- Alex Johnson: Follow-up math session (next week)

### 📝 Trial Events (18 total)

Comprehensive event logs for completed and in-progress trials including:
- Trial start/completion timestamps
- Step progression tracking
- Robot action logs
- Performance metrics
- Duration tracking

## Database Schema Coverage

The seed script populates the following tables:
- ✅ `users` - Authentication and user profiles
- ✅ `userSystemRoles` - Role-based access control
- ✅ `robots` - Available robot platforms
- ✅ `studies` - Research study containers
- ✅ `studyMembers` - Study team memberships
- ✅ `participants` - Study participants with demographics
- ✅ `experiments` - Experimental protocols
- ✅ `steps` - Experiment step definitions
- ✅ `trials` - Individual trial instances
- ✅ `trialEvents` - Detailed trial execution logs

## Use Cases for Testing

### Authentication & Authorization
- Test login with different user roles
- Verify role-based access restrictions
- Test study membership permissions

### Study Management
- Create new studies and experiments
- Manage study team memberships
- Test study status workflows

### Experiment Design
- Modify existing experiment templates
- Create new experimental steps
- Test robot integration scenarios

### Trial Execution
- Practice wizard interface with in-progress trial
- Review completed trial data
- Test trial scheduling and management

### Data Analysis
- Analyze trial performance metrics
- Export trial event data
- Generate study reports

### Participant Management
- Add new participants to studies
- Manage consent and demographics
- Test participant communication

## Realistic Scenarios

The seed data includes realistic scenarios based on actual HRI research:

1. **Child-Robot Learning**: Age-appropriate math tutoring with emotional support
2. **Elderly Care**: Health monitoring and social companionship
3. **Navigation Trust**: Public space robot guidance and safety
4. **Multi-session Studies**: Follow-up trials and retention testing
5. **Team Collaboration**: Multi-role study teams with different permissions

## Development Workflow

1. **Reset Database**: Run seed script to start fresh
2. **Login**: Use admin account for full access
3. **Explore**: Navigate through studies, experiments, and trials
4. **Test Features**: Create new entities or modify existing ones
5. **Verify**: Check role-based permissions with different user accounts

## Data Consistency

The seed script ensures:
- Proper foreign key relationships
- Realistic timestamps and durations
- Appropriate role assignments
- Valid experimental workflows
- Comprehensive audit trails

## Security Notes

- All passwords are hashed using bcrypt
- Sensitive participant data is stored in JSONB fields (ready for encryption)
- Role-based access is properly configured
- Admin privileges are limited to designated accounts

## Future Enhancements

The seed script can be extended to include:
- Plugin system data
- Media capture references
- Consent form templates
- Export job histories
- Advanced robot configurations

This comprehensive seed data provides a solid foundation for developing and testing all aspects of the HRIStudio platform.