# HRIStudio Project Overview

## Executive Summary

HRIStudio is a web-based platform designed to standardize and improve the reproducibility of Wizard of Oz (WoZ) studies in Human-Robot Interaction (HRI) research. The platform addresses critical challenges in HRI research by providing a comprehensive experimental workflow management system with standardized terminology, visual experiment design tools, real-time wizard control interfaces, and comprehensive data capture capabilities.

## Project Goals

### Primary Objectives
1. **Enhance Scientific Rigor**: Standardize WoZ study methodologies to improve reproducibility
2. **Lower Barriers to Entry**: Make HRI research accessible to researchers without deep robot programming expertise
3. **Enable Collaboration**: Support multi-user workflows with role-based access control
4. **Ensure Data Integrity**: Comprehensive capture and secure storage of all experimental data
5. **Support Multiple Robot Platforms**: Provide a plugin-based architecture for robot integration

### Key Problems Addressed
- Lack of standardized terminology in WoZ studies
- Poor documentation practices leading to unreproducible experiments
- Technical barriers preventing non-programmers from conducting HRI research
- Inconsistent wizard behavior across trials
- Limited data capture and analysis capabilities in existing tools

## Core Features

### 1. Hierarchical Experiment Structure
- **Study**: Top-level container for research projects
- **Experiment**: Parameterized protocol templates within a study
- **Trial**: Executable instances of experiments with specific participants
- **Step**: Distinct phases in the execution sequence
- **Action**: Atomic tasks for wizards or robots

### 2. Visual Experiment Designer (EDE)
- Drag-and-drop interface for creating experiment workflows
- No-code solution for experiment design
- Context-sensitive help and best practice guidance
- Automatic generation of robot-specific action components
- Parameter configuration with validation

### 3. Adaptive Wizard Interface
- Real-time experiment execution dashboard
- Step-by-step guidance for consistent execution
- Quick actions for unscripted interventions
- Live video feed integration
- Timestamped event logging
- Customizable per-experiment controls

### 4. Robot Platform Integration
- Plugin-based architecture for robot support
- Abstract action definitions with platform-specific translations
- Support for RESTful APIs, ROS2, and custom protocols
- Plugin Store with trust levels (Official, Verified, Community)
- Version tracking for reproducibility

### 5. Comprehensive Data Management
- Automatic capture of all experimental data
- Synchronized multi-modal data streams (video, audio, logs, sensor data)
- Encrypted storage for sensitive participant data
- Role-based access control for data security
- Export capabilities for analysis tools

### 6. Collaboration Features
- Multi-user support with defined roles
- Project dashboards with status tracking
- Token-based resource sharing for external collaboration
- Activity logs and audit trails
- Support for double-blind study designs
- Comment system for team communication
- File attachments for supplementary materials

## System Architecture

### Three-Layer Architecture

#### 1. User Interface Layer
- **Experiment Designer**: Visual programming interface for creating experiments
- **Wizard Interface**: Real-time control and monitoring during trials
- **Playback & Analysis**: Data exploration and visualization tools
- **Administration Panel**: System configuration and user management
- **Plugin Store**: Browse and install robot platform integrations

#### 2. Data Management Layer
- **Database**: PostgreSQL for structured data and metadata
- **Object Storage**: MinIO (S3-compatible) for media files
- **Access Control**: Role-based permissions system
- **API Layer**: tRPC for type-safe client-server communication
- **Data Models**: Drizzle ORM for database operations

#### 3. Robot Integration Layer
- **Plugin System**: Modular robot platform support
- **Action Translation**: Abstract to platform-specific command mapping
- **Communication Protocols**: Support for REST, ROS2, and custom protocols
- **State Management**: Robot status tracking and synchronization

## User Roles and Permissions

### Administrator
- Full system access
- User management capabilities
- System configuration
- Plugin installation and management
- Database maintenance

### Researcher
- Create and manage studies
- Design experiments
- Manage team members
- View all trial data
- Export data for analysis

### Wizard
- Execute assigned experiments
- Control robot during trials
- Make real-time decisions
- View experiment instructions
- Access quick actions

### Observer
- Read-only access to experiments
- Monitor live trial execution
- Add notes and annotations
- View historical data
- No control capabilities

## Technology Stack

### Frontend
- **Framework**: Next.js 14+ (App Router)
- **UI Components**: shadcn/ui (built on Radix UI)
- **Styling**: Tailwind CSS
- **State Management**: nuqs (URL state), React Server Components
- **Forms**: React Hook Form with Zod validation
- **Real-time**: WebSockets for live updates

### Backend
- **Runtime**: Node.js with Bun package manager
- **API**: tRPC for type-safe endpoints
- **Database**: PostgreSQL with Drizzle ORM
- **Authentication**: NextAuth.js v5 (Auth.js)
- **File Storage**: MinIO (S3-compatible object storage)
- **Background Jobs**: Bull queue with Redis

### Infrastructure
- **Containerization**: Docker and Docker Compose
- **Development**: Hot reloading, TypeScript strict mode
- **Testing**: Vitest for unit tests, Playwright for E2E
- **CI/CD**: GitHub Actions
- **Monitoring**: OpenTelemetry integration

## Key Concepts

### Experiment Lifecycle
1. **Design Phase**: Researchers create experiment templates using visual designer
2. **Configuration Phase**: Set parameters and assign team members
3. **Execution Phase**: Wizards run trials with participants
4. **Analysis Phase**: Review captured data and generate insights
5. **Sharing Phase**: Export or share experiment materials

### Data Flow
1. **Input**: Experiment designs, wizard actions, robot responses, sensor data
2. **Processing**: Action translation, state management, data synchronization
3. **Storage**: Structured data in PostgreSQL, media files in MinIO
4. **Output**: Real-time updates, analysis reports, exported datasets

### Plugin Architecture
- **Action Definitions**: Abstract representations of robot capabilities
- **Parameter Schemas**: Type-safe configuration with validation
- **Communication Adapters**: Platform-specific protocol implementations
- **Version Management**: Semantic versioning for compatibility

### Token-Based Sharing Model
- **Share Links**: Generate unique tokens for resource access
- **Permission Control**: Granular permissions (read, comment, annotate)
- **Expiration**: Time-limited access for security
- **Access Tracking**: Monitor usage and analytics
- **Public Access**: No authentication required for shared resources
- **Revocation**: Instant access removal when needed

## Development Principles

### Code Quality
- TypeScript throughout with strict type checking
- Functional programming patterns (avoid classes)
- Comprehensive error handling
- Extensive logging for debugging
- Clean architecture with separation of concerns

### User Experience
- Mobile-first responsive design
- Progressive enhancement
- Optimistic UI updates
- Comprehensive loading states
- Intuitive error messages

### Performance
- Server-side rendering where possible
- Lazy loading for non-critical components
- Image optimization (WebP, proper sizing)
- Database query optimization
- Caching strategies

### Security
- Role-based access control at all levels
- Data encryption at rest and in transit
- Input validation and sanitization
- Rate limiting on API endpoints
- Audit logging for compliance

## Success Metrics

### Technical Metrics
- Page load time < 2 seconds
- API response time < 200ms (p95)
- High uptime for critical services
- Zero data loss incidents
- Support for 100+ concurrent users

### User Success Metrics
- Time to create first experiment < 30 minutes
- High trial execution consistency
- Complete data capture
- High user satisfaction score
- Active monthly users growth

## Future Considerations

### Planned Enhancements
- AI-powered experiment design suggestions
- Advanced analytics and visualization tools
- Mobile app for wizard control
- Cloud-hosted SaaS offering
- Integration with popular analysis tools (R, Python)

### Extensibility Points
- Custom plugin development SDK
- Webhook system for external integrations
- Custom report generation
- API for third-party tools
- Theming and white-labeling support