# HRIStudio: A Web-Based Wizard-of-Oz Platform for Human-Robot Interaction Research

A comprehensive platform designed to standardize and improve the reproducibility of Wizard of Oz (WoZ) studies in Human-Robot Interaction research. HRIStudio provides researchers with standardized tools for designing experiments, executing trials, and analyzing data while ensuring reproducibility and scientific rigor.

## Overview

HRIStudio addresses critical challenges in HRI research by providing a comprehensive experimental workflow management system with standardized terminology, visual experiment design tools, real-time wizard control interfaces, and comprehensive data capture capabilities.

### Key Problems Solved

- **Lack of standardized terminology** in WoZ studies
- **Poor documentation practices** leading to unreproducible experiments
- **Technical barriers** preventing non-programmers from conducting HRI research
- **Inconsistent wizard behavior** across trials
- **Limited data capture** and analysis capabilities in existing tools

### Core Features

- **Hierarchical Structure**: Study → Experiment → Trial → Step → Action
- **Visual Experiment Designer**: Drag-and-drop protocol creation with 26+ core blocks
- **Plugin System**: Extensible robot platform integration (RESTful, ROS2, custom)
- **Consolidated Wizard Interface**: 3-panel design with trial controls, horizontal timeline, and unified robot controls
- **Real-time Trial Execution**: Live wizard control with comprehensive data capture
- **Role-Based Access**: Administrator, Researcher, Wizard, Observer (4 distinct roles)
- **Unified Form Experiences**: 73% code reduction through standardized patterns
- **Enterprise DataTables**: Advanced filtering, pagination, export capabilities
- **Mock Robot Integration**: Complete simulation system for development and testing
- **Intelligent Control Flow**: Loops with implicit approval, branching logic, parallel execution

## Quick Start

### Prerequisites
- [Bun](https://bun.sh) (package manager)
- [PostgreSQL](https://postgresql.org) 14+
- [Docker](https://docker.com) (recommended)

### Installation

```bash
# Clone the repository
git clone <repo-url> hristudio
cd hristudio

# Install dependencies
bun install

# Start database (Docker)
bun run docker:up

# Setup database schema and seed data
bun db:push
bun db:seed

# Start development server
bun dev
```

### Default Login Credentials

- **Administrator**: `sean@soconnor.dev` / `password123`
- **Researcher**: `alice.rodriguez@university.edu` / `password123`
- **Wizard**: `emily.watson@lab.edu` / `password123`
- **Observer**: `maria.santos@tech.edu` / `password123`

## Technology Stack

- **Framework**: Next.js 15 with App Router and React 19 RC
- **Language**: TypeScript (strict mode) - 100% type safety throughout
- **Database**: PostgreSQL with Drizzle ORM for type-safe operations
- **Authentication**: NextAuth.js v5 with database sessions and JWT
- **API**: tRPC for end-to-end type-safe client-server communication
- **UI**: Tailwind CSS + shadcn/ui (built on Radix UI primitives)
- **Storage**: Cloudflare R2 (S3-compatible) for media files
- **Deployment**: Vercel serverless platform with Edge Runtime
- **Package Manager**: Bun exclusively
- **Real-time**: WebSocket with Edge Runtime compatibility

## Architecture

### Core Components

#### 1. Visual Experiment Designer
- **Repository-based block system** with 26+ core blocks across 4 categories
- **Plugin architecture** for both core functionality and robot actions
- Context-sensitive help and best practice guidance
- **Core Block Categories**:
  - **Events (4)**: Trial triggers, speech detection, timers, key presses
  - **Wizard Actions (6)**: Speech, gestures, object handling, rating, notes
  - **Control Flow (8)**: Loops, conditionals, parallel execution, error handling
  - **Observation (8)**: Behavioral coding, timing, recording, surveys, sensors

#### 2. Robot Platform Integration
- **Unified plugin architecture** for both core blocks and robot actions
- Abstract action definitions with platform-specific translations
- Support for RESTful APIs, ROS2, and custom protocols
- **Repository system** for plugin distribution and management
- Plugin Store with trust levels (Official, Verified, Community)

#### 3. Adaptive Wizard Interface
- **3-Panel Design**: Trial controls (left), horizontal timeline (center), robot control & status (right)
- **Horizontal Step Progress**: Non-scrolling step navigation with visual progress indicators
- **Consolidated Robot Controls**: Single location for connection, autonomous life, actions, and monitoring
- Real-time experiment execution dashboard
- Step-by-step guidance for consistent execution
- Quick actions for unscripted interventions
- Live video feed integration
- Timestamped event logging

#### 4. Comprehensive Data Management
- Automatic capture of all experimental data
- Synchronized multi-modal data streams
- Encrypted storage for sensitive participant data
- Role-based access control for data security

## User Roles

- **Administrator**: Full system access, user management, plugin installation
- **Researcher**: Create studies, design experiments, manage teams, analyze data
- **Wizard**: Execute trials, control robots, make real-time decisions
- **Observer**: Read-only access, monitor trials, add annotations

## Development

### Available Scripts

```bash
# Development
bun dev                    # Start development server
bun build                  # Build for production
bun start                  # Start production server

# Database
bun db:push               # Push schema changes
bun db:studio             # Open database GUI
bun db:seed               # Seed with comprehensive test data
bun db:seed:simple        # Seed with minimal test data
bun db:seed:plugins       # Seed plugin repositories and plugins
bun db:seed:core-blocks   # Seed core block system

# Code Quality
bun typecheck             # TypeScript validation
bun lint                  # ESLint with autofix
bun format:check          # Prettier formatting check
bun format:write          # Apply Prettier formatting

# Docker
bun run docker:up         # Start PostgreSQL container
bun run docker:down       # Stop PostgreSQL container
```

### Project Structure

```
src/
├── app/                      # Next.js App Router pages
│   ├── (auth)/              # Authentication pages
│   ├── (dashboard)/         # Main application pages
│   │   ├── studies/         # Study management
│   │   ├── experiments/     # Experiment design & designer
│   │   ├── participants/    # Participant management
│   │   ├── trials/          # Trial execution and monitoring
│   │   ├── plugins/         # Plugin management
│   │   └── admin/           # System administration
│   └── api/                 # API routes and webhooks
├── components/              # UI components
│   ├── ui/                  # shadcn/ui base components
│   ├── experiments/         # Experiment designer components
│   ├── plugins/             # Plugin management components
│   └── [entity]/            # Entity-specific components
├── server/                  # Backend code
│   ├── api/routers/         # tRPC routers (11 total)
│   ├── auth/               # NextAuth.js v5 configuration
│   └── db/                 # Database schema and setup
├── lib/                    # Utilities and configurations
└── hooks/                  # Custom React hooks
```

### Database Schema

31 tables with comprehensive relationships:
- **Core Entities**: users, studies, experiments, participants, trials
- **Execution**: trial_events, steps, actions
- **Integration**: robots, plugins, plugin_repositories
- **Collaboration**: study_members, comments, attachments
- **System**: roles, permissions, audit_logs

## Core Concepts

### Experiment Lifecycle
1. **Design Phase**: Visual experiment creation using block-based designer
2. **Configuration Phase**: Parameter setup and team assignment
3. **Execution Phase**: Real-time trial execution with wizard control
4. **Analysis Phase**: Data review and insight generation
5. **Sharing Phase**: Export and collaboration features

### Plugin Architecture
- **Action Definitions**: Abstract robot capabilities
- **Parameter Schemas**: Type-safe configuration with validation
- **Communication Adapters**: Platform-specific implementations
- **Repository System**: Centralized plugin distribution

## Documentation

Comprehensive documentation available in the `docs/` folder:

- **[Quick Reference](docs/quick-reference.md)**: 5-minute setup guide and essential commands
- **[Project Overview](docs/project-overview.md)**: Complete feature overview and architecture
- **[Implementation Details](docs/implementation-details.md)**: Architecture decisions and patterns
- **[Database Schema](docs/database-schema.md)**: Complete PostgreSQL schema documentation
- **[API Routes](docs/api-routes.md)**: Comprehensive tRPC API reference
- **[Core Blocks System](docs/core-blocks-system.md)**: Repository-based block architecture
- **[Plugin System](docs/plugin-system-implementation-guide.md)**: Robot integration guide
- **[Project Status](docs/project-status.md)**: Current completion status (98% complete)

## Research Paper

This platform is described in our research paper: **"A Web-Based Wizard-of-Oz Platform for Collaborative and Reproducible Human-Robot Interaction Research"**

Key contributions:
- Assessment of state-of-the-art in WoZ study tools
- Identification of reproducibility challenges in HRI research
- Novel architectural approach with hierarchical experiment structure
- Repository-based plugin system for robot integration
- Comprehensive evaluation of platform effectiveness

Full paper available at: [docs/paper.md](docs/paper.md)

## Current Status

- **Production Ready**: Complete platform with all major features
- **31 Database Tables**: Comprehensive data model
- **12 tRPC Routers**: Complete API coverage
- **26+ Core Blocks**: Repository-based experiment building blocks
- **4 User Roles**: Complete role-based access control
- **Plugin System**: Extensible robot integration architecture
- **Trial System**: Unified design with real-time execution capabilities

## NAO6 Robot Integration

Complete NAO6 robot integration is available in the separate **[nao6-hristudio-integration](../nao6-hristudio-integration/)** repository.

### Features
- Complete ROS2 driver integration for NAO V6.0
- WebSocket communication via rosbridge
- 9 robot actions: speech, movement, gestures, sensors, LEDs
- Real-time control from wizard interface
- Production-ready with NAOqi 2.8.7.4

### Quick Start
```bash
# Start NAO integration
cd ~/naoqi_ros2_ws
source install/setup.bash
ros2 launch nao_launch nao6_hristudio.launch.py nao_ip:=nao.local

# Start HRIStudio
cd ~/Documents/Projects/hristudio
bun dev

# Test at: http://localhost:3000/nao-test
```

### Documentation
- **[Integration README](../nao6-hristudio-integration/README.md)** - Complete setup guide
- **[NAO6 Quick Reference](docs/nao6-quick-reference.md)** - Essential commands
- **[Installation Guide](../nao6-hristudio-integration/docs/INSTALLATION.md)** - Detailed setup
- **[Troubleshooting](../nao6-hristudio-integration/docs/TROUBLESHOOTING.md)** - Common issues

## Deployment

### Vercel (Recommended)
```bash
# Install Vercel CLI
bun add -g vercel

# Deploy
vercel --prod
```

### Environment Variables
```bash
DATABASE_URL=postgresql://...
NEXTAUTH_URL=https://your-domain.com
NEXTAUTH_SECRET=your-secret
CLOUDFLARE_R2_ACCOUNT_ID=...
CLOUDFLARE_R2_ACCESS_KEY_ID=...
CLOUDFLARE_R2_SECRET_ACCESS_KEY=...
CLOUDFLARE_R2_BUCKET_NAME=hristudio-files
```

## Contributing

1. Read the project documentation in `docs/`
2. Follow the established patterns in `.rules`
3. Use TypeScript strict mode throughout
4. Implement proper error handling and loading states
5. Test with multiple user roles
6. Use `bun` exclusively for package management

## License

[License information to be added]

## Citation

If you use HRIStudio in your research, please cite our paper:

```bibtex
[Citation to be added once published]
```

---

**HRIStudio**: Advancing the reproducibility and accessibility of Human-Robot Interaction research through standardized, collaborative tools.