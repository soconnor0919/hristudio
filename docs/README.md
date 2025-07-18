# HRIStudio Documentation

Welcome to the comprehensive documentation for HRIStudio - a web-based platform for standardizing and improving Wizard of Oz (WoZ) studies in Human-Robot Interaction research.

## 📚 Documentation Overview

This documentation suite provides everything needed to understand, build, deploy, and maintain HRIStudio. It's designed for AI agents, developers, and technical teams who will be implementing the platform.

### Core Documents

1. **[Project Overview](./project-overview.md)**
   - Executive summary and project goals
   - Core features and system architecture
   - User roles and permissions
   - Technology stack overview
   - Key concepts and success metrics

2. **[Database Schema](./database-schema.md)**
   - Complete PostgreSQL schema with Drizzle ORM
   - Table definitions and relationships
   - Indexes and performance optimizations
   - Views and stored procedures
   - Migration guidelines

3. **[API Routes](./api-routes.md)**
   - Comprehensive tRPC route documentation
   - Request/response schemas
   - Authentication requirements
   - WebSocket events
   - Rate limiting and error handling

4. **[Feature Requirements](./feature-requirements.md)**
   - Detailed user stories and acceptance criteria
   - Functional requirements by module
   - Non-functional requirements
   - UI/UX specifications
   - Integration requirements

5. **[Implementation Guide](./implementation-guide.md)**
   - Step-by-step technical implementation
   - Code examples and patterns
   - Frontend and backend architecture
   - Real-time features implementation
   - Testing strategies

6. **[Deployment & Operations](./deployment-operations.md)**
   - Infrastructure requirements
   - Vercel deployment strategies
   - Monitoring and observability
   - Backup and recovery procedures
   - Security operations

7. **[ROS2 Integration](./ros2-integration.md)**
   - rosbridge WebSocket architecture
   - Client-side ROS connection management
   - Message type definitions
   - Robot plugin implementation
   - Security considerations for robot communication

## 🚀 Quick Start for Developers

### Prerequisites
- Node.js 18+ with Bun package manager
- PostgreSQL 15+
- Docker and Docker Compose (for local development)
- S3-compatible storage (Cloudflare R2 recommended for Vercel)
- ROS2 with rosbridge_suite (for robot integration)

### Initial Setup
1. Clone the repository
2. Copy `.env.example` to `.env.local`
3. Run `docker-compose up -d` for local services
4. Run `bun install` to install dependencies
5. Run `bun db:migrate` to set up the database
6. Run `bun dev` to start the development server

### For AI Agents Building the Application

When implementing HRIStudio, follow this sequence:

1. **Start with Project Setup**
   - Use the Implementation Guide to set up the project structure
   - Follow the rules in `rules.txt` for coding standards
   - Reference the Project Overview for architectural decisions

2. **Implement Database Layer**
   - Use the Database Schema document to create all tables
   - Implement the schema files with Drizzle ORM
   - Set up relationships and indexes as specified

3. **Build API Layer**
   - Follow the API Routes document to implement all tRPC routes
   - Ensure proper authentication and authorization
   - Implement error handling and validation

4. **Create UI Components**
   - Reference Feature Requirements for UI specifications
   - Use shadcn/ui components exclusively
   - Follow the component patterns in Implementation Guide

5. **Add Real-time Features**
   - Implement WebSocket server for trial execution
   - Add real-time updates for wizard interface
   - Ensure proper state synchronization

6. **Implement Robot Integration**
   - Follow ROS2 Integration guide for robot plugins
   - Set up rosbridge on robot systems
   - Test WebSocket communication

7. **Deploy and Monitor**
   - Follow Deployment & Operations guide for Vercel
   - Set up monitoring and logging
   - Implement backup strategies

## 📋 Key Implementation Notes

### Architecture Principles
- **Modular Design**: Each feature is self-contained
- **Type Safety**: Full TypeScript with strict mode
- **Server-First**: Leverage React Server Components
- **Real-time**: WebSocket for live trial execution
- **Secure**: Role-based access control throughout

### Technology Choices
- **Next.js 15**: App Router for modern React patterns
- **tRPC**: Type-safe API communication
- **Drizzle ORM**: Type-safe database queries
- **NextAuth.js v5**: Authentication and authorization
- **shadcn/ui**: Consistent UI components
- **Cloudflare R2**: S3-compatible object storage
- **roslib.js**: WebSocket-based ROS2 communication
- **Vercel KV**: Edge-compatible caching (instead of Redis)

### Critical Features
1. **Visual Experiment Designer**: Drag-and-drop interface
2. **Wizard Interface**: Real-time control during trials
3. **Plugin System**: Extensible robot platform support
4. **Data Capture**: Comprehensive recording of all trial data
5. **Collaboration**: Multi-user support with role-based access

## 🔧 Development Workflow

### Code Organization
```
src/
├── app/           # Next.js app router pages
├── components/    # Reusable UI components
├── features/      # Feature-specific modules
├── lib/          # Core utilities and setup
├── server/       # Server-side code
└── types/        # TypeScript type definitions
```

### Testing Strategy
- Unit tests for utilities and hooks
- Integration tests for tRPC procedures
- E2E tests for critical user flows
- Performance testing for real-time features

### Deployment Pipeline
1. Run tests and type checking
2. Build Docker image
3. Run security scans
4. Deploy to staging
5. Run smoke tests
6. Deploy to production

## 🤝 Contributing Guidelines

### For AI Agents
- Always reference the documentation before implementing
- Follow the patterns established in the Implementation Guide
- Ensure all code follows the rules in `rules.txt`
- Implement comprehensive error handling
- Add proper TypeScript types for all code

### Code Quality Standards
- No `any` types in TypeScript
- All components must be accessible (WCAG 2.1 AA)
- API routes must have proper validation
- Database queries must be optimized
- Real-time features must handle disconnections

## 📞 Support and Resources

### Documentation Updates
This documentation is designed to be comprehensive and self-contained. If you identify gaps or need clarification:
1. Check all related documents first
2. Look for patterns in the Implementation Guide
3. Reference the rules.txt for coding standards

### Key Integration Points
- **Authentication**: NextAuth.js with database sessions
- **File Storage**: Cloudflare R2 with presigned URLs
- **Real-time**: WebSocket with reconnection logic (Edge Runtime compatible)
- **Robot Control**: ROS2 via rosbridge WebSocket protocol
- **Caching**: Vercel KV for serverless-compatible caching
- **Monitoring**: Vercel Analytics and structured logging

## 🎯 Success Criteria

The implementation is considered successful when:
- All features from Feature Requirements are implemented
- All API routes from API Routes document are functional
- Database schema matches the specification exactly
- Real-time features work reliably
- Security requirements are met
- Performance targets are achieved

## 📝 Document Versions

- **Version**: 1.0.0
- **Last Updated**: December 2024
- **Target Platform**: HRIStudio v1.0

Remember: This documentation represents a complete specification for building HRIStudio. Every technical decision and implementation detail has been carefully considered to create a robust, scalable platform for HRI research.