# HRIStudio Architecture Decisions

## Overview

This document captures key architectural decisions made for HRIStudio, including technology choices, deployment strategies, and integration approaches. These decisions are based on modern web development best practices, scalability requirements, and the specific needs of HRI research.

## Technology Stack Decisions

### 1. Next.js 15 (not 14)

**Decision**: Use Next.js 15 with React 19 RC

**Rationale**:
- Latest stable features and performance improvements
- Better server component support
- Improved caching mechanisms
- Enhanced TypeScript support
- Future-proof for upcoming React features

**Implementation**:
```json
{
  "dependencies": {
    "next": "^15.0.0",
    "react": "rc",
    "react-dom": "rc"
  }
}
```

### 2. Vercel Deployment (not self-hosted)

**Decision**: Deploy on Vercel's serverless platform

**Rationale**:
- Automatic scaling without infrastructure management
- Built-in CI/CD with GitHub integration
- Global CDN for static assets
- Edge runtime support for real-time features
- Simplified deployment process
- Cost-effective for research projects

**Implications**:
- Use Vercel KV instead of Redis
- Edge-compatible WebSocket implementation
- Serverless function optimization
- Environment variable management via Vercel CLI

### 3. No Redis - Alternative Solutions

**Decision**: Avoid Redis dependency, use platform-native solutions

**Alternatives Implemented**:

#### For Caching:
- **Vercel KV**: Serverless Redis-compatible key-value store
- **In-memory Maps**: For real-time state during WebSocket sessions
- **Edge Config**: For feature flags and configuration

#### For Real-time State:
```typescript
// In-memory stores for active sessions
export const trialStateStore = new Map<string, TrialState>();
export const activeConnections = new Map<string, Set<WebSocket>>();
```

#### For Background Jobs:
- Use Vercel Cron Jobs for scheduled tasks
- Implement queue with database-backed job table
- Use serverless functions with retry logic

**Rationale**:
- Reduces infrastructure complexity
- Better integration with Vercel platform
- Lower operational overhead
- Sufficient for research scale

### 4. ROS2 Integration via rosbridge

**Decision**: Use WebSocket-based rosbridge protocol

**Architecture**:
```
HRIStudio (Vercel) → WebSocket → rosbridge_server → ROS2 Robot
```

**Rationale**:
- No ROS2 installation required on server
- Works with serverless architecture
- Language-agnostic communication
- Well-established protocol
- Supports real-time control

**Implementation Details**:
- Use `roslib.js` for client-side communication
- Plugin system abstracts robot-specific details
- Support for topics, services, and actions
- SSL/TLS for production security

### 5. Docker for Development Only

**Decision**: Use Docker Compose for local development, not production

**Development Stack**:
```yaml
services:
  db:      # PostgreSQL 15
  minio:   # S3-compatible storage
```

**Production Stack**:
- Vercel for application hosting
- Managed PostgreSQL (Vercel Postgres, Neon, or Supabase)
- Cloudflare R2 for object storage

**Rationale**:
- Simplifies local development setup
- Production uses managed services
- Better performance with platform-native solutions
- Reduced operational complexity

## Data Architecture Decisions

### 1. PostgreSQL with Drizzle ORM

**Decision**: PostgreSQL as primary database with Drizzle ORM

**Rationale**:
- JSONB support for flexible metadata
- Strong consistency for research data
- Excellent TypeScript integration with Drizzle
- Built-in full-text search
- Proven reliability

### 2. Cloudflare R2 for Object Storage

**Decision**: Use Cloudflare R2 instead of AWS S3

**Rationale**:
- No egress fees
- S3-compatible API
- Better pricing for research budgets
- Global edge network
- Integrated with Cloudflare ecosystem

### 3. Hierarchical Data Model

**Decision**: Implement Study → Experiment → Step → Action hierarchy

**Rationale**:
- Matches research methodology
- Clear ownership and permissions
- Efficient querying patterns
- Natural audit trail

## Security Decisions

### 1. Database-Level Encryption

**Decision**: Encrypt sensitive participant data at database level

**Implementation**:
- Use PostgreSQL's pgcrypto extension
- Encrypt PII fields
- Key management via environment variables

### 2. Role-Based Access Control

**Decision**: Four-tier role system

**Roles**:
1. **Administrator**: System-wide access
2. **Researcher**: Study creation and management
3. **Wizard**: Trial execution only
4. **Observer**: Read-only access

### 3. WebSocket Security

**Decision**: Token-based authentication for WebSocket connections

**Implementation**:
- Session tokens for authentication
- SSL/TLS required in production
- Connection-specific permissions
- Automatic reconnection with auth

## Performance Decisions

### 1. Server Components First

**Decision**: Maximize use of React Server Components

**Rationale**:
- Reduced client bundle size
- Better SEO
- Faster initial page loads
- Simplified data fetching

### 2. Edge Runtime for Real-time

**Decision**: Use Vercel Edge Runtime for WebSocket handling

**Benefits**:
- Global distribution
- Low latency
- Automatic scaling
- WebSocket support

### 3. Optimistic UI Updates

**Decision**: Implement optimistic updates for better UX

**Implementation**:
- tRPC mutations with optimistic updates
- Rollback on errors
- Visual feedback during operations

## Integration Decisions

### 1. Plugin Architecture for Robots

**Decision**: Modular plugin system for robot support

**Structure**:
```typescript
interface RobotPlugin {
  id: string;
  name: string;
  actions: ActionDefinition[];
  executeAction(action, params): Promise<Result>;
}
```

### 2. Event-Driven Architecture

**Decision**: Use events for loose coupling

**Applications**:
- Trial execution events
- System notifications
- Audit logging
- Real-time updates

### 3. API Design with tRPC

**Decision**: tRPC for type-safe API communication

**Benefits**:
- End-to-end type safety
- No API documentation needed
- Automatic client generation
- Smaller bundle sizes

## Operational Decisions

### 1. Monitoring Strategy

**Decision**: Platform-native monitoring

**Stack**:
- Vercel Analytics for web vitals
- Sentry for error tracking
- PostHog for product analytics
- Custom metrics API

### 2. Backup Strategy

**Decision**: Automated daily backups

**Implementation**:
- Database: Point-in-time recovery
- Media: R2 object versioning
- Configuration: Git version control

### 3. Development Workflow

**Decision**: GitHub-centric workflow

**Process**:
- Feature branches
- PR-based reviews
- Automated testing
- Preview deployments
- Protected main branch

## Future Considerations

### 1. Multi-Region Support

**Preparation**:
- Database read replicas
- Edge caching strategy
- Regional storage buckets

### 2. Mobile Application

**Considerations**:
- React Native for code sharing
- Offline-first architecture
- WebSocket compatibility

### 3. AI Integration

**Potential Features**:
- Experiment design suggestions
- Automated analysis
- Natural language commands
- Anomaly detection

## Decision Log

| Date | Decision | Rationale | Status |
|------|----------|-----------|---------|
| 2024-12 | Next.js 15 | Latest features, better performance | Approved |
| 2024-12 | Vercel deployment | Simplified ops, automatic scaling | Approved |
| 2024-12 | No Redis | Platform-native alternatives | Approved |
| 2024-12 | rosbridge for ROS2 | Serverless compatible | Approved |
| 2024-12 | Cloudflare R2 | Cost-effective storage | Approved |

## Conclusion

These architectural decisions prioritize:
1. **Developer Experience**: Type safety, modern tooling
2. **Operational Simplicity**: Managed services, serverless
3. **Research Needs**: Data integrity, reproducibility
4. **Scalability**: Automatic scaling, edge distribution
5. **Cost Efficiency**: Pay-per-use pricing, no egress fees

The architecture is designed to evolve with the project while maintaining stability for research operations.