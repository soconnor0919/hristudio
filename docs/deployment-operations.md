# HRIStudio Deployment and Operations Guide

## Overview

This guide covers deployment strategies, operational procedures, monitoring, and maintenance for HRIStudio in both development and production environments.

## Table of Contents

1. [Deployment Strategies](#deployment-strategies)
2. [Infrastructure Requirements](#infrastructure-requirements)
3. [Environment Configuration](#environment-configuration)
4. [Deployment Process](#deployment-process)
5. [Monitoring and Observability](#monitoring-and-observability)
6. [Backup and Recovery](#backup-and-recovery)
7. [Scaling Strategies](#scaling-strategies)
8. [Security Operations](#security-operations)
9. [Maintenance Procedures](#maintenance-procedures)
10. [Troubleshooting](#troubleshooting)

## Deployment Strategies

### Development Environment

```yaml
# docker-compose.dev.yml
version: '3.8'

services:
  db:
    image: postgres:15
    environment:
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: postgres
      POSTGRES_DB: hristudio_dev
    ports:
      - "5432:5432"
    volumes:
      - postgres_dev_data:/var/lib/postgresql/data

  minio:
    image: minio/minio
    ports:
      - "9000:9000"
      - "9001:9001"
    environment:
      MINIO_ROOT_USER: minioadmin
      MINIO_ROOT_PASSWORD: minioadmin
    command: server --console-address ":9001" /data
    volumes:
      - minio_dev_data:/data

volumes:
  postgres_dev_data:
  minio_dev_data:
```

### Production Environment - Vercel Deployment

#### Vercel Configuration

```json
// vercel.json
{
  "framework": "nextjs",
  "buildCommand": "bun run build",
  "installCommand": "bun install",
  "regions": ["iad1", "sfo1"],
  "functions": {
    "app/api/trpc/[trpc]/route.ts": {
      "maxDuration": 60
    },
    "app/api/ws/route.ts": {
      "maxDuration": 60,
      "runtime": "edge"
    }
  },
  "env": {
    "NODE_ENV": "production"
  }
}
```

#### External Services Configuration

Since Vercel is serverless, we need external managed services:

1. **Database**: Use a managed PostgreSQL service
   - Recommended: Neon, Supabase, or PostgreSQL on DigitalOcean
   - Connection pooling is critical for serverless

2. **Object Storage**: Use a cloud S3-compatible service
   - Recommended: AWS S3, Cloudflare R2, or Backblaze B2
   - MinIO for development only

3. **WebSocket**: Use Edge Runtime for WebSocket connections
   - Vercel supports WebSockets in Edge Functions
   - Consider Pusher or Ably for complex real-time needs

#### Database Connection Pooling

```typescript
// src/lib/db/serverless.ts
import { neonConfig } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/postgres-js';
import postgres from 'postgres';
import * as schema from './schema';

// Configure for Vercel Edge Runtime
neonConfig.fetchConnectionCache = true;

const connectionString = process.env.DATABASE_URL!;

// For serverless, use connection pooling
const sql = postgres(connectionString, {
  max: 1,
  idle_timeout: 20,
  max_lifetime: 60 * 2,
});

export const db = drizzle(sql, { schema });
```

## Infrastructure Requirements

### Development Environment

- **Local Machine**: 
  - CPU: 2 cores minimum
  - RAM: 4GB minimum
  - Storage: 20GB available
  - Docker Desktop installed

- **Local Services** (via Docker):
  - PostgreSQL 15
  - MinIO for S3-compatible storage

### Production Environment (Vercel + External Services)

#### Vercel Plan Requirements

- **Recommended**: Pro plan or higher
  - Longer function execution times (60s vs 10s)
  - More bandwidth and build minutes
  - Better performance in multiple regions
  - Advanced analytics

#### External Service Requirements

1. **Database (PostgreSQL)**:
   - **Neon** (Recommended for Vercel):
     - Serverless PostgreSQL
     - Automatic scaling
     - Connection pooling built-in
     - 0.5 GB free tier available
   - **Alternative**: Supabase, PlanetScale, or Railway

2. **Object Storage**:
   - **Cloudflare R2** (Recommended):
     - S3-compatible API
     - No egress fees
     - Global distribution
   - **Alternative**: AWS S3, Backblaze B2

3. **Real-time Communication**:
   - **Vercel Edge Functions** for WebSockets
   - **Alternative**: Pusher, Ably, or Supabase Realtime

### Service Configuration Example

```typescript
// lib/config/services.ts
export const servicesConfig = {
  database: {
    // Neon serverless PostgreSQL
    url: process.env.DATABASE_URL,
    // Prisma Data Proxy for connection pooling
    proxyUrl: process.env.DATABASE_PROXY_URL,
  },
  storage: {
    // Cloudflare R2
    endpoint: process.env.R2_ENDPOINT,
    accessKeyId: process.env.R2_ACCESS_KEY_ID,
    secretAccessKey: process.env.R2_SECRET_ACCESS_KEY,
    bucket: process.env.R2_)
## Database Setup

### Vercel Postgres

```bash
# Create database via Vercel CLI
vercel postgres create hristudio-db

# Connect to project
vercel postgres link hristudio-db

# Environment variables are automatically added
```

### External PostgreSQL Providers

#### Neon

```typescript
// Example connection string
DATABASE_URL="postgresql://user:password@xxx.neon.tech/hristudio?sslmode=require"
```

#### Supabase

```typescript
// Example connection string
DATABASE_URL="postgresql://postgres.xxx:password@xxx.supabase.co:5432/postgres"
```

### Database Migrations

```bash
# Run migrations before deployment
bun db:migrate

# Or use GitHub Actions
name: Deploy
on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: oven-sh/setup-bun@v1
      - run: bun install
      - run: bun db:migrate
        env:
          DATABASE_URL: ${{ secrets.DATABASE_URL }}
```

## Storage Configuration

### AWS S3

```typescript
// Environment variables
S3_ENDPOINT=https://s3.amazonaws.com
S3_ACCESS_KEY_ID=your-access-key
S3_SECRET_ACCESS_KEY=your-secret-key
S3_BUCKET_NAME=hristudio-media
S3_REGION=us-east-1
```

### Cloudflare R2 (Recommended for Vercel)

```typescript
// Environment variables
S3_ENDPOINT=https://xxx.r2.cloudflarestorage.com
S3_ACCESS_KEY_ID=your-access-key
S3_SECRET_ACCESS_KEY=your-secret-key
S3_BUCKET_NAME=hristudio-media
S3_REGION=auto
```

### Storage Configuration

Create bucket with CORS policy:

```json
{
  "CORSRules": [
    {
      "AllowedOrigins": ["https://your-app.vercel.app"],
      "AllowedMethods": ["GET", "PUT", "POST", "DELETE"],
      "AllowedHeaders": ["*"],
      "MaxAgeSeconds": 3600
    }
  ]
}
```

## Environment Configuration

### Vercel Environment Variables

Set these in Vercel Dashboard or via CLI:

```bash
# Core Application
NODE_ENV=production
NEXT_PUBLIC_APP_URL=https://your-app.vercel.app

# Database (automatically set if using Vercel Postgres)
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
DATABASE_URL_UNPOOLED=postgresql://user:pass@host/db?sslmode=require

# Authentication
NEXTAUTH_URL=https://your-app.vercel.app
NEXTAUTH_SECRET=<generate-with-openssl-rand-base64-64>

# OAuth Providers (optional)
GOOGLE_CLIENT_ID=xxx
GOOGLE_CLIENT_SECRET=xxx
GITHUB_ID=xxx
GITHUB_SECRET=xxx

# Storage (Cloudflare R2 recommended)
S3_ENDPOINT=https://xxx.r2.cloudflarestorage.com
S3_ACCESS_KEY_ID=xxx
S3_SECRET_ACCESS_KEY=xxx
S3_BUCKET_NAME=hristudio-media
S3_REGION=auto

# Email (SendGrid, Resend, etc.)
SMTP_HOST=smtp.sendgrid.net
SMTP_PORT=587
SMTP_USER=apikey
SMTP_PASS=xxx
SMTP_FROM=noreply@your-domain.com

# ROS2 Integration
NEXT_PUBLIC_ROSBRIDGE_URL=wss://your-ros-bridge.com:9090

# Monitoring (optional)
SENTRY_DSN=https://xxx@sentry.io/xxx
NEXT_PUBLIC_POSTHOG_KEY=xxx
NEXT_PUBLIC_POSTHOG_HOST=https://app.posthog.com

# Feature Flags
NEXT_PUBLIC_ENABLE_OAUTH=true
NEXT_PUBLIC_ENABLE_ANALYTICS=true
```

### Environment Variable Management

```bash
# Add variable for all environments
vercel env add DATABASE_URL

# Add for specific environments
vercel env add STRIPE_KEY production

# Pull to .env.local for development
vercel env pull .env.local

# List all variables
vercel env ls
```

### Security Configuration

```nginx
# nginx/nginx.conf
server {
    listen 443 ssl http2;
    server_name hristudio.example.com;

    ssl_certificate /etc/ssl/certs/hristudio.crt;
    ssl_certificate_key /etc/ssl/private/hristudio.key;
    
    # SSL Configuration
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers ECDHE-RSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384;
    ssl_prefer_server_ciphers off;
    
    # Security Headers
    add_header X-Frame-Options "SAMEORIGIN" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header Referrer-Policy "strict-origin-when-cross-origin" always;
    add_header Content-Security-Policy "default-src 'self'; script-src 'self' 'unsafe-inline' 'unsafe-eval'; style-src 'self' 'unsafe-inline';" always;
    
    # Rate Limiting
    limit_req_zone $binary_remote_addr zone=api:10m rate=10r/s;
    limit_req zone=api burst=20 nodelay;
    
    location / {
        proxy_pass http://app:3000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_cache_bypass $http_upgrade;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

### Continuous Integration/Deployment

Vercel automatically deploys on push to GitHub. For additional CI/CD:

```yaml
# .github/workflows/ci.yml
name: CI/CD Pipeline

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: oven-sh/setup-bun@v1
      
      - name: Install dependencies
        run: bun install
        
      - name: Run type checking
        run: bun run type-check
        
      - name: Run linting
        run: bun run lint
        
      - name: Run tests
        run: bun test
        env:
          DATABASE_URL: ${{ secrets.DATABASE_URL_TEST }}

  database:
    needs: test
    if: github.ref == 'refs/heads/main'
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: oven-sh/setup-bun@v1
      
      - name: Install dependencies
        run: bun install
        
      - name: Run migrations
        run: bun db:migrate
        env:
          DATABASE_URL: ${{ secrets.DATABASE_URL }}

  e2e-tests:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: oven-sh/setup-bun@v1
      
      - name: Install dependencies
        run: bun install
        
      - name: Install Playwright
        run: bunx playwright install --with-deps
        
      - name: Run E2E tests
        run: bun run test:e2e
        env:
          PLAYWRIGHT_TEST_BASE_URL: ${{ secrets.PREVIEW_URL }}
```

### Vercel Deployment Workflow

1. **Automatic Deployments**:
   - Push to `main` → Production deployment
   - Push to other branches → Preview deployment
   - Pull requests → Preview deployment with unique URL

2. **Manual Deployments**:
   ```bash
   # Deploy current branch
   vercel
   
   # Deploy to production
   vercel --prod
   
   # Deploy with specific environment
   vercel --env preview
   ```

3. **Rollback**:
   ```bash
   # List deployments
   vercel ls
   
   # Rollback to previous
   vercel rollback
   
   # Rollback to specific deployment
   vercel rollback [deployment-url]
   ```

### Deployment Checklist

- [ ] Run database migrations
- [ ] Update environment variables
- [ ] Clear application cache
- [ ] Warm up application cache
- [ ] Verify health checks passing
- [ ] Run smoke tests
- [ ] Monitor error rates
- [ ] Update documentation
- [ ] Notify team

### Database Migration Strategy

```bash
#!/bin/bash
# scripts/migrate.sh

set -e

echo "Starting database migration..."

# Backup current database
pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql

# Run migrations
bun db:migrate

# Verify migration
bun db:check

echo "Migration completed successfully"
```

## Monitoring and Observability

### Application Monitoring

```typescript
// src/lib/monitoring/metrics.ts
import { register, Counter, Histogram, Gauge } from 'prom-client';

// Request metrics
// Use Edge-compatible metrics
export const metrics = {
  httpRequestDuration: (method: string, route: string, status: number, duration: number) => {
    // Send to analytics service
    analytics.track('http_request', {
      method,
      route,
      status,
      duration,
    });
  },
  
  trialsStarted: (studyId: string, experimentId: string) => {
    analytics.track('trial_started', {
      studyId,
      experimentId,
    });
  },
  
  activeUsers: (count: number) => {
    analytics.track('active_users', { count });
  },
};

// For Vercel Analytics
export { Analytics } from '@vercel/analytics/react';
export { SpeedInsights } from '@vercel/speed-insights/next';
```

### Logging Configuration

```typescript
// src/lib/monitoring/logger.ts
import winston from 'winston';
import { Logtail } from '@logtail/node';

const logtail = new Logtail(process.env.LOGTAIL_TOKEN);

export const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.json()
  ),
  defaultMeta: {
    service: 'hristudio',
    environment: process.env.NODE_ENV,
  },
  transports: [
    new winston.transports.Console({
      format: winston.format.simple(),
    }),
    new winston.transports.File({
      filename: 'logs/error.log',
      level: 'error',
    }),
    new winston.transports.File({
      filename: 'logs/combined.log',
    }),
    new winston.transports.Http({
      host: 'logs.example.com',
      port: 443,
      path: '/logs',
      ssl: true,
    }),
  ],
});

// Structured logging helpers
export const logEvent = (event: string, metadata?: any) => {
  logger.info('Event occurred', { event, ...metadata });
};

export const logError = (error: Error, context?: any) => {
  logger.error('Error occurred', { 
    error: error.message, 
    stack: error.stack,
    ...context 
  });
};
```

### Health Checks

```typescript
// src/app/api/health/route.ts
import { NextResponse } from 'next/server';
import { db } from '@/lib/db';
import { redis } from '@/lib/redis';
import { checkS3Connection } from '@/lib/storage/s3';

export async function GET() {
  const checks = {
    app: 'ok',
    database: 'checking',
    redis: 'checking',
    storage: 'checking',
  };

  try {
    // Database check
    await db.execute('SELECT 1');
    checks.database = 'ok';
  } catch (error) {
    checks.database = 'error';
  }

  try {
    // Redis check
    await redis.ping();
    checks.redis = 'ok';
  } catch (error) {
    checks.redis = 'error';
  }

  try {
    // S3 check
    await checkS3Connection();
    checks.storage = 'ok';
  } catch (error) {
    checks.storage = 'error';
  }

  const healthy = Object.values(checks).every(status => status === 'ok');
  
  return NextResponse.json(
    {
      status: healthy ? 'healthy' : 'unhealthy',
      timestamp: new Date().toISOString(),
      checks,
    },
    { status: healthy ? 200 : 503 }
  );
}
```

### Monitoring with Vercel

Vercel provides built-in monitoring, but you can enhance with:

1. **Vercel Analytics**: Built-in web analytics
   ```tsx
   // app/layout.tsx
   import { Analytics } from '@vercel/analytics/react';
   import { SpeedInsights } from '@vercel/speed-insights/next';
   
   export default function RootLayout({ children }) {
     return (
       <html>
         <body>
           {children}
           <Analytics />
           <SpeedInsights />
         </body>
       </html>
     );
   }
   ```

2. **Sentry Integration**:
   ```bash
   # Install Sentry
   bun add @sentry/nextjs
   
   # Configure with wizard
   bunx @sentry/wizard@latest -i nextjs
   ```

3. **Custom Monitoring Dashboard**:
   ```typescript
   // app/api/metrics/route.ts
   export async function GET() {
     const metrics = await collectMetrics();
     return NextResponse.json(metrics);
   }
   ```

4. **PostHog for Product Analytics**:
   ```typescript
   // lib/posthog.ts
   import posthog from 'posthog-js';
   
   if (typeof window !== 'undefined') {
     posthog.init(process.env.NEXT_PUBLIC_POSTHOG_KEY!, {
       api_host: process.env.NEXT_PUBLIC_POSTHOG_HOST,
     });
   }
   ```

## Backup and Recovery

### Automated Backup Strategy

```bash
#!/bin/bash
# scripts/backup.sh

set -e

BACKUP_DIR="/backups/$(date +%Y/%m/%d)"
mkdir -p $BACKUP_DIR

# Database backup
echo "Backing up database..."
pg_dump $DATABASE_URL | gzip > "$BACKUP_DIR/db_$(date +%H%M%S).sql.gz"

# Media files backup
echo "Backing up media files..."
aws s3 sync s3://$S3_BUCKET_NAME "$BACKUP_DIR/media/" --exclude "*.tmp"

# Application data backup
echo "Backing up application data..."
tar -czf "$BACKUP_DIR/app_data_$(date +%H%M%S).tar.gz" /app/data

# Upload to backup storage
aws s3 sync $BACKUP_DIR s3://hristudio-backups/$(date +%Y/%m/%d)/

# Clean up old local backups
find /backups -type d -mtime +7 -exec rm -rf {} +

echo "Backup completed successfully"
```

### Disaster Recovery Plan

1. **RTO (Recovery Time Objective)**: 4 hours
2. **RPO (Recovery Point Objective)**: 1 hour

#### Recovery Procedures

```bash
#!/bin/bash
# scripts/restore.sh

set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <backup-date>"
    exit 1
fi

BACKUP_DATE=$1
RESTORE_DIR="/tmp/restore"

# Download backup
echo "Downloading backup from $BACKUP_DATE..."
aws s3 sync s3://hristudio-backups/$BACKUP_DATE/ $RESTORE_DIR/

# Restore database
echo "Restoring database..."
gunzip -c $RESTORE_DIR/db_*.sql.gz | psql $DATABASE_URL

# Restore media files
echo "Restoring media files..."
aws s3 sync $RESTORE_DIR/media/ s3://$S3_BUCKET_NAME

# Restore application data
echo "Restoring application data..."
tar -xzf $RESTORE_DIR/app_data_*.tar.gz -C /

echo "Restore completed successfully"
```

## Scaling with Vercel

### Automatic Scaling

Vercel automatically scales your application:
- **Serverless Functions**: Scale from 0 to thousands of instances
- **Edge Functions**: Run globally at edge locations
- **Static Assets**: Served from global CDN

### Database Connection Pooling

For PostgreSQL with Vercel:

```typescript
// lib/db/index.ts
import { Pool } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-serverless';

// Use connection pooling for serverless
const pool = new Pool({ connectionString: process.env.DATABASE_URL });
export const db = drizzle(pool);
```

### Caching Strategy

Use Vercel's Edge Config and KV Storage instead of Redis:

```typescript
// lib/cache/kv.ts
import { kv } from '@vercel/kv';

export const cache = {
  async get<T>(key: string): Promise<T | null> {
    try {
      return await kv.get<T>(key);
    } catch {
      return null;
    }
  },

  async set<T>(key: string, value: T, ttlSeconds?: number): Promise<void> {
    if (ttlSeconds) {
      await kv.set(key, value, { ex: ttlSeconds });
    } else {
      await kv.set(key, value);
    }
  },

  async invalidate(pattern: string): Promise<void> {
    const keys = await kv.keys(pattern);
    if (keys.length > 0) {
      await kv.del(...keys);
    }
  },
};
```

### Edge Config for Feature Flags

```typescript
// lib/edge-config.ts
import { get } from '@vercel/edge-config';

export async function getFeatureFlag(key: string): Promise<boolean> {
  const value = await get(key);
  return value === true

## Security Operations

### Security Scanning

```yaml
# .github/workflows/security.yml
name: Security Scan

on:
  schedule:
    - cron: '0 0 * * *'
  push:
    branches: [main]

jobs:
  dependency-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run Snyk
        uses: snyk/actions/node@master
        env:
          SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}

  container-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run Trivy
        uses: aquasecurity/trivy-action@master
        with:
          image-ref: 'hristudio/app:latest'
          format: 'sarif'
          output: 'trivy-results.sarif'

  code-scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run CodeQL
        uses: github/codeql-action/analyze@v2
```

### Security Monitoring

```typescript
// src/lib/security/monitor.ts
export const securityMonitor = {
  logFailedLogin(email: string, ip: string) {
    logger.warn('Failed login attempt', { email, ip });
    // Track failed attempts
  },

  logSuspiciousActivity(userId: string, activity: string) {
    logger.warn('Suspicious activity detected', { userId, activity });
    // Alert security team
  },

  checkRateLimit(identifier: string, limit: number): boolean {
    // Implement rate limiting logic
    return true;
  },
};
```

## Maintenance Procedures

### Regular Maintenance Tasks

```bash
#!/bin/bash
# scripts/maintenance.sh

# Weekly tasks
if [ "$(date +%u)" -eq 7 ]; then
    echo "Running weekly maintenance..."
    
    # Vacuum database
    psql $DATABASE_URL -c "VACUUM ANALYZE;"
    
    # Clean up old logs
    find /logs -name "*.log" -mtime +30 -delete
    
    # Update dependencies
    bun update --save
fi

# Daily tasks
echo "Running daily maintenance..."

# Clean up temporary files
find /tmp -name "hristudio-*" -mtime +1 -delete

# Rotate logs
logrotate /etc/logrotate.d/hristudio

# Check disk space
df -h | awk '$5 > 80 {print "Warning: " $6 " is " $5 " full"}'
```

### Update Procedures

1. **Minor Updates**:
   - Test in staging environment
   - Deploy during low-traffic period
   - Monitor for issues

2. **Major Updates**:
   - Schedule maintenance window
   - Notify users in advance
   - Backup all data
   - Test rollback procedure
   - Deploy with gradual rollout

## Troubleshooting

### Common Issues

#### High Memory Usage

```bash
# Check memory usage by process
ps aux --sort=-%mem | head -10

# Check Node.js heap
node --inspect app.js
# Connect Chrome DevTools to inspect heap

# Increase memory limit if needed
NODE_OPTIONS="--max-old-space-size=4096" bun start
```

#### Database Connection Issues

```sql
-- Check active connections
SELECT count(*) FROM pg_stat_activity;

-- Kill idle connections
SELECT pg_terminate_backend(pid) 
FROM pg_stat_activity 
WHERE state = 'idle' 
AND state_change < now() - interval '5 minutes';
```

#### Storage Issues

```bash
# Check disk usage
du -sh /var/lib/docker/*
docker system prune -a

# Check S3 usage
aws s3 ls s3://hristudio-media --recursive --summarize
```

### Debug Mode

```typescript
// Enable debug logging
if (process.env.DEBUG) {
  console.log = (...args) => {
    logger.debug('Console log', { args });
    originalConsoleLog(...args);
  };
}
```

### Performance Profiling

```bash
# CPU profiling
node --cpu-prof app.js

# Memory profiling
node --heap-prof app.js

# Analyze with Chrome DevTools
```

## Disaster Recovery Testing

Schedule regular DR tests:

1. **Monthly**: Restore single service
2. **Quarterly**: Full system restore
3. **Annually**: Complete datacenter failover

Test checklist:
- [ ] Restore database from backup
- [ ] Restore media files
- [ ] Verify data integrity
- [ ] Test all critical functions
- [ ] Measure recovery time
- [ ] Document issues and improvements