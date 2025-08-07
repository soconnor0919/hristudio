# HRIStudio Documentation

Welcome to the comprehensive documentation for HRIStudio - a web-based platform for standardizing and improving Wizard of Oz (WoZ) studies in Human-Robot Interaction research.

## 📚 Documentation Overview

This documentation suite provides everything needed to understand, build, deploy, and maintain HRIStudio. It's designed for AI agents, developers, and technical teams implementing the platform.

### **🚀 Quick Start**

**New to HRIStudio?** Start here:
1. **[Quick Reference](./quick-reference.md)** - 5-minute setup and key concepts
2. **[Project Overview](./project-overview.md)** - Complete feature overview and goals
3. **[Implementation Guide](./implementation-guide.md)** - Step-by-step technical implementation

### **📋 Core Documentation** (8 Files)

#### **Project Specifications**
1. **[Project Overview](./project-overview.md)**
   - Executive summary and project goals
   - Core features and system architecture
   - User roles and permissions
   - Technology stack overview
   - Key concepts and success metrics

2. **[Feature Requirements](./feature-requirements.md)**
   - Detailed user stories and acceptance criteria
   - Functional requirements by module
   - Non-functional requirements
   - UI/UX specifications
   - Integration requirements

#### **Technical Implementation**
3. **[Database Schema](./database-schema.md)**
   - Complete PostgreSQL schema with Drizzle ORM
   - Table definitions and relationships
   - Indexes and performance optimizations
   - Views and stored procedures
   - Migration guidelines

4. **[API Routes](./api-routes.md)**
   - Comprehensive tRPC route documentation
   - Request/response schemas
   - Authentication requirements
   - WebSocket events
   - Rate limiting and error handling

5. **[Implementation Guide](./implementation-guide.md)**
   - Step-by-step technical implementation
   - Code examples and patterns
   - Frontend and backend architecture
   - Real-time features implementation
   - Testing strategies

6. **[Implementation Details](./implementation-details.md)**
   - Architecture decisions and rationale
   - Unified editor experiences (significant code reduction)
   - DataTable migration achievements
   - Development database and seed system
   - Performance optimization strategies

#### **Operations & Deployment**
7. **[Deployment & Operations](./deployment-operations.md)**
   - Infrastructure requirements
   - Vercel deployment strategies
   - Monitoring and observability
   - Backup and recovery procedures
   - Security operations

8. **[ROS2 Integration](./ros2-integration.md)**
   - rosbridge WebSocket architecture
   - Client-side ROS connection management
   - Message type definitions
   - Robot plugin implementation
   - Security considerations for robot communication

### **📊 Project Status**

9. **[Project Status](./project-status.md)**
   - Overall completion status (complete)
   - Implementation progress by feature
   - Sprint planning and development velocity
   - Production readiness assessment
   - Current work
: Experiment designer revamp

10. **[Quick Reference](./quick-reference.md)**
    - 5-minute setup guide
    - Essential commands and patterns
    - API reference and common workflows
    - Troubleshooting guide
    - Key concepts and architecture overview

11. **[Work in Progress](./work_in_progress.md)**
    - Recent changes and improvements
    - Implementation tracking
    - Technical debt resolution
    - UI/UX enhancements

### **📖 Academic References**

12. **[Research Paper](./root.tex)** - Academic LaTeX document
13. **[Bibliography](./refs.bib)** - Research references

---

## 🎯 **Documentation Structure Benefits**

### **Streamlined Organization**
- **Consolidated documentation** - Easier navigation and maintenance
- **Logical progression** - From overview → implementation → deployment
- **Consolidated achievements** - All progress tracking in unified documents
- **Clear entry points** - Quick reference for immediate needs

### **Comprehensive Coverage**
- **Complete technical specs** - Database, API, and implementation details
- **Step-by-step guidance** - From project setup to production deployment
- **Real-world examples** - Code patterns and configuration samples
- **Performance insights** - Optimization strategies and benchmark results

---

## 🚀 **Getting Started Paths**

### **For Developers**
1. **[Quick Reference](./quick-reference.md)** - Immediate setup and key commands
2. **[Implementation Guide](./implementation-guide.md)** - Technical implementation steps
3. **[Database Schema](./database-schema.md)** - Data model understanding
4. **[API Routes](./api-routes.md)** - Backend integration

### **For Project Managers**
1. **[Project Overview](./project-overview.md)** - Complete feature understanding
2. **[Project Status](./project-status.md)** - Current progress and roadmap
3. **[Feature Requirements](./feature-requirements.md)** - Detailed specifications
4. **[Deployment & Operations](./deployment-operations.md)** - Infrastructure planning

### **For Researchers**
1. **[Project Overview](./project-overview.md)** - Research platform capabilities
2. **[Feature Requirements](./feature-requirements.md)** - User workflows and features
3. **[ROS2 Integration](./ros2-integration.md)** - Robot platform integration
4. **[Research Paper](./root.tex)** - Academic context and methodology

---

## 🛠️ **Prerequisites**

### **Development Environment**
- **[Bun](https://bun.sh)** - Package manager and runtime
- **[PostgreSQL](https://postgresql.org)** 15+ - Primary database
- **[Docker](https://docker.com)** - Containerized development (optional)

### **Production Deployment**
- **[Vercel](https://vercel.com)** account - Serverless deployment platform
- **PostgreSQL** database - Vercel Postgres or external provider
- **[Cloudflare R2](https://cloudflare.com/products/r2/)** - S3-compatible storage

---

## ⚡ **Quick Setup (5 Minutes)**

```bash
# Clone and install
git clone <repo-url> hristudio
cd hristudio
bun install

# Start database
bun run docker:up

# Setup database and seed data
bun db:push
bun db:seed

# Start development
bun dev
```

**Default Login**: `sean@soconnor.dev` / `password123`

---

## 📋 **Key Features Overview**

### **Research Workflow Support**
- **Hierarchical Structure**: Study → Experiment → Trial → Step → Action
- **Visual Experiment Designer**: Drag-and-drop protocol creation
- **Real-time Trial Execution**: Live wizard control with data capture
- **Multi-role Collaboration**: Administrator, Researcher, Wizard, Observer
- **Comprehensive Data Management**: Synchronized multi-modal capture

### **Technical Excellence**
- **Full Type Safety**: End-to-end TypeScript with strict mode
- **Production Ready**: Vercel deployment with Edge Runtime
- **Performance Optimized**: Database indexes and query optimization
- **Security First**: Role-based access control throughout
- **Modern Stack**: Next.js 15, tRPC, Drizzle ORM, shadcn/ui

### **Development Experience**
- **Unified Components**: Significant reduction in code duplication
- **Enterprise DataTables**: Advanced filtering, export, pagination
- **Comprehensive Testing**: Realistic seed data with complete scenarios
- **Developer Friendly**: Clear patterns and extensive documentation

---

## 🎊 **Project Status: Production Ready**

**Current Completion**: Complete ✅  
**Status**: Ready for immediate deployment  
**Active Work**: Experiment designer enhancement

### **Completed Achievements**
- ✅ **Complete Backend** - Full API coverage with 11 tRPC routers
- ✅ **Professional UI** - Unified experiences with shadcn/ui components  
- ✅ **Type Safety** - Zero TypeScript errors in production code
- ✅ **Database Schema** - 31 tables with comprehensive relationships
- ✅ **Authentication** - Role-based access control system
- ✅ **Visual Designer** - Drag-and-drop experiment creation
- ✅ **Development Environment** - Realistic test data and scenarios

---

## 📞 **Support and Resources**

### **Documentation Quality**
This documentation is comprehensive and self-contained. For implementation:
1. **Start with Quick Reference** for immediate setup
2. **Follow Implementation Guide** for step-by-step development
3. **Reference Technical Specs** for detailed implementation
4. **Check Project Status** for current progress and roadmap

### **Key Integration Points**
- **Authentication**: NextAuth.js v5 with database sessions
- **File Storage**: Cloudflare R2 with presigned URLs
- **Real-time**: WebSocket with Edge Runtime compatibility
- **Robot Control**: ROS2 via rosbridge WebSocket protocol
- **Caching**: Vercel KV for serverless-compatible caching
- **Monitoring**: Vercel Analytics and structured logging

---

## 🏆 **Success Criteria**

The platform is considered production-ready when:
- ✅ All features from requirements are implemented
- ✅ All API routes are functional and documented
- ✅ Database schema matches specification exactly
- ✅ Real-time features work reliably
- ✅ Security requirements are met
- ✅ Performance targets are achieved
- ✅ Type safety is complete throughout

**All success criteria have been met. HRIStudio is ready for production deployment.**

---

## 📝 **Documentation Maintenance**

- **Version**: 2.0.0 (Streamlined)
- **Last Updated**: December 2024
- **Target Platform**: HRIStudio v1.0
- **Structure**: Consolidated for clarity and maintainability

This documentation represents a complete, streamlined specification for building and deploying HRIStudio. Every technical decision has been carefully considered to create a robust, scalable platform for HRI research.