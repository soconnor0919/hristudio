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

5. **[Core Blocks System](./core-blocks-system.md)**
   - Repository-based plugin architecture
   - 26 essential blocks across 4 categories
   - Event triggers, wizard actions, control flow, observation
   - Block loading and validation system
   - Integration with experiment designer

6. **[Plugin System Implementation](./plugin-system-implementation-guide.md)**
   - Robot plugin architecture and development
   - Repository management and trust levels
   - Plugin installation and configuration
   - Action definitions and parameter schemas
   - ROS2 integration patterns

7. **[Implementation Guide](./implementation-guide.md)**
   - Step-by-step technical implementation
   - Code examples and patterns
   - Frontend and backend architecture
   - Real-time features implementation
   - Testing strategies

8. **[Implementation Details](./implementation-details.md)**
   - Architecture decisions and rationale
   - Unified editor experiences (significant code reduction)
   - DataTable migration achievements
   - Development database and seed system
   - Performance optimization strategies

#### **Operations & Deployment**
9. **[Deployment & Operations](./deployment-operations.md)**
   - Infrastructure requirements
   - Vercel deployment strategies
   - Monitoring and observability
   - Backup and recovery procedures
   - Security operations

10. **[ROS2 Integration](./ros2-integration.md)**
    - rosbridge WebSocket architecture
    - Client-side ROS connection management
    - Message type definitions
    - Robot plugin implementation
    - Security considerations for robot communication

### **📊 Project Status**

11. **[Project Status](./project-status.md)**
    - Overall completion status (complete)
    - Implementation progress by feature
    - Sprint planning and development velocity
    - Production readiness assessment
    - Core blocks system completion

12. **[Quick Reference](./quick-reference.md)**
    - 5-minute setup guide
    - Essential commands and patterns
    - API reference and common workflows
    - Core blocks system overview
    - Key concepts and architecture overview

13. **[Work in Progress](./work_in_progress.md)**
    - Recent changes and improvements
    - Core blocks system implementation
    - Plugin architecture enhancements
    - Panel-based wizard interface (matching experiment designer)
    - Technical debt resolution
    - UI/UX enhancements

### **🤖 Robot Integration Guides**

14. **[NAO6 Complete Integration Guide](./nao6-integration-complete-guide.md)** - Comprehensive NAO6 setup, troubleshooting, and production deployment
15. **[NAO6 Quick Reference](./nao6-quick-reference.md)** - Essential commands and troubleshooting for NAO6 integration
16. **[NAO6 ROS2 Setup](./nao6-ros2-setup.md)** - Basic NAO6 ROS2 driver installation guide

### **📖 Academic References**

17. **[Research Paper](./root.tex)** - Academic LaTeX document
18. **[Bibliography](./refs.bib)** - Research references

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
3. **[NAO6 Quick Reference](./nao6-quick-reference.md)** - Essential NAO6 robot control commands
4. **[ROS2 Integration](./ros2-integration.md)** - Robot platform integration
5. **[Research Paper](./root.tex)** - Academic context and methodology

### **For Robot Integration**
1. **[NAO6 Complete Integration Guide](./nao6-integration-complete-guide.md)** - Full NAO6 setup and troubleshooting
2. **[NAO6 Quick Reference](./nao6-quick-reference.md)** - Essential commands and quick fixes
3. **[ROS2 Integration](./ros2-integration.md)** - General robot integration patterns

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
- **Visual Experiment Designer**: Repository-based plugin architecture with 26 core blocks
- **Core Block Categories**: Events, wizard actions, control flow, observation blocks
- **Real-time Trial Execution**: Live wizard control with data capture
- **Multi-role Collaboration**: Administrator, Researcher, Wizard, Observer
- **Comprehensive Data Management**: Synchronized multi-modal capture

### **Technical Excellence**
- **Full Type Safety**: End-to-end TypeScript with strict mode
- **Production Ready**: Vercel deployment with Edge Runtime
- **Performance Optimized**: Database indexes and query optimization
- **Security First**: Role-based access control throughout
- **Modern Stack**: Next.js 15, tRPC, Drizzle ORM, shadcn/ui
- **Consistent Architecture**: Panel-based interfaces across visual programming tools

### **Development Experience**
- **Unified Components**: Significant reduction in code duplication
- **Panel Architecture**: 90% code sharing between experiment designer and wizard interface
- **Consolidated Wizard**: 3-panel design with trial controls, horizontal timeline, and unified robot controls
- **Enterprise DataTables**: Advanced filtering, export, pagination
- **Comprehensive Testing**: Realistic seed data with complete scenarios
- **Developer Friendly**: Clear patterns and extensive documentation

### **Robot Integration**
- **NAO6 Full Support**: Complete ROS2 integration with movement, speech, and sensor control
- **Real-time Control**: WebSocket-based robot control through web interface
- **Safety Features**: Emergency stops, movement limits, and comprehensive monitoring
- **Production Ready**: Tested with NAO V6.0 / NAOqi 2.8.7.4 / ROS2 Humble
- **Troubleshooting Guides**: Complete documentation for setup and problem resolution

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
- ✅ **Visual Designer** - Repository-based plugin architecture
- ✅ **Consolidated Wizard Interface** - 3-panel design with horizontal timeline and unified robot controls
- ✅ **Core Blocks System** - 26 blocks across events, wizard, control, observation
- ✅ **Plugin Architecture** - Unified system for core blocks and robot actions
- ✅ **Development Environment** - Realistic test data and scenarios
- ✅ **NAO6 Robot Integration** - Full ROS2 integration with comprehensive control and monitoring
- ✅ **Intelligent Control Flow** - Loops with implicit approval, branching, parallel execution

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

**All success criteria have been met. HRIStudio is ready for production deployment with full NAO6 robot integration support.**

---

## 📝 **Documentation Maintenance**

- **Version**: 2.0.0 (Streamlined)
- **Last Updated**: December 2024
- **Target Platform**: HRIStudio v1.0
- **Structure**: Consolidated for clarity and maintainability

This documentation represents a complete, streamlined specification for building and deploying HRIStudio. Every technical decision has been carefully considered to create a robust, scalable platform for HRI research.