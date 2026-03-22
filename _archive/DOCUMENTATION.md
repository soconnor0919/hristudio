# HRIStudio Documentation Overview

Clean, organized documentation for the HRIStudio platform.

## Quick Links

### Getting Started
- **[README.md](README.md)** - Main project overview and setup
- **[Quick Reference](docs/quick-reference.md)** - 5-minute setup guide

### HRIStudio Platform
- **[Project Overview](docs/project-overview.md)** - Features and architecture
- **[Database Schema](docs/database-schema.md)** - Complete database reference
- **[API Routes](docs/api-routes.md)** - tRPC API documentation
- **[Implementation Guide](docs/implementation-guide.md)** - Technical implementation

### NAO6 Robot Integration
- **[NAO6 Quick Reference](docs/nao6-quick-reference.md)** - Essential commands
- **[Integration Repository](../nao6-hristudio-integration/)** - Complete integration package
  - Installation guide
  - Usage instructions  
  - Troubleshooting
  - Plugin definitions

### Experiment Design
- **[Core Blocks System](docs/core-blocks-system.md)** - Experiment building blocks
- **[Plugin System](docs/plugin-system-implementation-guide.md)** - Robot plugins

### Deployment
- **[Deployment & Operations](docs/deployment-operations.md)** - Production deployment

### Research
- **[Research Paper](docs/paper.md)** - Academic documentation

## Repository Structure

```
hristudio/                          # Main web application
├── README.md                       # Start here
├── DOCUMENTATION.md                # This file
├── src/                            # Next.js application
├── docs/                           # Platform documentation
└── ...

nao6-hristudio-integration/         # NAO6 integration
├── README.md                       # Integration overview
├── docs/                           # NAO6 documentation
├── launch/                         # ROS2 launch files
├── scripts/                        # Utility scripts
├── plugins/                        # Plugin definitions
└── examples/                       # Usage examples
```

## Documentation Philosophy

- **One source of truth** - No duplicate docs
- **Clear hierarchy** - Easy to find what you need
- **Practical focus** - Real commands, not theory
- **Examples** - Working code samples

## For Researchers

Start here:
1. [README.md](README.md) - Setup HRIStudio
2. [NAO6 Quick Reference](docs/nao6-quick-reference.md) - Connect NAO robot
3. [Project Overview](docs/project-overview.md) - Understand the system

## For Developers

Start here:
1. [Implementation Guide](docs/implementation-guide.md) - Technical architecture
2. [Database Schema](docs/database-schema.md) - Data model
3. [API Routes](docs/api-routes.md) - Backend APIs

## Support

- Check documentation first
- Use NAO6 integration repo for robot-specific issues
- Main HRIStudio repo for platform issues

---

**Last Updated:** December 2024  
**Version:** 1.0 (Simplified)
