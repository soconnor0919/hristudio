# HRIStudio

HRIStudio is a comprehensive web-based platform for managing Wizard of Oz (WoZ) studies in Human-Robot Interaction research. It provides researchers with standardized tools for designing experiments, executing trials, and analyzing data while ensuring reproducibility and scientific rigor.

## Features

- **Visual Experiment Designer**: Drag-and-drop interface for creating complex interaction scenarios
- **Real-time Trial Control**: Live robot control with responsive wizard interface during experiments
- **Hierarchical Study Structure**: Organized workflow from Study → Experiment → Trial → Step → Action
- **Multi-modal Data Capture**: Synchronized recording of video, audio, logs, and sensor data
- **Role-based Access Control**: Four distinct roles (Administrator, Researcher, Wizard, Observer)
- **Robot Platform Integration**: Support for multiple robot platforms via RESTful APIs, ROS, and custom plugins
- **Collaborative Research**: Team management with secure data sharing and role-based permissions

## Tech Stack

- **Framework**: [Next.js 15](https://nextjs.org) with App Router
- **Authentication**: [NextAuth.js v5](https://next-auth.js.org)
- **Database**: [PostgreSQL](https://postgresql.org) with [Drizzle ORM](https://orm.drizzle.team)
- **Storage**: [MinIO](https://min.io) (S3-compatible) for media files
- **API**: [tRPC](https://trpc.io) for type-safe client-server communication
- **UI**: [Tailwind CSS](https://tailwindcss.com) with [shadcn/ui](https://ui.shadcn.com) and [Radix UI](https://radix-ui.com)
- **Package Manager**: [Bun](https://bun.sh) (exclusively)

## Getting Started

### Prerequisites

- [Bun](https://bun.sh) (latest version)
- [PostgreSQL](https://postgresql.org) (14+)
- [Docker](https://docker.com) (optional, for containerized deployment)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/your-org/hristudio.git
cd hristudio
```

2. Install dependencies:
```bash
bun install
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your database credentials and other settings
```

4. Start the database (using Docker):
```bash
bun run docker:up
```

5. Push the database schema:
```bash
bun run db:push
```

6. Start the development server:
```bash
bun dev
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

## Development Commands

| Command | Description |
|---------|-------------|
| `bun dev` | Start development server |
| `bun build` | Build for production |
| `bun start` | Start production server |
| `bun typecheck` | Run TypeScript checks |
| `bun lint` | Run ESLint |
| `bun lint --fix` | Fix ESLint issues |
| `bun test` | Run tests |
| `bun db:generate` | Generate database schema |
| `bun db:migrate` | Run database migrations |
| `bun db:push` | Push schema changes |
| `bun db:studio` | Open database studio |

## Project Structure

```
src/
├── app/                      # Next.js app router
│   ├── (auth)/              # Authentication pages
│   ├── (dashboard)/         # Main application pages
│   ├── api/                 # API routes
│   └── layout.tsx
├── components/              # Shared UI components
│   ├── ui/                  # shadcn/ui components
│   ├── dashboard/           # Dashboard-specific components
│   ├── experiments/         # Experiment-related components
│   └── studies/             # Study management components
├── lib/                     # Utilities and configurations
│   ├── db/                  # Database setup and schemas
│   ├── trpc/                # tRPC setup and routers
│   └── auth/                # NextAuth configuration
└── types/                   # Shared TypeScript types
```

## Architecture

HRIStudio follows a three-layer architecture:

1. **User Interface Layer**: Browser-based interfaces for experiment design, wizard control, and data analysis
2. **Data Management Layer**: PostgreSQL database with role-based access control and MinIO for media storage
3. **Robot Integration Layer**: Platform-agnostic communication supporting RESTful APIs, ROS, and custom plugins

## Key Concepts

### Hierarchical Study Structure

- **Study**: Top-level container for a research project
- **Experiment**: Parameterized template specifying experimental protocol
- **Trial**: Executable instance with specific participant and conditions
- **Step**: Distinct phase containing wizard or robot instructions
- **Action**: Specific atomic task (speech, movement, input gathering, etc.)

### User Roles

- **Administrator**: Full system access and user management
- **Researcher**: Study creation, experiment design, data analysis
- **Wizard**: Trial execution and robot control
- **Observer**: Read-only access to trials and data

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes following the [project guidelines](./.rules)
4. Run tests: `bun test`
5. Run type checking: `bun typecheck`
6. Run linting: `bun lint`
7. Commit your changes: `git commit -m 'Add feature'`
8. Push to the branch: `git push origin feature-name`
9. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For questions, issues, or contributions:

- Create an [issue](https://github.com/your-org/hristudio/issues)
- Check the [documentation](./docs/)
- Review the [project rules](./.rules)

## Acknowledgments

HRIStudio was developed to advance Human-Robot Interaction research by providing standardized, reproducible methodologies for Wizard of Oz studies.