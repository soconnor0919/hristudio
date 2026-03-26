# Tutorial 1: Getting Started

Learn how to set up HRIStudio and log in for the first time.

## Objectives

- Install HRIStudio dependencies
- Start the development environment
- Log in and explore the interface

## Prerequisites

- [Bun](https://bun.sh) installed
- [Docker](https://docker.com) installed
- [Git](https://git-scm.com) installed

## Step 1: Clone the Repository

```bash
git clone https://github.com/soconnor0919/hristudio.git
cd hristudio
```

## Step 2: Install Dependencies

HRIStudio uses Bun as its package manager:

```bash
bun install
```

## Step 3: Start the Database

HRIStudio requires PostgreSQL. The easiest way is using Docker:

```bash
# Start PostgreSQL and MinIO (for file storage)
bun run docker:up

# Push database schema
bun db:push

# Seed with sample data
bun db:seed
```

This creates the database schema and populates it with:
- 4 default user accounts
- Sample study and experiments
- Test participants and trials

## Step 4: Start the Development Server

```bash
bun dev
```

The application will be available at `http://localhost:3000`.

## Step 5: Log In

Use one of the default accounts:

| Role | Email | Password |
|------|-------|----------|
| Administrator | `sean@soconnor.dev` | `password123` |
| Researcher | `felipe.perrone@bucknell.edu` | `password123` |
| Wizard | `emily.watson@lab.edu` | `password123` |
| Observer | `maria.santos@tech.edu` | `password123` |

## Exploring the Interface

After logging in, you'll see the main dashboard:

```
┌─────────────────────────────────────────────────────────────┐
│  HRIStudio                              [User] [Settings]   │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐          │
│  │ Studies │ │  Trials │ │Plugins  │ │  Admin  │          │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘          │
│                                                             │
│  Recent Activity                                            │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ • Study: Comparative WoZ Study - Ready              │   │
│  │ • Trial: P101 - Completed (5 min ago)                │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Navigation

- **Studies** - View and manage your research studies
- **Trials** - Monitor and manage experiment trials
- **Plugins** - Manage robot integrations
- **Admin** - System administration (admins only)

## Using Simulation Mode

If you don't have a physical robot, enable simulation mode:

1. Create or edit `hristudio/.env.local`
2. Add: `NEXT_PUBLIC_SIMULATION_MODE=true`
3. Restart the dev server

Simulation mode allows you to test experiments without connecting to a real robot.

## Troubleshooting

### Database Connection Failed

```bash
# Check if Docker is running
docker ps

# Restart the database
bun run docker:down
bun run docker:up
bun db:push
```

### Port Already in Use

If port 3000 is in use:

```bash
# Use a different port
PORT=3001 bun dev
```

### Seed Script Fails

```bash
# Reset the database
bun run docker:down -v
bun run docker:up
bun db:push
bun db:seed
```

## Next Steps

Now that you're set up:

1. **[Your First Study](02-your-first-study.md)** - Create a research study
2. **[Designing Experiments](03-designing-experiments.md)** - Build your first protocol
3. **[Simulation Mode](09-simulation-mode.md)** - Test without a robot

---

**Previous**: [Tutorials Overview](../tutorials/README.md) | **Next**: [Your First Study](02-your-first-study.md)
