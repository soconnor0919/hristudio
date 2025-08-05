# A Web-Based Wizard-of-Oz Platform for Collaborative and Reproducible Human-Robot Interaction Research

A markdown-based conference presentation using Marp with a custom mono/minimal theme, featuring ASCII diagrams and professional formatting.

## Quick Start

```bash
# Install dependencies
bun install
# OR
npm install

# Start live preview (recommended for editing)
./start-presentation.sh preview

# Build PDF for conference submission
./start-presentation.sh pdf
```

## Available Scripts

- `bun run preview` - Live preview with auto-reload
- `bun run watch` - Watch mode with browser preview
- `bun run build:pdf` - Generate PDF for conference
- `bun run build:html` - Generate HTML version
- `bun run build:pptx` - Generate PowerPoint format
- `bun run build:all` - Build all formats
- `bun run serve` - Start development server

## Files

- `hristudio-presentation.md` - Main presentation content
- `start-presentation.sh` - Quick start helper script
- `package.json` - Dependencies and scripts

## Design System

The presentation uses a mono/minimal aesthetic with:

- **Font**: JetBrains Mono (Geist Mono fallback)
- **Colors**: Matching HRIStudio's oklch color scheme
- **Layout**: Clean, spacious design with no unnecessary elements
- **Typography**: Consistent hierarchy with proper spacing

## Development Workflow

1. **Edit**: Modify `hristudio-presentation.md` in your editor
2. **Preview**: Run `./start-presentation.sh preview` for live reload
3. **Build**: Generate final PDF with `./start-presentation.sh pdf`
4. **Version Control**: All files are text-based and git-friendly

## VS Code Integration

Install "Marp for VS Code" extension for:
- Live preview in editor
- Syntax highlighting
- Immediate feedback while editing

## Conference Requirements

The generated PDF meets standard conference requirements:
- 16:9 aspect ratio for projectors
- High-quality embedded fonts
- Professional typography
- Consistent with academic presentation standards

## Customization

- Edit content in `hristudio-presentation.md`
- Modify colors/fonts in the `<style>` section
- Add new slides by inserting `---` separators
- Use custom CSS classes for special formatting

This setup provides a developer-friendly, version-controlled presentation workflow that maintains design consistency with the HRIStudio platform.