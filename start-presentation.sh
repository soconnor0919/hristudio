#!/bin/bash

# HRIStudio Presentation Quick Start Script

echo "🎯 A Web-Based Wizard-of-Oz Platform for Collaborative and Reproducible Human-Robot Interaction Research"
echo "======================================================================================================"
echo ""

# Check if Marp is installed
if ! command -v marp &> /dev/null; then
    echo "⚠️  Marp CLI not found. Installing..."
    if command -v bun &> /dev/null; then
        bun install
    elif command -v npm &> /dev/null; then
        npm install
    else
        echo "❌ No package manager found. Please install Node.js/npm or Bun."
        exit 1
    fi
fi

echo "📝 Available commands:"
echo ""
echo "1. Live Preview (recommended for editing)"
echo "   bun run preview OR npm run preview"
echo ""
echo "2. Watch mode (auto-reload on changes)"  
echo "   bun run watch OR npm run watch"
echo ""
echo "3. Build PDF for conference"
echo "   bun run build:pdf OR npm run build:pdf"
echo ""
echo "4. Build all formats (PDF, HTML, PPTX)"
echo "   bun run build:all OR npm run build:all"
echo ""
echo "5. Start server (for remote access)"
echo "   bun run serve OR npm run serve"
echo ""

# Parse command line argument
case "$1" in
    "preview"|"p")
        echo "🔄 Starting live preview..."
        if command -v bun &> /dev/null; then
            bun run preview
        else
            npm run preview
        fi
        ;;
    "watch"|"w")  
        echo "👀 Starting watch mode..."
        if command -v bun &> /dev/null; then
            bun run watch
        else
            npm run watch
        fi
        ;;
    "pdf")
        echo "📄 Building PDF..."
        if command -v bun &> /dev/null; then
            bun run build:pdf
        else
            npm run build:pdf
        fi
        echo "✅ PDF generated: hristudio-presentation.pdf"
        ;;
    "all"|"build")
        echo "🏗️  Building all formats..."
        if command -v bun &> /dev/null; then
            bun run build:all
        else
            npm run build:all
        fi
        echo "✅ All formats generated"
        ;;
    "serve"|"s")
        echo "🌐 Starting server..."
        if command -v bun &> /dev/null; then
            bun run serve
        else
            npm run serve
        fi
        ;;
    *)
        echo "Usage: $0 [preview|watch|pdf|all|serve]"
        echo "Or run without arguments to see this menu."
        ;;
esac