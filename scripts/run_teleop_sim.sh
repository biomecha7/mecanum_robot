#!/bin/bash
# Pi Teleop Simulator Runner
# This script installs dependencies and runs the teleop simulator

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/../.venv"

echo "🤖 Pi Teleop Simulator Setup & Runner"
echo "======================================"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "📦 Creating Python virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# Activate virtual environment
echo "🔧 Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Install/upgrade pip and dependencies
echo "📥 Installing dependencies..."
pip install --upgrade pip
pip install -r "$SCRIPT_DIR/requirements.txt"

# Run the simulator
echo "🚀 Starting Pi Teleop Simulator..."
echo ""

# Check if any arguments were passed
if [ $# -eq 0 ]; then
    # No arguments, run in basic mode
    python3 "$SCRIPT_DIR/simulate_pi_teleop.py"
else
    # Pass all arguments to the script
    python3 "$SCRIPT_DIR/simulate_pi_teleop.py" "$@"
fi

echo ""
echo "✅ Simulator finished"
