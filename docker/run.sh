#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="${1:-hsv}"

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  echo "Usage: $0 [hsv|hsv_tuner] [docker-compose options...]"
  echo ""
  echo "  Modes:"
  echo "    hsv           - HSV segmentation, no GUI (default)"
  echo "    hsv_tuner     - HSV segmentation with tuner GUI"
  echo ""
  echo "  Examples:"
  echo "    $0                            # HSV segmentation"
  echo "    $0 hsv_tuner                  # HSV with tuner GUI"
  echo "    $0 hsv -d                     # Detached mode"
  echo ""
  echo "  Environment variables:"
  echo "    ROS_DISTRO=jazzy $0           # Use Jazzy"
  exit 0
fi

COMPOSE_FILES="-f docker-compose.yml"

# Add NVIDIA GPU support if available
if command -v nvidia-smi &> /dev/null; then
  COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.nvidia.yml"
  echo "=== NVIDIA GPU detected ==="
fi

echo "=== Starting core (segmentation=${MODE}) ==="
SEGMENTATION="$MODE" docker compose $COMPOSE_FILES up trackdlo-core "${@:2}"
