#!/bin/bash
set -eu  # Exit immediately if a command exits with a non-zero status

# Default values
BRANCH="develop"

if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker before running this script."
    exit 1
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "\033[1;33m📢 IMPORTANT SETUP INSTRUCTIONS\033[0m"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "⚠️  WARNING: This image contains proprietary software and should not be shared outside of the organization."
echo -e "To clone the \033[1mHoloOcean\033[0m repository, you must:"
echo -e "  1. ✅ Follow the HoloOcean setup instructions to link your GitHub account with \033[1mEpic Games\033[0m."
echo -e "  2. 🔐 Ensure you have an \033[1mSSH key\033[0m configured for GitHub access."
echo "     (HTTPS cloning will not work.)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
# Ask if user has signed EULA
read -p "Have you signed the EULA for this software? [y/n]: " eula_signed
eula_signed=$(echo "$eula_signed" | tr '[:upper:]' '[:lower:]')
if [[ "$eula_signed" != "y" ]]; then
  echo "❌ You must sign the EULA before continuing."
  exit 1
fi

read -p "Do you want to allow docker to access the display? [y/n]: " display_access
display_access=$(echo "$display_access" | tr '[:upper:]' '[:lower:]') 
if [[ "$display_access" == "y" ]]; then
  xhost +local:docker
  echo "WARNING: the command (xhost +local:docker) will need to be run again after every reboot"
fi

# Parse command-line options
while getopts "b:" opt; do
  case ${opt} in
    b )
      BRANCH=$OPTARG
      ;;
    \? )
      echo "Usage: $0 [-b branch_name] [-r]"
      echo "  -b : Specify git branch (default: develop)"
      exit 1
      ;;
  esac
done

# Function to create tmp dir and copy necessary files
create_copy_tmp() {
    echo "🔧 Setting up temporary directory..."
    mkdir tmp
    cd tmp

    echo "📥 Cloning HoloOcean repository..."
    git clone git@github.com:byu-holoocean/HoloOcean.git 
    cd HoloOcean

    echo "📂 Checking out branch: $BRANCH"
    git checkout "$BRANCH"
}

# Check for existing tmp directory
if [ -d "tmp" ]; then
  echo "📁 'tmp/' directory already exists."
  read -p "Do you want to skip cloning and copying ROS packages? [y/n]: " skip_tmp
  skip_tmp=$(echo "$skip_tmp" | tr '[:upper:]' '[:lower:]')
  if [[ "$skip_tmp" == "y" ]]; then
    echo "⚠️  Skipping setup. Will only build the Docker image with existing tmp directory."
  else
    echo "🧹 Removing old tmp directory..."
    rm -rf tmp/
    create_copy_tmp
  fi
else
  create_copy_tmp
fi

echo "🐳 Building Docker image..."
docker compose build

# Ask if user wants to clean up
echo "🧹 Cleaning up temporary files..."
rm -rf tmp/

# Optional: Ask if user wants to clean up
read -p "Would you like to start the container in the background? [y/n]: " startup
startup=$(echo "$startup" | tr '[:upper:]' '[:lower:]')
if [[ "$startup" == "y" ]]; then
  echo "🐳 Starting up container"
  docker compose up -d
fi

echo "✅ Done!"
