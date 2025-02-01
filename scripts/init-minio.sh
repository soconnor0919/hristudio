#!/bin/bash

# Install MinIO client if not already installed
if ! command -v mc &> /dev/null; then
    echo "Installing MinIO client..."
    curl -O https://dl.min.io/client/mc/release/darwin-amd64/mc
    chmod +x mc
    sudo mv mc /usr/local/bin/
fi

# Configure MinIO client
mc alias set local http://localhost:9000 minioadmin minioadmin

# Create bucket if it doesn't exist
if ! mc ls local/uploads &> /dev/null; then
    echo "Creating uploads bucket..."
    mc mb local/uploads
fi

# Set bucket policy to public
echo "Setting bucket policy..."
mc policy set public local/uploads

echo "MinIO initialization complete!" 