#!/bin/bash
set -e

# Determine version from git tags, fallback to 0.0.1
VERSION=$(git describe --tags 2>/dev/null || echo "0.0.1")
VERSION=$(echo "$VERSION" | sed 's/^v//')

# Generate configure.ac from template
if [ -f configure.ac.in ]; then
    sed "s/PACKAGE_VERSION/${VERSION}/g" configure.ac.in > configure.ac
else
    echo "ERROR: configure.ac.in not found" >&2
    exit 1
fi

autoreconf -if
