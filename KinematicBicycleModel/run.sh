#!/bin/bash

# Exit immediately on any error
set -e

echo "🔧 Building the project..."
make

echo "🚗 Running simulation..."
./build/a.out

echo "📈 Plotting results..."
python3 script/plot.py

echo "✅ All done!"
