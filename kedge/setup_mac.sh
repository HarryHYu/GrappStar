#!/bin/bash
# one-time setup on a Mac (Homebrew assumed)
set -e
cd "$(dirname "$0")"
brew install node graphviz arduino-cli || true
python3.13 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m playwright install chromium
npm install playwright@1.63 && npx playwright install chromium
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "Adafruit INA219" "Adafruit BNO08x" "HX711"
echo "test: cd fw/test && g++ -std=c++17 -O1 -I../kedge_dart -o test_logic test_logic.cpp && ./test_logic"
echo "pipeline: source .venv/bin/activate && ./run_all.sh out29 v29"
