#! /usr/bin/env bash

set -e

cd ~
git clone https://ptss37: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@github.com/jebiio/JebiFirmware.git
cd ~/JebiFirmware
git checkout kari
git submodule update --init --recursive
git submodule update --remote --merge "Tools/sitl_gazebo"
make px4_sitl gazebo << 'EOF'
y
y
EOF
