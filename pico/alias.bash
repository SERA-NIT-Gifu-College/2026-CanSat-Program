#!/usr/bin/env bash

alias upload="sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c 'adapter speed 5000' -c 'program build/cansat2026.elf verify reset exit'"
alias debug="sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c 'adapter speed 5000'"
alias serial="tio /dev/ttyACM0"
