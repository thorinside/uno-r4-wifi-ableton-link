ACLI   ?= arduino-cli
FQBN   := arduino:renesas_uno:unor4wifi
PORT   ?= /dev/cu.usbmodem*
SKETCH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))link_clock

.PHONY: compile upload deploy monitor

compile:
	$(ACLI) compile --fqbn $(FQBN) "$(SKETCH)"

upload:
	@PID=$$(lsof "$(PORT)" 2>/dev/null | awk 'NR>1 {print $$2}'); \
	if [ -n "$$PID" ]; then echo "Releasing port (PID $$PID)..."; kill "$$PID" && sleep 1; fi
	$(ACLI) upload --fqbn $(FQBN) --port "$(PORT)" "$(SKETCH)"

deploy: compile upload

monitor:
	$(ACLI) monitor --port "$(PORT)" --config baudrate=115200
