# RC car ESP32-S3 bring-up. Run `make help`. Setup steps: docs/setup.md
# USB cable goes in the board's right-hand USB-C port, labelled "COM".
# Linux/macOS. On Windows use Git Bash and set VENV_BIN=.venv/Scripts.
PYTHON   ?= python3
VENV     := .venv
VENV_BIN ?= $(VENV)/bin
PY       := $(VENV_BIN)/python
PIO      := $(VENV_BIN)/pio
FW_DIR   := firmware
CLI      := $(PY) tools/rccar.py
ifeq ($(origin PORT),undefined)       # auto-detect once per make run
PORT     := $(shell $(CLI) find-port 2>/dev/null)
endif
PORT_ARG := $(if $(PORT),--port $(PORT),)

.DEFAULT_GOAL := help
.PHONY: help setup check detect ping sync-time tune clean build flash monitor \
        test test-throttle calibrate teleop session

help:            ## list targets
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  make %-14s %s\n", $$1, $$2}'

$(PY):
	$(PYTHON) -m venv $(VENV)

setup: $(PY)     ## create .venv and install platformio, pyserial, esptool
	$(PY) -m pip install -q --upgrade pip
	$(PY) -m pip install -q -r tools/requirements.txt

check: $(PY)     ## verify python packages, pio, serial permissions
	$(CLI) check-env

detect: $(PY)    ## find the ESP32 on USB, read chip id, query firmware
	$(CLI) $(PORT_ARG) detect

ping: $(PY)      ## check the firmware answers + USB round-trip time
	$(CLI) $(PORT_ARG) ping

sync-time: $(PY) ## set the ESP32 clock from this laptop
	$(CLI) $(PORT_ARG) sync-time

tune: $(PY)      ## push tuning.toml to the ESP32 and save it to flash
	$(CLI) $(PORT_ARG) tune

clean:           ## delete firmware/.pio build cache
	rm -rf $(FW_DIR)/.pio

build: clean     ## clean + compile firmware
	$(PIO) run -d $(FW_DIR)

flash: clean     ## clean + compile + upload firmware over the COM port
	$(PIO) run -d $(FW_DIR) -t upload $(if $(PORT),--upload-port $(PORT),)

monitor:         ## raw serial monitor (Ctrl+C to exit)
	$(PIO) device monitor -d $(FW_DIR) $(if $(PORT),--port $(PORT),)

test: $(PY)      ## steering sweep + failsafe check (motor stays off)
	$(CLI) $(PORT_ARG) test

test-throttle: $(PY) ## test + short motor run, WHEELS OFF THE GROUND
	$(CLI) $(PORT_ARG) test --throttle

calibrate: $(PY) ## interactive servo center/left/right calibration
	$(CLI) $(PORT_ARG) calibrate

teleop: $(PY)    ## keyboard driving: arrows = heading/speed
	$(CLI) $(PORT_ARG) teleop

session: $(PY)   ## interactive tuning/test session (tools/esp32_session.py)
	$(PY) tools/esp32_session.py $(PORT_ARG)
