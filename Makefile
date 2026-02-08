VENV = .venv
PYTHON = $(VENV)/bin/python
UV = $(shell raise error if uv not installed)

.PHONY: all install clean run

all: install

install: $(VENV)
	@echo "Syncing dependencies..."
	uv sync

$(VENV): pyproject.toml
	@echo "Creating virtual environment and installing..."
	uv venv
	uv sync

clean:
	rm -rf $(VENV)
	rm -rf __pycache__
