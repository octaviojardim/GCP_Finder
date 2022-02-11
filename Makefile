SHELL := /bin/bash

.ONESHELL:

.PHONY: install
install:
	python3 -m venv venv
	./venv/bin/pip install -r ./requirements.txt
	sudo apt install exiftool

clean:
	rm -r env


