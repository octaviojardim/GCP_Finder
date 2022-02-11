SHELL := /bin/bash

.ONESHELL:

.PHONY: install
install:
	python3 -m venv venv
	./venv/bin/pip install -r ./requirements.txt
	apt install exiftool

clean:
	rm -r env

